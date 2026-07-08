#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "LOGISTICS_ROBOT";

// ====================================================================
// 1. 硬件引脚宏定义 (Hardware Pin Assignments)
// ====================================================================

/* --- 串口通信引脚 (避免了原18引脚冲突) --- */
#define CAM_TX_PIN      (16) // ESP32 TXD -> 连接 OpenMV 的 RXD
#define CAM_RX_PIN      (17) // ESP32 RXD <- 连接 OpenMV 的 TXD

/* --- 舵机 PWM 控制引脚 --- */
#define BASE_PIN        (5)  // 底座360度伺服电机引脚 -> 对应 LEDC 通道 0
#define CLAW_PIN        (4)  // 机械臂爪子舵机引脚   -> 对应 LEDC 通道 1
#define MID_ARM_PIN     (18) // 机械臂中臂舵机引脚   -> 对应 LEDC 通道 2
#define LOW_ARM_PIN     (15) // 机械臂下臂舵机引脚   -> 对应 LEDC 通道 3

/* --- 直流电机驱动引脚 (高低电平控制正反转) --- */
#define L1A             (25) // 左前轮控制端 A
#define L1B             (26) // 左前轮控制端 B
#define L2A             (33) // 左后轮控制端 A
#define L2B             (32) // 左后轮控制端 B
#define R1A             (27) // 右前轮控制端 A
#define R1B             (14) // 右前轮控制端 B
#define R2A             (22) // 右后轮控制端 A
#define R2B             (23) // 右后轮控制端 B

// ====================================================================
// 2. 通信与算法参数配置
// ====================================================================
#define CAM_UART_PORT   UART_NUM_1      // 使用 ESP32 的硬件串口 1
#define BUF_SIZE        (1024)          // 串口接收缓冲区大小
const char* TRIGGER_CODE = "13-5-4011"; // OpenMV 发送的具体动作触发码

/* --- LEDC 定时器配置 (针对模拟舵机：50Hz 频率，20ms 周期) --- */
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_14_BIT // 14位分辨率，占空比数值范围 0 ~ 16383

// ====================================================================
// 3. 全局状态变量
// ====================================================================
float KP = 1.4f;                        // PD 算法：比例系数
float KD = 0.5f;                        // PD 算法：微分系数
int last_error_x = 0;                   // 记录上一次的 X 轴偏差，用于计算微分项值
volatile bool is_target_lost = true;    // 视觉目标是否丢失标志
volatile bool is_pickup_running = false;// 机械臂当前是否正在执行抓取动作（状态锁）
volatile int target_x = 0;              // 图像中目标的 X 坐标偏差
volatile int target_y = 0;              // 图像中目标的 Y 坐标 (常用于估算距离或高低)

// 距离触发阈值：当 Y 坐标达到或超过此数值，判定车辆靠得足够近，可以停车抓取
const int DISTANCE_THRESHOLD = 200; 

// 函数声明
void ting();
void qianjing();

// ====================================================================
// 4. 底层硬件驱动控制函数
// ====================================================================

/**
 * @brief 将时间宽度(微秒 us)转换为 14位 LEDC 寄存器所需的精确占空比数值
 * 计算公式: (pulse_us / 20000us) * 2^14
 */
uint32_t us_to_duty(uint32_t pulse_us) {
    return (pulse_us * 16384) / 20000;
}

/**
 * @brief 初始化直流电机控制所需的普通 GPIO 引脚
 */
void init_motor_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,   // 禁用中断
        .mode = GPIO_MODE_OUTPUT,         // 设置为推挽输出模式
        // 利用位掩码一次性配置好所有 8 个车轮控制引脚
        .pin_bit_mask = (1ULL<<L1A)|(1ULL<<L1B)|(1ULL<<L2A)|(1ULL<<L2B)|
                        (1ULL<<R1A)|(1ULL<<R1B)|(1ULL<<R2A)|(1ULL<<R2B),
        .pull_down_en = 0,                // 禁用下拉
        .pull_up_en = 0                   // 禁用上拉
    };
    gpio_config(&io_conf);
    ting();                               // 初始化完成后默认电机全部停止
}

/**
 * @brief 配置 LEDC 外设，初始化底座及机械臂的 4 个舵机 PWM 通道
 */
void init_servos() {
    // 配置时钟源和频率
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50,           // 50Hz 适合绝大多数标准舵机
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 将 4 个舵机引脚顺序排列，依次绑定到 LEDC_CHANNEL_0 至 CHANNEL_3
    int pins[] = {BASE_PIN, CLAW_PIN, MID_ARM_PIN, LOW_ARM_PIN};
    for(int i=0; i<4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_0 + i, // 依次分配通道 0, 1, 2, 3
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = pins[i],
            .duty           = us_to_duty(1500),   // 默认初始中置脉宽 1500us (底座停止)
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }
}

/**
 * @brief 设置 180 度标准角度舵机的目标角度
 * @param channel 选择对应的 LEDC 通道 (CHANNEL_1:爪子, CHANNEL_2:中臂, CHANNEL_3:下臂)
 * @param angle   目标角度 (0 ~ 180 度映射至脉宽 500us ~ 2500us)
 */
void set_servo_angle(ledc_channel_t channel, int angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    uint32_t pulse = 500 + (angle * 2000 / 180); // 线性转换公式
    ledc_set_duty(LEDC_MODE, channel, us_to_duty(pulse));
    ledc_update_duty(LEDC_MODE, channel);
}

/**
 * @brief 设置底座 360 度连续旋转伺服电机的 PWM 脉宽值
 * @param pulse_us 脉宽时间 (1500us 左右停止，大于 1500 正转，小于 1500 反转)
 */
void set_base_servo_pwm(int pulse_us) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, us_to_duty(pulse_us));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

// ====================================================================
// 5. 底盘运动状态控制
// ====================================================================
void ting() { // 车辆全停
    gpio_set_level(L1A, 0); gpio_set_level(L1B, 0);
    gpio_set_level(L2A, 0); gpio_set_level(L2B, 0);
    gpio_set_level(R1A, 0); gpio_set_level(R1B, 0);
    gpio_set_level(R2A, 0); gpio_set_level(R2B, 0);
}

void qianjing() { // 车辆直线前进
    gpio_set_level(L1A, 1); gpio_set_level(L1B, 0);
    gpio_set_level(L2A, 1); gpio_set_level(L2B, 0);
    gpio_set_level(R1A, 1); gpio_set_level(R1B, 0);
    gpio_set_level(R2A, 1); gpio_set_level(R2B, 0);
}

// ====================================================================
// 6. FreeRTOS 任务逻辑
// ====================================================================

/**
 * @brief FreeRTOS 独立抓取任务：接管控制权，执行有序的机械臂动作序列
 * 利用 vTaskDelay 替代原本会锁死系统的 delay()，确保底层系统不丢失心跳
 */
void pickup_task(void *pvParameters) {
    is_pickup_running = true;
    ting(); // 必须安全停车才能抓取
    ESP_LOGI(TAG, ">>>> 状态切换：车辆已停车，开始物理抓取流程 <<<<");

    // 机械臂姿态初始化归位
    set_servo_angle(LEDC_CHANNEL_1, 60);  // 爪子张开 (CH1)
    set_servo_angle(LEDC_CHANNEL_2, 150); // 中臂就位 (CH2)
    set_servo_angle(LEDC_CHANNEL_3, 50);  // 下臂俯低 (CH3)
    vTaskDelay(pdMS_TO_TICKS(1000));      // 强延时 1000ms 等待舵机到位

    // 动作序列逐帧输出，防止动作过猛造成车身倾翻
    for (int pos=150; pos<=180; pos++) { set_servo_angle(LEDC_CHANNEL_2, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=50; pos<=110; pos++)  { set_servo_angle(LEDC_CHANNEL_3, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=60; pos<=170; pos++)  { set_servo_angle(LEDC_CHANNEL_1, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300)); // 闭合爪子
    
    // 抬起与收回
    for (int pos=110; pos>=40; pos--)  { set_servo_angle(LEDC_CHANNEL_3, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=180; pos>=170; pos--) { set_servo_angle(LEDC_CHANNEL_2, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, ">>>> 状态切换：抓取流程结束，释放控制锁 <<<<");
    is_pickup_running = false;
    vTaskDelete(NULL); // 执行完毕，自动销毁当前动态创建的任务，释放内存
}

/**
 * @brief 数据解析与中枢控制主任务：负责解析摄像头串口，执行PD追踪和多条件抓取触发机制
 */
void uart_vision_task(void *pvParameters) {
    uint8_t *cam_data = (uint8_t *) malloc(BUF_SIZE);
    char line_buf[128];
    int line_pos = 0;
    TickType_t last_target_time = 0;

    int current_pwm = 1500;       // 底座当前实时输出的脉宽值
    const int MAX_PWM_STEP = 15;  // 限制每周期最大脉宽阶跃，实现底座调速平滑去抖
    const int DEAD_ZONE = 15;     // 视觉中心死区（偏差在此像素内视为对准，底座不转动）

    while (1) {
        // 非阻塞读取来自 OpenMV 的串口原始字节串
        int len = uart_read_bytes(CAM_UART_PORT, cam_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (cam_data[i] == '\n' || cam_data[i] == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0'; // 截断字符串
                        
                        // 条件1：精准检测到了无线/OpenMV 下发字符触发码
                        if (strstr(line_buf, TRIGGER_CODE) != NULL) {
                            ESP_LOGI(TAG, "🚨 触发匹配：收到串行控制命令码！");
                            if (!is_pickup_running) {
                                xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                            }
                        } 
                        // 条件2：解析正常视觉物体的实时像素误差坐标（形如 "X:23, Y:185"）
                        else if (sscanf(line_buf, "X:%d, Y:%d", &target_x, &target_y) == 2) {
                            last_target_time = xTaskGetTickCount(); // 刷新生存时间
                            is_target_lost = false;

                            // 🔄【自主跟随与触发】：非抓取状态下，判断前行与靠拢逻辑
                            if (!is_pickup_running) {
                                if (target_y >= DISTANCE_THRESHOLD) {
                                    // 距离达到临界，判定抵达终点，直接启动抓取
                                    ESP_LOGW(TAG, "🔔 触发匹配：物体已接近安全抓取点 (Y 轴阈值: %d <= %d)", DISTANCE_THRESHOLD, target_y);
                                    xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                                } else {
                                    // 尚未接近，车体继续前行迫近目标
                                    qianjing();
                                }
                            }

                            // 🔄【底座旋转 PD 追踪阻尼计算】
                            if (!is_pickup_running) {
                                int target_pwm = 1500;
                                if (abs(target_x) > DEAD_ZONE) {
                                    int d_error = target_x - last_error_x; // 计算当前差值（微分项）
                                    int speed_offset = (int)((target_x * KP) + (d_error * KD)); // PD 公式
                                    
                                    if (speed_offset > 400) speed_offset = 400;   // 电流极限限幅
                                    if (speed_offset < -400) speed_offset = -400;
                                    target_pwm = 1500 - speed_offset;             // 修正输出
                                }
                                last_error_x = target_x;

                                // 平滑限速发生器，防止运动剧烈产生振荡
                                if (target_pwm > current_pwm + MAX_PWM_STEP) current_pwm += MAX_PWM_STEP; 
                                else if (target_pwm < current_pwm - MAX_PWM_STEP) current_pwm -= MAX_PWM_STEP; 
                                else current_pwm = target_pwm;    
                                
                                set_base_servo_pwm(current_pwm); // 输出最终驱动
                            }
                        }
                        line_pos = 0; // 重置行指针
                    }
                } else if (line_pos < sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = cam_data[i];
                }
            }
        }

        // 超时保活检测：若超过 1 秒未在串口收到任何有效定位，说明目标丢失
        if ((xTaskGetTickCount() - last_target_time) * portTICK_PERIOD_MS > 1000) {
            if (!is_target_lost) {
                is_target_lost = true;
                last_error_x = 0;
                ESP_LOGE(TAG, "⚠️ 警告：追踪目标丢失，整车停车进入安全就绪态");
                if(!is_pickup_running) ting(); // 安全机制：立即停走
            }
            if (current_pwm != 1500 && !is_pickup_running) {
                // 底座平滑强制减速回中停止
                if (1500 > current_pwm + MAX_PWM_STEP) current_pwm += MAX_PWM_STEP;
                else if (1500 < current_pwm - MAX_PWM_STEP) current_pwm -= MAX_PWM_STEP;
                else current_pwm = 1500;
                set_base_servo_pwm(current_pwm);
            }
        }
    }
}

// ====================================================================
// 7. 系统内核入口
// ====================================================================
void app_main(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "  智能物流机器人 ESP-IDF 原生固件初始化中...  ");
    ESP_LOGI(TAG, "==========================================");
    
    // 基础硬件驱动层建立
    init_motor_gpio();
    init_servos();

    // 硬件配置底层异步高级串口1 (UART1)
    uart_config_t cam_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CAM_UART_PORT, &cam_uart_config);
    // 绑定物理复用管脚
    uart_set_pin(CAM_UART_PORT, CAM_TX_PIN, CAM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CAM_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 将视觉数据接收与控制闭环逻辑挂载至 FreeRTOS 内核，给以 5 级中优先级运行
    xTaskCreate(uart_vision_task, "uart_vision_task", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "固件内核初始化完毕，控制流移交任务总线。");
}