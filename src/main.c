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
// 1. 硬件引脚宏定义 
// ====================================================================

/* --- 串口通信引脚 --- */
#define CAM_TX_PIN      (16) 
#define CAM_RX_PIN      (17) 

/* --- 蜂鸣器引脚 【新增】 --- */
#define BUZZER_PIN      (19) // 连接有源蜂鸣器的信号(I/O)脚

/* --- 舵机 PWM 控制引脚 --- */
#define BASE_PIN        (5)  
#define CLAW_PIN        (4)  
#define MID_ARM_PIN     (18) 
#define LOW_ARM_PIN     (15) 

/* --- 直流电机驱动引脚 --- */
#define L1A 25
#define L1B 26
#define L2A 33
#define L2B 32
#define R1A 27
#define R1B 14
#define R2A 22
#define R2B 23

// ====================================================================
// 2. 通信与算法参数配置
// ====================================================================
#define CAM_UART_PORT   UART_NUM_1      
#define BUF_SIZE        (1024)          
const char* TRIGGER_CODE = "13-5-4011"; 

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_14_BIT 

// ====================================================================
// 3. 全局状态变量
// ====================================================================
float KP = 1.4f;                        
float KD = 0.5f;                        
int last_error_x = 0;                   
volatile bool is_target_lost = true;    
volatile bool is_pickup_running = false;
volatile int target_x = 0;              
volatile int target_y = 0;              
const int DISTANCE_THRESHOLD = 200; 

// 函数声明
void ting();
void qianjing();

// ====================================================================
// 4. 底层硬件驱动控制函数
// ====================================================================

uint32_t us_to_duty(uint32_t pulse_us) {
    return (pulse_us * 16384) / 20000;
}

/**
 * @brief 蜂鸣器发声函数 【新增】
 * @param times 响几次
 * @param duration_ms 每次响的时间及间隔时间
 */
void beep(int times, int duration_ms) {
    for(int i = 0; i < times; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level(BUZZER_PIN, 0);
        if (i < times - 1) { // 最后一次响完不加额外间隔
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
        }
    }
}

/**
 * @brief 初始化蜂鸣器 GPIO 【新增】
 */
void init_buzzer() {
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0); // 默认关闭
}

void init_motor_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,   
        .mode = GPIO_MODE_OUTPUT,         
        .pin_bit_mask = (1ULL<<L1A)|(1ULL<<L1B)|(1ULL<<L2A)|(1ULL<<L2B)|
                        (1ULL<<R1A)|(1ULL<<R1B)|(1ULL<<R2A)|(1ULL<<R2B),
        .pull_down_en = 0,                
        .pull_up_en = 0                   
    };
    gpio_config(&io_conf);
    ting();                               
}

void init_servos() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50,           
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    int pins[] = {BASE_PIN, CLAW_PIN, MID_ARM_PIN, LOW_ARM_PIN};
    for(int i=0; i<4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_0 + i, 
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = pins[i],
            .duty           = us_to_duty(1500),   
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }
}

void set_servo_angle(ledc_channel_t channel, int angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    uint32_t pulse = 500 + (angle * 2000 / 180); 
    ledc_set_duty(LEDC_MODE, channel, us_to_duty(pulse));
    ledc_update_duty(LEDC_MODE, channel);
}

void set_base_servo_pwm(int pulse_us) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, us_to_duty(pulse_us));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

// ====================================================================
// 5. 底盘运动状态控制
// ====================================================================
void ting() { 
    gpio_set_level(L1A, 0); gpio_set_level(L1B, 0);
    gpio_set_level(L2A, 0); gpio_set_level(L2B, 0);
    gpio_set_level(R1A, 0); gpio_set_level(R1B, 0);
    gpio_set_level(R2A, 0); gpio_set_level(R2B, 0);
}

void qianjing() { 
    gpio_set_level(L1A, 1); gpio_set_level(L1B, 0);
    gpio_set_level(L2A, 1); gpio_set_level(L2B, 0);
    gpio_set_level(R1A, 1); gpio_set_level(R1B, 0);
    gpio_set_level(R2A, 1); gpio_set_level(R2B, 0);
}

// ====================================================================
// 6. FreeRTOS 任务逻辑
// ====================================================================

void pickup_task(void *pvParameters) {
    is_pickup_running = true;
    ting(); 
    ESP_LOGI(TAG, ">>>> 状态切换：开始抓取 <<<<");
    
    // 【修改】抓取前底座补正与前冲逻辑 (修复了之前遗漏的底座微调)
    set_servo_angle(LEDC_CHANNEL_1, 60);  // 爪子张开
    set_servo_angle(LEDC_CHANNEL_2, 150); // 中臂就位
    set_servo_angle(LEDC_CHANNEL_3, 50);  // 下臂俯低
    vTaskDelay(pdMS_TO_TICKS(1000));      

    for (int pos=150; pos<=180; pos++) { set_servo_angle(LEDC_CHANNEL_2, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=50; pos<=110; pos++)  { set_servo_angle(LEDC_CHANNEL_3, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=60; pos<=170; pos++)  { set_servo_angle(LEDC_CHANNEL_1, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300)); 
    
    // 抬起
    for (int pos=110; pos>=40; pos--)  { set_servo_angle(LEDC_CHANNEL_3, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos=180; pos>=170; pos--) { set_servo_angle(LEDC_CHANNEL_2, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));

    // 【新增】底座前冲与复位补偿 (根据你原始代码的 setBaseSpeed(0.8) 逻辑恢复)
    set_base_servo_pwm(1900); // 1.5 + 0.8*0.5 -> ~1900us
    vTaskDelay(pdMS_TO_TICKS(1300));
    set_base_servo_pwm(1500);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 放下/复位
    for (int pos=170; pos>=60; pos--) { set_servo_angle(LEDC_CHANNEL_1, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 底座后退
    set_base_servo_pwm(1150); // 1.5 - 0.7*0.5 -> ~1150us
    vTaskDelay(pdMS_TO_TICKS(1300));
    set_base_servo_pwm(1500);

    for (int pos=170; pos>=90; pos--) { set_servo_angle(LEDC_CHANNEL_2, pos); vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, ">>>> 抓取结束，释放锁 <<<<");
    
    beep(1, 800); // 【新增】抓取完成长鸣

    is_pickup_running = false;
    vTaskDelete(NULL); 
}

void uart_vision_task(void *pvParameters) {
    uint8_t *cam_data = (uint8_t *) malloc(BUF_SIZE);
    char line_buf[128];
    int line_pos = 0;
    TickType_t last_target_time = xTaskGetTickCount(); // 初始化防止开机误报丢失

    int current_pwm = 1500;       
    const int MAX_PWM_STEP = 15;  
    const int DEAD_ZONE = 15;     

    while (1) {
        int len = uart_read_bytes(CAM_UART_PORT, cam_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (cam_data[i] == '\n' || cam_data[i] == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0'; 
                        
                        if (strstr(line_buf, TRIGGER_CODE) != NULL) {
                            ESP_LOGI(TAG, "🚨 收到指令，触发抓取！");
                            if (!is_pickup_running) {
                                beep(2, 100); // 【新增】指令触发抓取：滴-滴
                                xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                            }
                        } 
                        else if (sscanf(line_buf, "X:%d, Y:%d", &target_x, &target_y) == 2) {
                            last_target_time = xTaskGetTickCount(); 
                            
                            if (is_target_lost) {
                                is_target_lost = false;
                                ESP_LOGI(TAG, "👀 重新锁定目标");
                            }

                            if (!is_pickup_running) {
                                if (target_y >= DISTANCE_THRESHOLD) {
                                    ESP_LOGW(TAG, "🔔 距离就绪，触发抓取！");
                                    beep(2, 100); // 【新增】距离满足触发抓取：滴-滴
                                    xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                                } else {
                                    qianjing();
                                }
                            }

                            if (!is_pickup_running) {
                                int target_pwm = 1500;
                                if (abs(target_x) > DEAD_ZONE) {
                                    int d_error = target_x - last_error_x; 
                                    int speed_offset = (int)((target_x * KP) + (d_error * KD)); 
                                    
                                    if (speed_offset > 400) speed_offset = 400;   
                                    if (speed_offset < -400) speed_offset = -400;
                                    target_pwm = 1500 - speed_offset;             
                                }
                                last_error_x = target_x;

                                if (target_pwm > current_pwm + MAX_PWM_STEP) current_pwm += MAX_PWM_STEP; 
                                else if (target_pwm < current_pwm - MAX_PWM_STEP) current_pwm -= MAX_PWM_STEP; 
                                else current_pwm = target_pwm;    
                                
                                set_base_servo_pwm(current_pwm); 
                            }
                        }
                        line_pos = 0; 
                    }
                } else if (line_pos < sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = cam_data[i];
                }
            }
        }

        // 目标丢失检测
        if ((xTaskGetTickCount() - last_target_time) * portTICK_PERIOD_MS > 1000) {
            if (!is_target_lost) {
                is_target_lost = true;
                last_error_x = 0;
                ESP_LOGE(TAG, "⚠️ 目标丢失！");
                
                if(!is_pickup_running) {
                    ting(); 
                    beep(3, 50); // 【新增】丢失报警：急促三声滴滴滴
                }
            }
            if (current_pwm != 1500 && !is_pickup_running) {
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
    ESP_LOGI(TAG, "系统启动中...");
    
    init_motor_gpio();
    init_servos();
    init_buzzer(); // 【新增】初始化蜂鸣器

    uart_config_t cam_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CAM_UART_PORT, &cam_uart_config);
    uart_set_pin(CAM_UART_PORT, CAM_TX_PIN, CAM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CAM_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(uart_vision_task, "uart_vision_task", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "系统初始化完毕，准备就绪。");
    beep(1, 500); // 【新增】启动成功，长鸣一声
}