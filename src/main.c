/**
 * =========================================================================================
 * 🤖 物流机器人主控程序 (带详细调试注释版)
 * =========================================================================================
 * * [调试指南] - 🎮 控制方式速查表
 * * 1. 💻 电脑串口调试 (通过板载 USB 线，波特率: 115200，需勾选"发送新行/加回车换行")
 * - 适用场景：精细标定每个舵机的物理角度，防止机械臂互相干涉或打齿。
 * - 指令格式 (字母+空格+数值)：
 * - B 1500 : 调整底座 (Base) 脉宽到 1620us (有效范围约 500-2500)
 * - C 90   : 调整爪子 (Claw) 角度到 90度 (有效范围 0-180)
 * - M 150  : 调整中臂 (Mid) 角度到 150度 (有效范围 0-180)
 * - L 50   : 调整下臂 (Low) 角度到 50度 (有效范围 0-180)
 * - P      : 强制执行一次抓取全流程测试
 * * 2. 📱 蓝牙手机遥控 (连接蓝牙串口助手 APP，波特率: 9600)
 * - 适用场景：脱离电脑线缆，进行场地测试。
 * - 指令格式 (单字母，无需回车)：
 * - M (或 m) : 【模式切换】开机默认是视觉自动，按 M 切换为蓝牙手动，蜂鸣器会滴一声。
 * - L (或 l) : 【左转】底盘向左微调 (限手动模式有效)
 * - R (或 r) : 【右转】底盘向右微调 (限手动模式有效)
 * - C (或 c) : 【居中】底盘立刻回到中间位置 (1500us) (限手动模式有效)
 * - P (或 p) : 【抓取】一键触发抓取流程 (限手动模式有效)
 * * 3. 📷 视觉自动控制 (连接摄像头模块，波特率: 115200)
 * - 适用场景：全自动寻物抓取 (需要按 M 键切回自动模式才生效)。
 * - 触发条件：收到 "X:..., Y:..." 追踪坐标，Y 大于阈值时，或直接收到特征码指令。
 * =========================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "LOGISTICS_ROBOT";

// ====================================================================
// 1. 硬件引脚宏定义
// ====================================================================
#define CAM_TX_PIN (16)
#define CAM_RX_PIN (17)
#define BT_TX_PIN (11)
#define BT_RX_PIN (12)
#define BUZZER_PIN (19)
#define I2C_MASTER_SDA_IO (13)
#define I2C_MASTER_SCL_IO (14)
#define I2C_MASTER_NUM I2C_NUM_0

// ====================================================================
// 2. 通信与参数配置
// ====================================================================
#define PC_UART_PORT UART_NUM_0
#define CAM_UART_PORT UART_NUM_1
#define BT_UART_PORT UART_NUM_2
#define BUF_SIZE (1024)
const char *TRIGGER_CODE = "13-5-4011";

#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// ====================================================================
// 3. 抓取动作角度与参数配置 【新增提炼】
// ====================================================================

/* --- 爪子 (Channel 1) 角度 --- */
#define CLAW_OPEN 70   // 张开角度
#define CLAW_CLOSE 150 // 抓紧角度

/* --- 中臂 (Channel 2) 角度 --- */
#define MID_READY 150 // 准备下探的初始高度
#define MID_DOWN 180  // 完全降下接触物品的高度
#define MID_LIFT 170  // 抓到物品后微抬的高度
#define MID_SAFE 100  // 闲置/收回时的安全高度

/* --- 下臂 (Channel 3) 角度 --- */
#define LOW_READY 50 // 准备下探的初始高度
#define LOW_DOWN 150 // 完全降下接触物品的高度
#define LOW_LIFT 40  // 抓到物品后抬起的高度

/* --- 底座冲刺脉宽 (us) --- */


/* --- 动作速度参数 --- */
#define STEP_DELAY_MS 10 // 舵机每转动1度的等待时间(毫秒)。数字越大，动作越慢越平滑
// ====================================================================
// 4. 全局状态变量
// ====================================================================
float KP = 1.4f;
float KD = 0.5f;
int last_error_x = 0;
volatile bool is_target_lost = true;
volatile bool is_pickup_running = false;
volatile int target_x = 0;
volatile int target_y = 0;
const int DISTANCE_THRESHOLD = 200; // [调试指南] 视觉抓取阈值，如果离得太远就抓，把这个值调大；如果撞上了才抓，调小。

volatile bool is_manual_mode = true;
volatile int current_base_pwm = 1620;
const int MAX_PWM_STEP = 15;

// ====================================================================
// 5. 底层硬件驱动控制函数
// ====================================================================

void beep(int times, int duration_ms)
{
    for (int i = 0; i < times; i++)
    {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level(BUZZER_PIN, 0);
        if (i < times - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
        }
    }
}

void init_buzzer()
{
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);
}

esp_err_t pca9685_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_base, true);
    i2c_master_write_byte(cmd, on & 0xFF, true);
    i2c_master_write_byte(cmd, on >> 8, true);
    i2c_master_write_byte(cmd, off & 0xFF, true);
    i2c_master_write_byte(cmd, off >> 8, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

void init_pca9685()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    pca9685_write_byte(PCA9685_MODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    pca9685_write_byte(PCA9685_MODE1, 0x10);
    pca9685_write_byte(PCA9685_PRESCALE, 121);
    pca9685_write_byte(PCA9685_MODE1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    pca9685_write_byte(PCA9685_MODE1, 0xA1);

    ESP_LOGI(TAG, "PCA9685 初始化完成 (50Hz)");
}

void set_servo_pulse(uint8_t channel, uint32_t pulse_us)
{
    uint16_t off_tick = (pulse_us * 4096) / 20000;
    pca9685_set_pwm(channel, 0, off_tick);
}

/* * [调试指南] - 角度调整原理
 * 这里将 0~180度 映射为 500us~2500us 的脉冲宽度。
 * 不同的舵机存在机械公差，如果你输入 0度 发现它实际只转到了 10度，或者有异响（死区堵转），
 * 请不要强制将角度设为 0，建议将运动范围限制在 10~170度 之间保护舵机。
 */
void set_servo_angle(uint8_t channel, int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    uint32_t pulse = 500 + (angle * 2000 / 180);
    set_servo_pulse(channel, pulse);
}

void set_base_servo_pwm(int pulse_us)
{
    if (pulse_us < 500)
        pulse_us = 500;
    if (pulse_us > 2500)
        pulse_us = 2500;
    set_servo_pulse(0, pulse_us);
    current_base_pwm = pulse_us;
}

/**
 * @brief 智能平滑控制舵机转动 (自动判断正转还是反转)
 */
void smooth_move_angle(uint8_t channel, int start_angle, int end_angle, int step_delay)
{
    if (start_angle < end_angle)
    {
        for (int pos = start_angle; pos <= end_angle; pos++)
        {
            set_servo_angle(channel, pos);
            vTaskDelay(pdMS_TO_TICKS(step_delay));
        }
    }
    else
    {
        for (int pos = start_angle; pos >= end_angle; pos--)
        {
            set_servo_angle(channel, pos);
            vTaskDelay(pdMS_TO_TICKS(step_delay));
        }
    }
}
// ====================================================================
// 6. FreeRTOS 任务逻辑
// ====================================================================

void pickup_task(void *pvParameters) {
    is_pickup_running = true;
    ESP_LOGI(TAG, ">>>> 状态切换：开始抓取 <<<<");
    
    // 1. 机械臂到达准备姿态
    set_servo_angle(1, CLAW_OPEN);  
    set_servo_angle(2, MID_READY); 
    set_servo_angle(3, LOW_READY);  
    vTaskDelay(pdMS_TO_TICKS(1000));      

    // 2. 缓慢下探 (单纯靠中臂和下臂的角度伸展去够物品)
    smooth_move_angle(2, MID_READY, MID_DOWN, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(300));
    smooth_move_angle(3, LOW_READY, LOW_DOWN, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // 3. 爪子合拢抓取
    smooth_move_angle(1, CLAW_OPEN, CLAW_CLOSE, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(300)); 
    
    // 4. 抬起物品
    smooth_move_angle(3, LOW_DOWN, LOW_LIFT, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(300));
    smooth_move_angle(2, MID_DOWN, MID_LIFT, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(500)); // 稍微多停顿一下，确保抬稳
    
    // 5. 放下物品并松开爪子
    smooth_move_angle(1, CLAW_CLOSE, CLAW_OPEN, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 6. 机械臂收回安全位置
    smooth_move_angle(2, MID_LIFT, MID_SAFE, STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, ">>>> 抓取结束，释放锁 <<<<");
    beep(1, 800); 

    is_pickup_running = false;
    vTaskDelete(NULL); 
}
void uart_pc_task(void *pvParameters)
{
    uint8_t data[128];
    char line_buf[128];
    int line_pos = 0;

    while (1)
    {
        int len = uart_read_bytes(PC_UART_PORT, data, sizeof(data) - 1, pdMS_TO_TICKS(50));
        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                // 遇到回车/换行，视为一条指令结束
                if (data[i] == '\n' || data[i] == '\r')
                {
                    if (line_pos > 0)
                    {
                        line_buf[line_pos] = '\0';
                        char cmd;
                        int val;

                        // 解析 P 指令 (触发抓取)
                        if (strcmp(line_buf, "P") == 0 || strcmp(line_buf, "p") == 0)
                        {
                            if (!is_pickup_running)
                            {
                                ESP_LOGI(TAG, "💻 电脑指令：触发抓取！");
                                beep(2, 100);
                                xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                            }
                            else
                            {
                                ESP_LOGW(TAG, "⚠️ 抓取中，忽略指令");
                            }
                        }
                        // 解析 B, C, M, L 调角指令 (如 "B 1600" 或 "C 90")
                        else if (sscanf(line_buf, "%c %d", &cmd, &val) == 2)
                        {
                            if (cmd == 'B' || cmd == 'b')
                            {
                                set_base_servo_pwm(val);
                                ESP_LOGI(TAG, "💻 底座(0通道) 脉宽设为: %dus", val);
                            }
                            else if (cmd == 'C' || cmd == 'c')
                            {
                                set_servo_angle(1, val);
                                ESP_LOGI(TAG, "💻 爪子(1通道) 角度设为: %d度", val);
                            }
                            else if (cmd == 'M' || cmd == 'm')
                            {
                                set_servo_angle(2, val);
                                ESP_LOGI(TAG, "💻 中臂(2通道) 角度设为: %d度", val);
                            }
                            else if (cmd == 'L' || cmd == 'l')
                            {
                                set_servo_angle(3, val);
                                ESP_LOGI(TAG, "💻 下臂(3通道) 角度设为: %d度", val);
                            }
                        }
                        line_pos = 0;
                    }
                }
                else if (line_pos < sizeof(line_buf) - 1)
                {
                    line_buf[line_pos++] = data[i];
                }
            }
        }
    }
}

void uart_bt_task(void *pvParameters)
{
    uint8_t bt_data[128];
    while (1)
    {
        int len = uart_read_bytes(BT_UART_PORT, bt_data, sizeof(bt_data) - 1, pdMS_TO_TICKS(50));
        if (len > 0)
        {
            bt_data[len] = '\0';
            for (int i = 0; i < len; i++)
            {
                char cmd = bt_data[i];

                // M 键：用来强制夺权。防止视觉追踪一直在发指令导致你遥控不了
                if (cmd == 'M' || cmd == 'm')
                {
                    is_manual_mode = !is_manual_mode;
                    ESP_LOGI(TAG, "切换模式: %s", is_manual_mode ? "手动" : "自动(视觉)");
                    beep(1, 200);
                }

                // 以下四个遥控按键，必须在手动模式下才有效
                if (is_manual_mode && !is_pickup_running)
                {
                    if (cmd == 'L' || cmd == 'l')
                    { // 左微调 (底座步进+50)
                        current_base_pwm += 50;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'R' || cmd == 'r')
                    { // 右微调 (底座步进-50)
                        current_base_pwm -= 50;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'C' || cmd == 'c')
                    { // 立刻回中
                        current_base_pwm = 1620;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'P' || cmd == 'p')
                    { // 强制触发抓取
                        ESP_LOGI(TAG, "✋ 蓝牙遥控，触发抓取！");
                        beep(2, 100);
                        xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                    }
                }
            }
        }
    }
}

void uart_vision_task(void *pvParameters)
{
    uint8_t *cam_data = (uint8_t *)malloc(BUF_SIZE);
    char line_buf[128];
    int line_pos = 0;
    TickType_t last_target_time = xTaskGetTickCount();
    const int DEAD_ZONE = 15; // 视觉死区 (画面中央附近允许的一点误差，防止底盘无意义的抖动)

    while (1)
    {
        int len = uart_read_bytes(CAM_UART_PORT, cam_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                if (cam_data[i] == '\n' || cam_data[i] == '\r')
                {
                    if (line_pos > 0)
                    {
                        line_buf[line_pos] = '\0';

                        // 仅当没被蓝牙抢去控制权时，才听视觉模块的话
                        if (!is_manual_mode)
                        {
                            if (strstr(line_buf, TRIGGER_CODE) != NULL)
                            {
                                ESP_LOGI(TAG, "🚨 视觉收到指令，触发抓取！");
                                if (!is_pickup_running)
                                {
                                    beep(2, 100);
                                    xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                                }
                            }
                            else if (sscanf(line_buf, "X:%d, Y:%d", &target_x, &target_y) == 2)
                            {
                                last_target_time = xTaskGetTickCount();
                                if (is_target_lost)
                                {
                                    is_target_lost = false;
                                    ESP_LOGI(TAG, "👀 重新锁定目标");
                                }
                                if (!is_pickup_running)
                                {
                                    // 根据纵向坐标 (距离) 触发抓取
                                    if (target_y >= DISTANCE_THRESHOLD)
                                    {
                                        ESP_LOGW(TAG, "🔔 距离就绪，触发抓取！");
                                        beep(2, 100);
                                        xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                                    }
                                }
                                if (!is_pickup_running)
                                {
                                    // 视觉 PID 跟随逻辑
                                    int target_pwm = 1620;
                                    if (abs(target_x) > DEAD_ZONE)
                                    {
                                        int d_error = target_x - last_error_x;
                                        // 如果觉得底盘跟踪太冲，调小 KP (现为1.4)；如果太迟钝，调大 KP
                                        int speed_offset = (int)((target_x * KP) + (d_error * KD));
                                        if (speed_offset > 400)
                                            speed_offset = 400;
                                        if (speed_offset < -400)
                                            speed_offset = -400;
                                        target_pwm = 1620 - speed_offset;
                                    }
                                    last_error_x = target_x;

                                    if (target_pwm > current_base_pwm + MAX_PWM_STEP)
                                        current_base_pwm += MAX_PWM_STEP;
                                    else if (target_pwm < current_base_pwm - MAX_PWM_STEP)
                                        current_base_pwm -= MAX_PWM_STEP;
                                    else
                                        current_base_pwm = target_pwm;

                                    set_base_servo_pwm(current_base_pwm);
                                }
                            }
                        }
                        line_pos = 0;
                    }
                }
                else if (line_pos < sizeof(line_buf) - 1)
                {
                    line_buf[line_pos++] = cam_data[i];
                }
            }
        }

        // 超时保护：如果一秒钟没看到目标，认为是目标丢失
        if (!is_manual_mode && (xTaskGetTickCount() - last_target_time) * portTICK_PERIOD_MS > 1000)
        {
            if (!is_target_lost)
            {
                is_target_lost = true;
                last_error_x = 0;
                ESP_LOGE(TAG, "⚠️ 目标丢失！");
                if (!is_pickup_running)
                    beep(3, 50);
            }
            if (current_base_pwm != 1620 && !is_pickup_running)
            {
                // 丢失目标后，底盘平滑归中，防止瞬间抽搐
                if (1620 > current_base_pwm + MAX_PWM_STEP)
                    current_base_pwm += MAX_PWM_STEP;
                else if (1620 < current_base_pwm - MAX_PWM_STEP)
                    current_base_pwm -= MAX_PWM_STEP;
                else
                    current_base_pwm = 1620;
                set_base_servo_pwm(current_base_pwm);
            }
        }
    }
}

// ====================================================================
// 6. 系统内核入口
// ====================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "系统启动中...");

    init_buzzer();
    init_pca9685();

    // [调试指南] 上电保护：开机时自动把所有舵机归零/归中，防止一通电机械臂乱甩打坏零件
    set_servo_pulse(0, 1620); // 底座 (Base) 居中
    set_servo_pulse(1, 1500); // 爪子 (Claw) 中间态
    set_servo_pulse(2, 1722); // 中臂 (Mid) 垂直
    set_servo_pulse(3, 1389); // 下臂 (Low) 垂直

    // 初始化电脑串口
    uart_driver_install(PC_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 初始化视觉模块串口
    uart_config_t cam_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CAM_UART_PORT, &cam_uart_config);
    uart_set_pin(CAM_UART_PORT, CAM_TX_PIN, CAM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CAM_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 初始化蓝牙串口
    uart_config_t bt_uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(BT_UART_PORT, &bt_uart_config);
    uart_set_pin(BT_UART_PORT, BT_TX_PIN, BT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(BT_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 启动多任务
    xTaskCreate(uart_pc_task, "uart_pc_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_vision_task, "uart_vision_task", 8192, NULL, 5, NULL);
    xTaskCreate(uart_bt_task, "uart_bt_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "系统初始化完毕，准备就绪。");
    beep(1, 500);
}