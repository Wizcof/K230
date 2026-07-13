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

/* --- 视觉模块串口通信引脚 --- */
#define CAM_TX_PIN (16)
#define CAM_RX_PIN (17)

/* --- 蓝牙模块串口通信引脚 【新增】 --- */
// 请将蓝牙模块的 TX 接 22，RX 接 23
#define BT_TX_PIN (22)
#define BT_RX_PIN (23)

/* --- 蜂鸣器引脚 --- */
#define BUZZER_PIN (19)

/* --- 舵机 PWM 控制引脚 --- */
#define BASE_PIN (5)
#define CLAW_PIN (4)
#define MID_ARM_PIN (18)
#define LOW_ARM_PIN (15)

// ====================================================================
// 2. 通信与算法参数配置
// ====================================================================
#define CAM_UART_PORT UART_NUM_1
#define BT_UART_PORT UART_NUM_2 // 【新增】蓝牙使用 UART2
#define BUF_SIZE (1024)
const char *TRIGGER_CODE = "13-5-4011";

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT

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

// 【新增】底盘控制与模式变量
volatile bool is_manual_mode = false; // 默认自动(视觉)模式
volatile int current_base_pwm = 1500; // 提取为全局变量以便双向控制
const int MAX_PWM_STEP = 15;          // 舵机平滑步进值

// ====================================================================
// 4. 底层硬件驱动控制函数
// ====================================================================

uint32_t us_to_duty(uint32_t pulse_us)
{
    return (pulse_us * 16384) / 20000;
}

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

void init_servos()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    int pins[] = {BASE_PIN, CLAW_PIN, MID_ARM_PIN, LOW_ARM_PIN};
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_MODE,
            .channel = LEDC_CHANNEL_0 + i,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = pins[i],
            .duty = us_to_duty(1500),
            .hpoint = 0};
        ledc_channel_config(&ledc_channel);
    }
}

void set_servo_angle(ledc_channel_t channel, int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    uint32_t pulse = 500 + (angle * 2000 / 180);
    ledc_set_duty(LEDC_MODE, channel, us_to_duty(pulse));
    ledc_update_duty(LEDC_MODE, channel);
}

void set_base_servo_pwm(int pulse_us)
{
    // 限制舵机脉宽范围防止卡死 (500us - 2500us)
    if (pulse_us < 500)
        pulse_us = 500;
    if (pulse_us > 2500)
        pulse_us = 2500;

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, us_to_duty(pulse_us));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

// ====================================================================
// 5. FreeRTOS 任务逻辑
// ====================================================================

void pickup_task(void *pvParameters)
{
    is_pickup_running = true;
    ESP_LOGI(TAG, ">>>> 状态切换：开始抓取 <<<<");

    set_servo_angle(LEDC_CHANNEL_1, 60);  // 爪子张开
    set_servo_angle(LEDC_CHANNEL_2, 150); // 中臂就位
    set_servo_angle(LEDC_CHANNEL_3, 50);  // 下臂俯低
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int pos = 150; pos <= 180; pos++)
    {
        set_servo_angle(LEDC_CHANNEL_2, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos = 50; pos <= 110; pos++)
    {
        set_servo_angle(LEDC_CHANNEL_3, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos = 60; pos <= 170; pos++)
    {
        set_servo_angle(LEDC_CHANNEL_1, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    // 抬起
    for (int pos = 110; pos >= 40; pos--)
    {
        set_servo_angle(LEDC_CHANNEL_3, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int pos = 180; pos >= 170; pos--)
    {
        set_servo_angle(LEDC_CHANNEL_2, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    // 底座前冲与复位补偿
    set_base_servo_pwm(1900);
    vTaskDelay(pdMS_TO_TICKS(1300));
    set_base_servo_pwm(1500);
    current_base_pwm = 1500; // 同步全局变量
    vTaskDelay(pdMS_TO_TICKS(500));

    // 放下/复位
    for (int pos = 170; pos >= 60; pos--)
    {
        set_servo_angle(LEDC_CHANNEL_1, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 底座后退
    set_base_servo_pwm(1150);
    vTaskDelay(pdMS_TO_TICKS(1300));
    set_base_servo_pwm(1500);
    current_base_pwm = 1500; // 同步全局变量

    for (int pos = 170; pos >= 90; pos--)
    {
        set_servo_angle(LEDC_CHANNEL_2, pos);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, ">>>> 抓取结束，释放锁 <<<<");
    beep(1, 800);

    is_pickup_running = false;
    vTaskDelete(NULL);
}

// 【新增】蓝牙控制任务
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

                // M: 切换模式 (Manual/Auto)
                if (cmd == 'M' || cmd == 'm')
                {
                    is_manual_mode = !is_manual_mode;
                    ESP_LOGI(TAG, "切换模式: %s", is_manual_mode ? "手动" : "自动(视觉)");
                    beep(1, 200);
                }

                // 以下指令仅在手动模式下生效
                if (is_manual_mode && !is_pickup_running)
                {
                    if (cmd == 'L' || cmd == 'l')
                    {
                        // 底盘左转 (脉宽增加)
                        current_base_pwm += 50;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'R' || cmd == 'r')
                    {
                        // 底盘右转 (脉宽减小)
                        current_base_pwm -= 50;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'C' || cmd == 'c')
                    {
                        // 底盘居中
                        current_base_pwm = 1500;
                        set_base_servo_pwm(current_base_pwm);
                    }
                    else if (cmd == 'P' || cmd == 'p')
                    {
                        // 触发抓取
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

    const int DEAD_ZONE = 15;

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

                        // 仅在自动模式下响应视觉触发指令
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
                                    if (target_y >= DISTANCE_THRESHOLD)
                                    {
                                        ESP_LOGW(TAG, "🔔 距离就绪，触发抓取！");
                                        beep(2, 100);
                                        xTaskCreate(pickup_task, "pickup_task", 4096, NULL, 5, NULL);
                                    }
                                }

                                // 视觉 PID 追踪底座
                                if (!is_pickup_running)
                                {
                                    int target_pwm = 1500;
                                    if (abs(target_x) > DEAD_ZONE)
                                    {
                                        int d_error = target_x - last_error_x;
                                        int speed_offset = (int)((target_x * KP) + (d_error * KD));

                                        if (speed_offset > 400)
                                            speed_offset = 400;
                                        if (speed_offset < -400)
                                            speed_offset = -400;
                                        target_pwm = 1500 - speed_offset;
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

        // 目标丢失检测 (仅在自动模式生效)
        if (!is_manual_mode && (xTaskGetTickCount() - last_target_time) * portTICK_PERIOD_MS > 1000)
        {
            if (!is_target_lost)
            {
                is_target_lost = true;
                last_error_x = 0;
                ESP_LOGE(TAG, "⚠️ 目标丢失！");

                if (!is_pickup_running)
                {
                    beep(3, 50);
                }
            }
            if (current_base_pwm != 1500 && !is_pickup_running)
            {
                if (1500 > current_base_pwm + MAX_PWM_STEP)
                    current_base_pwm += MAX_PWM_STEP;
                else if (1500 < current_base_pwm - MAX_PWM_STEP)
                    current_base_pwm -= MAX_PWM_STEP;
                else
                    current_base_pwm = 1500;
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

    init_servos();
    init_buzzer();

    // 初始化视觉模块串口 (UART1)
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

    // 初始化蓝牙模块串口 (UART2)
    uart_config_t bt_uart_config = {
        .baud_rate = 9600, // HC-05 等模块默认通常为 9600，按需修改
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(BT_UART_PORT, &bt_uart_config);
    uart_set_pin(BT_UART_PORT, BT_TX_PIN, BT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(BT_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 创建任务
    xTaskCreate(uart_vision_task, "uart_vision_task", 8192, NULL, 5, NULL);
    xTaskCreate(uart_bt_task, "uart_bt_task", 4096, NULL, 5, NULL); // 【新增】蓝牙任务

    ESP_LOGI(TAG, "系统初始化完毕，准备就绪。");
    beep(1, 500);
}