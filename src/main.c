#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// ---------------- 定义硬件引脚 ----------------
#define CAM_TX_PIN      (17) // ESP32 TX -> 摄像头 RX
#define CAM_RX_PIN      (18) // ESP32 RX <- 摄像头 TX
#define SERVO_PIN       (4)  // 舵机 PWM 控制引脚

// ---------------- 定义通信与控制参数 ----------------
#define CAM_UART_PORT   UART_NUM_1
#define CTRL_UART_PORT  UART_NUM_0   // 新增：用于接收调参指令的串口 (默认 USB 串口)
#define BUF_SIZE        (1024)

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_14_BIT 
#define LEDC_FREQUENCY  (50)              

static const char *TAG = "TRACKER";

// ---------------- 控制算法参数 (PD控制器) ----------------
const int DEAD_ZONE = 15;        // 死区，误差在此范围内时舵机停止
float KP = 1.4f;                 // 【修改】比例系数 (去掉 const 变为可调变量)
float KD = 0.5f;                 // 【修改】微分系数 (去掉 const 变为可调变量)
const uint32_t TARGET_TIMEOUT_MS = 1000; // 目标丢失超时时间(毫秒)

// 全局变量用于存储上一次的误差，供微分计算使用
int last_error_x = 0;

// 将脉宽(微秒)转换为 LEDC 寄存器占空比值
uint32_t us_to_duty(uint32_t pulse_us) {
    return (pulse_us * 16384) / 20000;
}

// 初始化 UART 外设
void init_uart(void) {
    // 1. 初始化摄像头通信串口
    uart_config_t cam_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CAM_UART_PORT, &cam_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CAM_UART_PORT, CAM_TX_PIN, CAM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CAM_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));

    // 2. 【新增】初始化控制台串口 (用于接收调参指令)
    uart_config_t ctrl_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CTRL_UART_PORT, &ctrl_uart_config));
    // 安装驱动以便可以接收数据
    ESP_ERROR_CHECK(uart_driver_install(CTRL_UART_PORT, 256 * 2, 0, 0, NULL, 0));
}

// 初始化 LEDC 输出 PWM
void init_servo(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PIN,
        .duty           = us_to_duty(1500), 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// 设置 360 度舵机运转速度 (脉宽)
void set_servo_pwm(int pulse_us) {
    uint32_t duty = us_to_duty(pulse_us);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// 主任务
void app_main(void) {
    ESP_LOGI(TAG, "系统初始化...");
    init_uart();
    init_servo();
    ESP_LOGI(TAG, "初始化完成，等待摄像头数据。");
    ESP_LOGI(TAG, "当前参数: KP=%.2f, KD=%.2f", KP, KD);
    ESP_LOGI(TAG, "可通过串口发送 P:1.5 或 D:0.8 进行动态调参");

    // 摄像头串口缓存
    uint8_t *cam_data = (uint8_t *) malloc(BUF_SIZE);
    char cam_line_buf[128];
    int cam_line_pos = 0;
    
    // 【新增】控制串口缓存
    uint8_t *ctrl_data = (uint8_t *) malloc(256);
    char ctrl_line_buf[64];
    int ctrl_line_pos = 0;

    TickType_t last_target_time = 0; 
    bool is_target_lost = true;      

    int current_pwm = 1500; 
    const int MAX_PWM_STEP = 15; 

    while (1) {
        // ---------------- 1. 处理调参串口数据 (非阻塞) ----------------
        int ctrl_len = uart_read_bytes(CTRL_UART_PORT, ctrl_data, 256 - 1, 0);
        if (ctrl_len > 0) {
            for (int i = 0; i < ctrl_len; i++) {
                if (ctrl_data[i] == '\n' || ctrl_data[i] == '\r') {
                    if (ctrl_line_pos > 0) {
                        ctrl_line_buf[ctrl_line_pos] = '\0'; 
                        
                        float new_val = 0;
                        // 解析 "P:1.5" 或 "D:0.8" 格式
                        if (sscanf(ctrl_line_buf, "P:%f", &new_val) == 1) {
                            KP = new_val;
                            ESP_LOGI(TAG, ">>>> 成功更新 KP = %.2f", KP);
                        } else if (sscanf(ctrl_line_buf, "D:%f", &new_val) == 1) {
                            KD = new_val;
                            ESP_LOGI(TAG, ">>>> 成功更新 KD = %.2f", KD);
                        } else {
                            ESP_LOGW(TAG, "未知调参指令: %s", ctrl_line_buf);
                        }
                        ctrl_line_pos = 0; 
                    }
                } else if (ctrl_line_pos < sizeof(ctrl_line_buf) - 1) {
                    ctrl_line_buf[ctrl_line_pos++] = ctrl_data[i];
                }
            }
        }

        // ---------------- 2. 处理摄像头串口数据 (非阻塞) ----------------
        int cam_len = uart_read_bytes(CAM_UART_PORT, cam_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        
        if (cam_len > 0) {
            for (int i = 0; i < cam_len; i++) {
                if (cam_data[i] == '\n' || cam_data[i] == '\r') {
                    if (cam_line_pos > 0) {
                        cam_line_buf[cam_line_pos] = '\0'; 
                        
                        int target_x = 0, target_y = 0;
                        
                        if (sscanf(cam_line_buf, "X:%d, Y:%d", &target_x, &target_y) == 2) {
                            
                            if (abs(target_x) > 600) {
                                cam_line_pos = 0;
                                continue; 
                            }

                            last_target_time = xTaskGetTickCount();
                            if (is_target_lost) {
                                is_target_lost = false;
                                last_error_x = target_x; 
                            }
                            
                            int target_pwm = 1500; 

                            if (abs(target_x) <= DEAD_ZONE) {
                                target_pwm = 1500; 
                                last_error_x = 0; 
                            } else {
                                int d_error = target_x - last_error_x;
                                int speed_offset = (int)((target_x * KP) + (d_error * KD));
                                
                                if (speed_offset > 400) speed_offset = 400;
                                if (speed_offset < -400) speed_offset = -400;
                                
                                target_pwm = 1500 - speed_offset;
                                last_error_x = target_x; 
                            }

                            if (target_pwm > current_pwm + MAX_PWM_STEP) {
                                current_pwm += MAX_PWM_STEP; 
                            } else if (target_pwm < current_pwm - MAX_PWM_STEP) {
                                current_pwm -= MAX_PWM_STEP; 
                            } else {
                                current_pwm = target_pwm;    
                            }

                            set_servo_pwm(current_pwm);
                        }
                        cam_line_pos = 0; 
                    }
                } else if (cam_line_pos < sizeof(cam_line_buf) - 1) {
                    cam_line_buf[cam_line_pos++] = cam_data[i];
                }
            }
        }

        // ---------------- 3. 处理目标丢失的逻辑 ----------------
        TickType_t current_time = xTaskGetTickCount();
        
        if ((current_time - last_target_time) * portTICK_PERIOD_MS > TARGET_TIMEOUT_MS) {
            if (!is_target_lost) {
                is_target_lost = true;
                last_error_x = 0; 
                ESP_LOGI(TAG, "目标丢失，舵机开始平滑减速至待机");
            }
            
            if (current_pwm != 1500) {
                if (1500 > current_pwm + MAX_PWM_STEP) {
                    current_pwm += MAX_PWM_STEP;
                } else if (1500 < current_pwm - MAX_PWM_STEP) {
                    current_pwm -= MAX_PWM_STEP;
                } else {
                    current_pwm = 1500; 
                }
                set_servo_pwm(current_pwm);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 这里的延时同时作为两个串口的周期调度
    }
    
    free(cam_data);
    free(ctrl_data);
}