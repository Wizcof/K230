# -*- coding: utf-8 -*-
'''
脚本名称：tracking_video_deploy.py
功能说明：基于官方 PipeLine 的视频流目标检测追踪，并发送串口偏移量
'''

import os, gc, math
from machine import UART, FPIOA
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *

# ================= 1. 串口 (UART) 初始化 =================
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
uart = UART(UART.UART1, 115200)

# ================= 2. 视频流与显示配置 =================
display_mode = "lt9611"
rgb888p_size = [1280, 720]

CENTER_X = rgb888p_size[0] // 2
CENTER_Y = rgb888p_size[1] // 2

# ================= 3. 模型与部署配置加载 =================
root_path = "/sdcard/mp_deployment_source/"
deploy_conf = read_json(root_path + "deploy_config.json")
kmodel_path = root_path + deploy_conf["kmodel_path"]
labels = deploy_conf["categories"]
confidence_threshold = deploy_conf["confidence_threshold"]
nms_threshold = deploy_conf["nms_threshold"]
model_input_size = deploy_conf["img_size"]
nms_option = deploy_conf["nms_option"]
model_type = deploy_conf["model_type"]

anchors = []
if model_type == "AnchorBaseDet":
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

inference_mode = "video"
debug_mode = 0

# ================= 4. 初始化 PipeLine 和 检测模型 =================
pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
pl.create()
display_size = pl.get_display_size()

det_app = DetectionApp(inference_mode, kmodel_path, labels, model_input_size, anchors, model_type, confidence_threshold, nms_threshold, rgb888p_size, display_size, debug_mode=debug_mode)
det_app.config_preprocess()

print("初始化成功，开始 PipeLine 视频流推理与追踪...")

# ================= 5. 主循环与追踪逻辑 =================
# 在 while True 外面初始化历史偏移量和滤波系数
last_dx, last_dy = 0, 0
# alpha 是滤波系数 (0 < alpha <= 1)
# alpha 越小，抗抖动能力越强，但跟随会有延迟
# alpha 越大，跟随越灵敏，但更容易受噪声影响（建议在 0.3 - 0.7 之间调参）
alpha = 0.5

try:
    while True:
        with ScopedTiming("total", 1):
            img = pl.get_frame()
            res = det_app.run(img)

            if res:
                if isinstance(res, dict) and 'boxes' in res and len(res['boxes']) > 0:
                    largest_box = None
                    max_area = 0

                    boxes = res['boxes']
                    # ... [提取 largest_box 的代码保持不变] ...

                    if largest_box:
                        b_x, b_y, b_w, b_h = largest_box

                        # 【新增约束】：过滤掉过于离谱的比例或极小的噪点框
                        # 比如快递盒/袋的长宽比通常不会超过 5:1，面积也不能太小
                        if b_w / (b_h + 0.001) > 5 or b_h / (b_w + 0.001) > 5 or (b_w * b_h) < 1000:
                            continue # 跳过这个不合理的框

                        cx = int(b_x + b_w // 2)
                        cy = int(b_y + b_h // 2)

                        # 计算原始偏移量
                        raw_dx = cx - CENTER_X
                        raw_dy = cy - CENTER_Y

                        # 【新增滤波】：应用 EMA 算法平滑坐标
                        dx = int(alpha * raw_dx + (1 - alpha) * last_dx)
                        dy = int(alpha * raw_dy + (1 - alpha) * last_dy)

                        # 更新历史坐标
                        last_dx = dx
                        last_dy = dy

                        uart_data = f"X:{dx}, Y:{dy}\n"

                        # 打印出来看看！
                        print(f"追踪成功！发送串口: {uart_data.strip()}")

                        uart.write(uart_data.encode('utf-8'))

                        # 在 OSD 图层画准星和文字
                        pl.osd_img.draw_cross(cx, cy, color=(0, 255, 0), size=10, thickness=2)
                        pl.osd_img.draw_string_advanced(10, 10, 30, uart_data.strip(), color=(255, 0, 0))

            det_app.draw_result(pl.osd_img, res)
            pl.show_image()
            gc.collect()

except KeyboardInterrupt as e:
    print("用户停止运行")
except BaseException as e:
    print(f"发生异常: '{e}'")
finally:
    det_app.deinit()
    pl.destroy()
