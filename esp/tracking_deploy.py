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
try:
    while True:
        with ScopedTiming("total", 1):
            img = pl.get_frame()
            res = det_app.run(img)

            if res:
                print("【调试】原始检测结果:", res)
                # 3. 追踪与串口逻辑 (精准解析 ndarray 字典格式)
                if isinstance(res, dict) and 'boxes' in res and len(res['boxes']) > 0:
                    largest_box = None
                    max_area = 0

                    # 提取底层的检测框数组和得分数组
                    boxes = res['boxes']
                    scores = res['scores'] if 'scores' in res else []

                    # 遍历所有检测到的目标
                    for i in range(len(boxes)):
                        try:
                            # 确保提取出的是普通数字
                            box = boxes[i]
                            x1 = int(box[0])
                            y1 = int(box[1])
                            x2 = int(box[2])
                            y2 = int(box[3])

                            # 将 [x1, y1, x2, y2] 格式转为宽和高
                            b_w = x2 - x1
                            b_h = y2 - y1

                            # 过滤掉非法或者太小的框（比如宽或高小于0的噪点）
                            if b_w <= 0 or b_h <= 0:
                                continue

                            # 计算面积
                            area = b_w * b_h
                            if area > max_area:
                                max_area = area
                                # 存储左上角坐标和宽高
                                largest_box = (x1, y1, b_w, b_h)

                        except Exception as e:
                            print(f"坐标解析跳过: {e}")
                            continue

                    # 如果找到了最大的有效框，计算中心点并发送串口
                    if largest_box:
                        b_x, b_y, b_w, b_h = largest_box
                        # 计算目标中心点 cx, cy
                        cx = int(b_x + b_w // 2)
                        cy = int(b_y + b_h // 2)

                        # 计算相对屏幕中心的偏移量
                        dx = cx - CENTER_X
                        dy = cy - CENTER_Y

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
