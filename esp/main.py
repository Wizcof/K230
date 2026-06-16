import time, os, gc, sys, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA

# ================= 串口 (UART) 初始化 =================
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
uart = UART(UART.UART1,115200)
# ================= 摄像头与参数配置 =================
DETECT_WIDTH = 640
DETECT_HEIGHT = 480

CENTER_X = DETECT_WIDTH // 2
CENTER_Y = DETECT_HEIGHT // 2

sensor = None

try:
    sensor = Sensor(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # 【关键修改】在 Display.VIRT 后面加上 to_ide = True，强制开启 IDE 画面回传
    Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps = 100, to_ide = True)

    MediaManager.init()
    sensor.run()

    fps = time.clock()
    frame_count = 0
    threshold = [50, 50, 0, 0, 0, 0]
    r = [CENTER_X - 25, CENTER_Y - 25, 50, 50]

    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()

        if frame_count < 60:
            if frame_count == 0:
                print("准备追踪，请将目标放入中心方框内...")
            img.draw_rectangle([v for v in r])
            frame_count += 1

        elif frame_count < 120:
            if frame_count == 60:
                print("正在学习颜色阈值...")
            elif frame_count == 119:
                print("阈值学习完成！开始追踪...")

            hist = img.get_histogram(roi=r)
            lo = hist.get_percentile(0.01)
            hi = hist.get_percentile(0.99)

            threshold[0] = (threshold[0] + lo.l_value()) // 2
            threshold[1] = (threshold[1] + hi.l_value()) // 2
            threshold[2] = (threshold[2] + lo.a_value()) // 2
            threshold[3] = (threshold[3] + hi.a_value()) // 2
            threshold[4] = (threshold[4] + lo.b_value()) // 2
            threshold[5] = (threshold[5] + hi.b_value()) // 2

            for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10):
                img.draw_rectangle([v for v in blob.rect()])
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_rectangle([v for v in r])
            frame_count += 1
            del hist

        else:
            # 追踪并获取偏移量
            blobs = img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10)
            if blobs:
                largest_blob = max(blobs, key=lambda b: b.pixels())

                img.draw_rectangle([v for v in largest_blob.rect()])
                img.draw_cross(largest_blob.cx(), largest_blob.cy())

                dx = largest_blob.cx() - CENTER_X
                dy = largest_blob.cy() - CENTER_Y

                # 格式化串口数据并发送
                uart_data = f"X:{dx}, Y:{dy}\n"
                uart.write(uart_data.encode('utf-8'))

                # 【新增体验优化】直接在 IDE 的回传画面左上角打印坐标，方便你核对串口发出的数据
                img.draw_string_advanced(10, 10, 30, uart_data.strip(), color=(255, 0, 0))

        # 刷新屏幕显示 (此时因为加了 to_ide=True，IDE 里会有实时画面)
        Display.show_image(img)
        gc.collect()

except KeyboardInterrupt as e:
    print(f"用户停止运行")
except BaseException as e:
    print(f"发生异常 '{e}'")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
