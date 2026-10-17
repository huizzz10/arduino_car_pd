# ===========================================================================
#  minimal.py —— 最小能跑的一段（用来判断"到底是代码问题还是固件/IDE 问题"）
#
#  就 6 行有效代码：开摄像头 -> 一直取图。没有画框、没有 find_blobs、
#  没有 statistics、不读任何库。画面会出现在 IDE 右上角。
#
#  ★ 如果**连这个都报错**，那就跟我们的代码无关了：
#      · 先确认 IDE 版本和摄像头固件是不是配套（5.0 固件要新版 IDE）
#      · 或者 工具 -> Run bootloader 重刷一次固件
# ===========================================================================

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)

n = 0
while True:
    img = sensor.snapshot()
    n += 1
    if n % 30 == 0:
        print("还在跑，第 %d 帧  %dx%d" % (n, img.width(), img.height()))
