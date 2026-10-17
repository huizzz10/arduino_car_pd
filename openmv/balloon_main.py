# ===========================================================================
#  balloon_main.py —— **上车跑的那份**（OpenMV4 H7 Plus 气球辨色 + I2C 发给 UNO）
#
#  ★ 部署（两份文件，顺序别反）
#      1) IDE 打开 balloon_core.py -> 工具(Tools) -> Save open script to OpenMV Cam
#      2) 再打开本文件 -> 同样存进去，存成 **main.py**
#      3) 弹出 USB 磁盘 -> 拔线 -> 给车通电，冷启动会自动跑 main.py
#    只存了 main.py 会报 ImportError（灯狂闪 + IDE 里中文提示）。
#
#  【这份和 threshold_lab.py 的分工】
#    · balloon_main.py = 上车：不打印、不画框（打印会让帧率减半），带 RGB 灯自检
#    · threshold_lab.py = 台面：只干"测颜色阈值"这一件事（命中哪个颜色、余量多少、
#                         实测 LAB 范围、建议阈值），跑起来不退出，出错也不死
#    · 算法和参数全在 balloon_core.py —— 改参数只改那一个文件
#
#  【上电自检】先扫 I2C：
#    · 找到 0x08（UNO）-> 蓝灯慢闪 2 下，开始工作
#    · 没找到          -> 红灯快闪 8 下（**仍旧继续跑**，方便你边跑边查）
#    · 中途 I2C 写失败 -> 只记数，不崩、不停
#
#  【和巡线脚本的关系】★摄像头只能跑一份 main.py：
#    这份一存成 main.py，摄像头就专职「辨色找球」，不再做巡线。
#    绕场巡线由 UNO 上的红外程序负责（pd_v3.0+1.01），两边互不干扰。
# ===========================================================================

import time

# ---- 载入核心库：没存 balloon_core.py 是最常见的翻车点，这里给个明白的提示 ----
try:
    import balloon_core as core
except ImportError:
    print("=" * 60)
    print("找不到 balloon_core.py ！")
    print("请先在 OpenMV IDE 里打开 balloon_core.py，用")
    print("  工具(Tools) -> Save open script to OpenMV Cam")
    print("把它存到摄像头，再存 main.py（本文件）。")
    print("=" * 60)
    try:
        from pyb import LED
        while True:                      # 红灯快闪，肉眼也能看出是“缺库”
            for i in (1, 2, 3):
                try:
                    LED(i).toggle()
                except Exception:
                    pass
            time.sleep_ms(100)
    except Exception:
        pass
    raise

# ---- 车上只留这两个开关 ----
DEBUG = False        # True = 在 IDE 里打印目标/帧率（会掉一半帧率，上车必须 False）
BLINK_EVERY = 0      # >0 = 每 N 帧蓝灯翻转一次当心跳（车上一般不需要，费电又晃眼）


def main():
    red, blue = core.make_leds()

    # ① 自检：链路开得起来吗（I2C 还要扫到从机地址；UART 端口打开就算通）
    dev, send, desc = core.make_link()
    found = core.link_selftest(dev)
    print("链路: %s" % desc)
    if found:
        core.blink(blue, 2)                      # 蓝灯慢闪 2 下 = 链路 OK
    else:
        core.blink(red, 8, 40, 40)               # 红灯快闪 8 下 = 没找到 UNO（继续跑）

    snap  = core.make_snapper()
    det   = core.BalloonDetector()
    clock = time.clock()
    errs  = 0
    n     = 0

    while True:
        clock.tick()
        img = snap()
        now = core.ticks_ms()

        det.scan(img, now)
        det.fps = clock.fps()

        # ② 发帧：气球帧 10 字节（帧类型 0x56）。写失败只记数 —— I2C 抖一下不该让车乱
        try:
            send(det.frame())
        except Exception:
            errs += 1

        n += 1
        if BLINK_EVERY > 0 and (n % BLINK_EVERY == 0) and blue is not None:
            blue.toggle()

        if DEBUG:
            print("%s err=%d" % (det.status(), errs))


main()
