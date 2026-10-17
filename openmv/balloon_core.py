# ===========================================================================
#  balloon_core.py —— OpenMV 气球辨色核心库（红球识别 + 找可扎目标）
#
#  用途：车绕完场、左转进场以后，场地里**没有线**可循，靠这台摄像头找球：
#        · 认出红球（规则：扎破红球 = 比赛立即终止）→ 报给 UNO 躲开
#        · 找出最近的非红气球 → 报它的水平偏移 + 大概多远，让 UNO 冲上去扎
#
#  ★ 部署（两份文件，顺序别反）
#      1) IDE 打开 balloon_core.py -> 工具(Tools) -> Save open script to OpenMV Cam
#      2) 再打开 balloon_main.py   -> 同样存到摄像头，存成 **main.py**
#      3) 弹出 USB 磁盘 -> 拔线 -> 给车通电，冷启动自动跑 main.py
#    台面调参用 threshold_lab.py（同一套算法，但打印 + 画框）。
#
#  ---------------------------------------------------------------------------
#  【输出协议：气球帧 10 字节，帧类型字节 = 0x56】
#     0xAA 0x56 | dx | size | flags | ped_dx | n_red | dist | seq | crc
#       dx     (int8)  ★目标水平偏移 = 目标中心 x - 画面中心（正 = 目标在画面右侧）
#                       单位像素，QQVGA 下范围 ±80，正好塞进 int8
#       size   (uint8) 目标外接框宽度(px)，0 = 本帧没有目标。距离的代理量
#       flags  (uint8) 见下面的 F_xxx
#       ped_dx (int8)  挡在正前方的红球偏移（只有 F_RED_AHEAD 置位时才有意义）
#       n_red  (uint8) 画面里红球个数（调试用，UNO 不一定要看）
#       dist   (uint8) 目标距离估计(cm)，0 = 估不出来（太大/太小/没目标）
#       seq    (uint8) 帧序号，每帧 +1，0~255 回绕（UNO 用它查摄像头是不是卡死）
#       crc    CRC8(第 2~8 字节)  多项式 0x07，初值 0x00
#    ★ 和巡线帧（0xAA 0x55，9 字节）用**帧类型字节**区分，长度也不同（10 vs 9/6），
#      所以 UNO 那边先看 buf[1] 再分派，两套帧可以同时存在于一条总线上。
#
#    flags：0x01 找到可扎目标（非红）      0x02 ★前方有红球挡路，别撞上去
#           0x04 画面里看到红球            0x08 目标已经很近（可以撞了）
#           0x10 本帧没有可扎目标（没球 / 只有红球）
#           0x20 低可信（画面太暗/没对比度，预留，当前不置位）
#           0x40 本帧是沿用上一帧的目标（短暂丢失时的保持）
#           0x80 本帧异常（内部出错，UNO 应该当没收到）
#  ---------------------------------------------------------------------------
#  【和巡线那套的关系】
#    两套算法本来互相独立、**不共用文件**；巡线视觉那套已在 2026-10-17 整理时删除，
#    现在仓库里只剩气球这一条线。本文件里的 crc8 / 链路 / 灯都是自带的，
#    为的是「只存两个文件也能跑」。
#    ★ crc8 两处必须完全一致：本文件、UNO 侧 car_balloon.ino
#  ---------------------------------------------------------------------------
#  【★ 现场标定：阈值必须重标，别信默认值】
#    默认的 LAB 阈值是按「白布 + 玫红/橙/黄/绿/蓝气球」估的，只能让你先跑起来。
#    标定步骤（用 threshold_lab.py）：
#      1) 把球放在场地上，摄像头对着它，看 IDE 里打印的 **中心点 LAB** 和
#         **每个 blob 的平均 LAB**；
#      2) 把该颜色的 L/A/B 范围填进下面的 TH_xxx；
#      3) 重点看红色和橙色：玫红 A 高、**B 不高**；橙色 **B 很高** —— 靠 B 分开；
#      4) 规则说场上不会出现粉红/紫红等与红球冲突的颜色，但**橙色可能出现**。
#
#    ★ 误判代价是不对称的：把红球判成可扎 = 直接终止比赛；
#      把非红球判成红球 = 漏扎一个，加时 60 秒。所以宁可保守（判不准时按红处理）。
# ===========================================================================

import time

# ============================== ★ 参数区 ==============================
# ★★ 现场调参基本只动这一段；改完两个入口脚本（balloon_main.py / threshold_lab.py）都会变 ★★

# —— 摄像头 ——
CAM_SKIP_MS    = 2000       # 上电让摄像头稳定的时间(ms)
IMG_W, IMG_H   = 160, 120   # QQVGA。想看得更远可以换 QVGA(320x240)，但帧率掉一半
FRAMESIZE_QVGA = False      # True = 用 QVGA（远距离更能看清球，代价是慢）
LOCK_EXPOSURE  = False      # True = 锁曝光。★灯光明亮恒定再锁，锁错了颜色全废
LOCK_WHITE_BAL = True       # ★必须锁白平衡：自动白平衡会把「玫红」慢慢调成别的颜色
BRIGHTNESS     = 0          # 画面偏暗就 +1/+2（-3~+3）

# —— 气球几何（用来把「像素宽度」换算成「距离」）——
BALLOON_DIAM_CM = 18.0      # 规则：气球最大直径 160~200mm，取中间值 18cm
FOCAL_PX        = 112.0     # 焦距(像素) = IMG_W / (2*tan(HFOV/2))；2.8mm 镜头 HFOV≈70.8° -> ≈112
                            # ★想更准就拿个球放 30cm 处实测 size，反推 FOCAL_PX = size*30/18
DIST_MAX_CM     = 250       # 距离估计的上限（超过就按 250 报，反正太远也没用）

# —— ★ 颜色阈值（LAB，格式 (L_min,L_max,A_min,A_max,B_min,B_max)）——
#    红色单独一组（要躲）；其余颜色一起当「可扎目标」。
#    ★★ 组委会买的是「马卡龙气球（100 个装）」（补充规则(二).1 的京东链接，
#       截图见 .rules_img/p-006-003.jpg）—— 马卡龙 = **粉彩色**：明度 L 高、
#       饱和度 A/B 都不大。所以下面的范围整体偏「淡」，不是浓色的范围。
#    玫红/粉红 = 中等 A、B 不高；橙色 = A 中等且 B 明显高；黄色 = 低 A、很高 B；
#    绿色 = 负 A；蓝色 = 负 A 且很负 B；紫色 = 正 A 且负 B。
TH_RED = [
    (45, 95, 25, 75, -20, 35),      # ★红球（马卡龙玫红/粉红）—— 这一组判错就终止比赛，宁可窄一点
]
TH_OTHER = [                        # 除红以外「可能是气球」的颜色，都算可扎目标
    (55, 100, 5, 60, 35, 95),       # 橙 / 蜜桃（马卡龙橙，B 高是它和红的分界）
    (65, 100, -20, 30, 30, 95),     # 黄
    (45, 95, -70, -10, 0, 60),      # 绿
    (45, 95, -45, 20, -90, -20),    # 蓝
    (40, 90, 5, 60, -75, -10),      # 紫（规则说不会有与红冲突的颜色，但留着当兜底）
]
TH_ALL = TH_RED + TH_OTHER          # 找「有个球」时用这一组（OR 关系）

RED_IS_CONSERVATIVE = True  # ★True = 拿不准就当红球（宁可漏扎，不要终止比赛）
LAB_RED_A_MIN  = 25         # 二次判定：A 均值 >= 这个值（马卡龙红饱和度不高，别设太大）
LAB_RED_B_MAX  = 35         #            且 B 均值 <= 这个值   → 判为红（橙色 B 高，卡在这里）
LAB_RED_L_MIN  = 45         #            且别太暗：马卡龙球都挺亮，太暗的多半是阴影或球口

# —— 找 blob 的通用门槛 ——
ROI_ALL       = (0, 0, IMG_W, IMG_H)   # 全画面找球
X_STRIDE      = 2           # 跳像素扫描：2 够用且快一倍
Y_STRIDE      = 2
MIN_PIXELS    = 40          # 一个 blob 至少这么多像素才算球（小 = 噪点）
MIN_AREA      = 40
MERGE_MARGIN  = 8           # 相邻 blob 合并间距：同一个球被分成两块时能合起来
MIN_W_PX      = 6           # 外接框最小宽度(px)：比这还小就看不清是啥颜色
MAX_ASPECT    = 3.0         # 长宽比超过这个 = 不是球（多半是色带/反光条）
MIN_DENSITY   = 0.35        # 外接框里实心程度：球是块状的，太稀疏的丢掉

# —— 目标挑选与判定 ——
NEAR_W_PX     = 55          # 外接框宽 >= 这个像素 = 已经很近（≈37cm），可以撞了
RED_BLOCK_W   = 25          # 红球外接框宽 >= 这个像素、且挡住画面中心 -> 报「前方有红球」
CONFIRM_FRAMES = 2          # 目标连续出现这么多帧才算「找到」（挡单帧噪点）
HOLD_MS       = 400         # 目标短暂丢失时，沿用上一帧结论这么久（丢一帧不该让车乱转）
CENTER_BAND   = 12          # 目标/红球中心离画面中心在这个像素带内，算「正前方」

# —— 低可信 ——
LOWCONF_SPREAD = 25         # 画面下部 ROI 的 (max-min) 太小 = 没对比度（全白/全黑/眩光）

# —— 通信通道（★二选一：P4/P5 同时是 I2C2 和 UART3，**不能同时用**）——
#   "uart" = UART3：P4 = TX、P5 = RX   ← 现在走这条（串口是字节流，UNO 侧要自己找帧头）
#   "i2c"  = I2C2 ：P4 = SCL、P5 = SDA ← 老方案，UNO 当从机，仍然可用
LINK = "uart"               # ★和 UNO 侧 car_balloon.ino 的 LINK_MODE 必须对应
UART_ID   = 3               # UART3 = P4/P5；想要第二条可以用 UART1 = P1/P0（和 I2C 不冲突）
UART_BAUD = 115200          # 硬件串口随便跑；★UNO 那边若用软串口(D8/D9)，两边都要改成 38400
I2C_ADDR = 0x08             # ★ 必须和 UNO 侧从机地址一致
I2C_FREQ = 100000           # 100kHz 足够：10 字节 @30fps = 3kB/s
BUS_ID   = 2                # I2C2 = P4(SCL) / P5(SDA)

# —— 标志位（和 UNO 侧一一对应，改一边必须改另一边）——
F_TARGET    = 0x01          # 找到可扎目标（非红）
F_RED_AHEAD = 0x02          # 前方有红球挡路
F_RED_SEEN  = 0x04          # 画面里看到红球
F_NEAR      = 0x08          # 目标已经近到可以撞
F_NONE      = 0x10          # 本帧没有可扎目标（没球 / 只有红球）
F_LOWCONF   = 0x20          # 低可信（预留，当前不置位）
F_HOLD      = 0x40          # 本帧沿用上一帧的目标
F_ERR       = 0x80          # 本帧异常

# —— 协议常量 ——
HDR0, HDR1_LINE, HDR1_BALLOON = 0xAA, 0x55, 0x56
FRAME_LEN = 10
IDX_DX, IDX_SIZE, IDX_FLAGS, IDX_PED, IDX_NRED, IDX_DIST, IDX_SEQ, IDX_CRC = 2, 3, 4, 5, 6, 7, 8, 9

# ============================== 时间：兼容 ticks 回绕 ==============================

try:
    ticks_ms   = time.ticks_ms
    ticks_diff = time.ticks_diff
except AttributeError:                       # 极老固件没有 ticks_*，退回毫秒时间戳
    def ticks_ms():
        return int(time.time() * 1000)

    def ticks_diff(a, b):
        return a - b

# ============================== 协议打包 / CRC8 ==============================

def _val(obj, name, default=0):
    """读 statistics 的一个字段，**新老固件都吃**。

    4.x：st.l_mean 是函数 -> st.l_mean()
    5.x：st 是 attrtuple，st.l_mean 直接是整数 —— 再调就是
         TypeError: 'int' object isn't callable
    ★ 5.x 上如果不改成这样，_lab_red 会一路返回 None，按保守策略
      **所有球都会被判成红球** —— 车会一个球都不敢扎。
    """
    v = getattr(obj, name, default)
    if callable(v):
        try:
            return v()
        except Exception:
            return default
    return v


def crc8(data):
    """CRC-8/ATM：多项式 0x07，初值 0x00，不取反。和 UNO 侧、.tests/parse_check.c 必须一致。"""
    crc = 0x00
    for b in data:
        crc ^= (b & 0xFF)
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack(dx, size, flags, ped_dx, n_red, dist, seq):
    """把一帧打成 10 字节。int8 字段用 & 0xFF 转成无符号字节，UNO 侧再当 int8 解释。"""
    d  = int(dx) & 0xFF
    s  = int(size) & 0xFF
    f  = int(flags) & 0xFF
    p  = int(ped_dx) & 0xFF
    nr = int(n_red) & 0xFF
    di = int(dist) & 0xFF
    q  = int(seq) & 0xFF
    c  = crc8((d, s, f, p, nr, di, q))
    return bytes([HDR0, HDR1_BALLOON, d, s, f, p, nr, di, q, c])


def _clamp_i8(v, lo=-127, hi=127):
    v = int(round(v))
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


# ============================== 新老固件兼容的初始化 ==============================

def make_snapper():
    """返回 snap()：调用一次 = 拍一帧。新固件 csi / 老固件 sensor 自动选。

    ★ 和巡线那套的唯一区别：这里要 **RGB565**（彩色）—— 辨色必须有颜色信息。
    """
    framesize_name = "QVGA" if FRAMESIZE_QVGA else "QQVGA"
    try:
        import csi
        cam = csi.CSI()
        cam.reset()
        cam.pixformat(csi.RGB565)
        cam.framesize(getattr(csi, framesize_name))
        cam.snapshot(time=CAM_SKIP_MS)
        cam.auto_gain(False)
        cam.auto_whitebal(not LOCK_WHITE_BAL)     # 锁白平衡：不锁的话玫红会被「调」成别的色
        if BRIGHTNESS:
            try:
                cam.brightness(BRIGHTNESS)
            except Exception:
                pass
        if LOCK_EXPOSURE:
            try:
                cam.auto_exposure(False)
            except Exception:
                pass
        return lambda: cam.snapshot()
    except ImportError:
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA if FRAMESIZE_QVGA else sensor.QQVGA)
        sensor.skip_frames(time=CAM_SKIP_MS)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(not LOCK_WHITE_BAL)
        if BRIGHTNESS:
            try:
                sensor.set_brightness(BRIGHTNESS)
            except Exception:
                pass
        if LOCK_EXPOSURE:
            try:
                sensor.set_auto_exposure(False)
            except Exception:
                pass
        return lambda: sensor.snapshot()


def make_i2c():
    """返回 (bus, send)：send(data) 就是一次主机写（UNO 是从机）。"""
    try:
        from machine import I2C
        bus = I2C(BUS_ID, freq=I2C_FREQ)
        return bus, (lambda data: bus.writeto(I2C_ADDR, data))
    except ImportError:
        from pyb import I2C
        bus = I2C(BUS_ID, I2C.MASTER, baudrate=I2C_FREQ)
        return bus, (lambda data: bus.send(data, I2C_ADDR))


def make_link():
    """按 LINK 打开通道，返回 (dev, send, 说明文字)。

    send(data) 收 bytes 直接发出去 —— 两个入口脚本不用关心底下是 UART 还是 I2C。
    ★ UART 那边不需要从机地址，也没有 ACK：线没接好不会报错，只是 UNO 收不到，
      所以「链路通没通」要靠 UNO 侧有没有回数据来判断（见气球辨色.md）。
    """
    if LINK == "uart":
        try:
            from machine import UART
        except ImportError:
            from pyb import UART
        try:
            u = UART(UART_ID, baudrate=UART_BAUD)      # 新固件写法
        except TypeError:
            u = UART(UART_ID, UART_BAUD)               # 老固件只吃位置参数
        return u, (lambda data: u.write(data)), "UART%d @%d (P4=TX, P5=RX)" % (UART_ID, UART_BAUD)

    bus, send = make_i2c()
    return bus, send, "I2C%d @%d 从机 0x%02X (P4=SCL, P5=SDA)" % (BUS_ID, I2C_FREQ, I2C_ADDR)


def link_selftest(dev):
    """I2C：扫到从机地址才算通；UART：端口打开就算通（没有地址可扫）。"""
    if LINK == "uart":
        return True
    addrs = i2c_scan(dev)
    return (addrs is not None) and (I2C_ADDR in addrs)


def i2c_scan(bus):
    """扫总线，返回地址列表；失败返回 None（调用方自己决定怎么报）。"""
    try:
        return bus.scan()
    except Exception:
        return None


def make_leds():
    """返回 (red, blue)，取不到就是 (None, None) —— 灯只是提示，不许影响识别。"""
    try:
        from machine import LED
        return LED("LED_RED"), LED("LED_BLUE")
    except Exception:
        try:
            from pyb import LED
            return LED(1), LED(3)
        except Exception:
            return None, None


def blink(led, times, on_ms=60, off_ms=120):
    if led is None:
        return
    for _ in range(times):
        led.on()
        time.sleep_ms(on_ms)
        led.off()
        time.sleep_ms(off_ms)


# ============================== 辨色核心 ==============================

def _rect_overlap_ratio(a, b):
    """b 有多大比例落在 a 里（0.0~1.0）。用来判断「这个 blob 就是那个红球」。"""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = ax if ax > bx else bx
    y1 = ay if ay > by else by
    x2 = (ax + aw) if (ax + aw) < (bx + bw) else (bx + bw)
    y2 = (ay + ah) if (ay + ah) < (by + bh) else (by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    barea = bw * bh
    if barea <= 0:
        return 0.0
    return float(inter) / float(barea)


def _shrink(rect, ratio=0.25):
    """把外接框往里收一圈再取统计值：不然背景（白布）会把均值拉偏。"""
    x, y, w, h = rect
    dx = int(w * ratio)
    dy = int(h * ratio)
    x2, y2 = x + dx, y + dy
    w2, h2 = w - 2 * dx, h - 2 * dy
    if w2 < 2 or h2 < 2:
        return rect
    return (x2, y2, w2, h2)


class BalloonDetector(object):
    """一帧进、一帧出：找出红球（要躲）和最近的非红气球（要扎）。

    用法（两个入口脚本都是这样）：
        det = BalloonDetector()
        ...
        det.scan(img, now)
        send(det.frame())
    """

    def __init__(self):
        self.seq      = 0
        self.fps      = 0.0
        self.dx       = 0        # 本帧上报的目标偏移
        self.size     = 0
        self.dist     = 0
        self.ped_dx   = 0
        self.n_red    = 0
        self.flags    = F_NONE
        self._conf    = 0        # 目标连续出现帧数
        self._last_dx = 0
        self._last_sz = 0
        self._last_ms = 0
        self._held    = False
        self.status_txt = ""
        self.reds     = []       # 本帧判为红球的 blob（只有台面调参脚本会去画它）
        self.others   = []       # 本帧判为非红的 blob

    # ---------- 工具 ----------

    def _balloon_like(self, b):
        """球应该是个方块状的实心团：太细长、太稀疏的都不是球。"""
        w, h = b.w(), b.h()
        if w < MIN_W_PX or h < 2:
            return False
        if w > h * MAX_ASPECT or h > w * MAX_ASPECT:
            return False
        if b.density() < MIN_DENSITY:
            return False
        return True

    def _lab_red(self, img, rect):
        """到画面里量一下这块的 LAB 均值。返回 True/False/None（None = 量不出来）。"""
        try:
            st = img.get_statistics(roi=_shrink(rect))
            a = _val(st, "a_mean")
            bb = _val(st, "b_mean")
            l = _val(st, "l_mean")
        except Exception:
            return None
        if l < LAB_RED_L_MIN:
            return RED_IS_CONSERVATIVE        # 太黑，多半是阴影
        if a >= LAB_RED_A_MIN and bb <= LAB_RED_B_MAX:
            return True
        if a <= LAB_RED_A_MIN - 10 or bb >= LAB_RED_B_MAX + 10:
            return False                      # 明显不是红（偏绿/偏黄/偏蓝）
        return None                           # 灰区：交给调用方按保守策略处理

    # ---------- 主流程 ----------

    def scan(self, img, now):
        """扫一帧。结果存在 self 的字段里，用 frame() 取走。"""
        cx_img = IMG_W // 2
        flags  = 0

        # ① 找出画面里所有的「球」（红 + 其它颜色一起找）
        blobs = img.find_blobs(
            TH_ALL, roi=ROI_ALL,
            x_stride=X_STRIDE, y_stride=Y_STRIDE,
            pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA,
            merge=True, margin=MERGE_MARGIN,
        )
        blobs = [b for b in blobs if self._balloon_like(b)]

        # ② 单独再找一遍红球：既给「躲红球」用，也给 ③ 的分类当依据
        red_blobs = img.find_blobs(
            TH_RED, roi=ROI_ALL,
            x_stride=X_STRIDE, y_stride=Y_STRIDE,
            pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA,
            merge=True, margin=MERGE_MARGIN,
        )
        red_blobs = [b for b in red_blobs if self._balloon_like(b)]

        # ③ 分类：每个 blob 是红还是非红
        reds, others = [], []
        for b in blobs:
            is_red = None
            for r in red_blobs:                      # 判据一：和红球阈值的 blob 大面积重合
                if _rect_overlap_ratio(b.rect(), r.rect()) > 0.5:
                    is_red = True
                    break
            if is_red is None:                       # 判据二：量这块的 LAB 均值
                is_red = self._lab_red(img, b.rect())
            if is_red is True:
                reds.append(b)
            elif is_red is False:
                others.append(b)
            else:                                    # 判不出来
                if RED_IS_CONSERVATIVE:
                    reds.append(b)                   # ★保守：当红球，宁可漏扎
                else:
                    others.append(b)

        self.n_red = len(reds)
        self.reds, self.others = reds, others
        if reds:
            flags |= F_RED_SEEN

        # ④ 前方有没有红球挡路（挡在画面中心、而且够大）
        self.ped_dx = 0
        for r in reds:
            if r.w() < RED_BLOCK_W:
                continue
            if r.x() - CENTER_BAND <= cx_img <= r.x() + r.w() + CENTER_BAND:
                self.ped_dx = _clamp_i8(r.cx() - cx_img)
                flags |= F_RED_AHEAD
                break

        # ⑤ 挑目标：非红球里挑「最大」的那个（最大 = 最近）
        target = None
        for b in others:
            if target is None or b.pixels() > target.pixels():
                target = b

        # ⑥ 目标确认 + 短暂丢失保持
        if target is not None:
            self._conf += 1
            self._last_dx = _clamp_i8(target.cx() - cx_img)
            self._last_sz = min(255, target.w())
            self._last_ms = now
            self._held    = False
        else:
            self._conf = 0
            if self._last_sz and ticks_diff(now, self._last_ms) <= HOLD_MS:
                self._held = True                    # 丢一两帧：沿用上一帧结论
            else:
                self._held = False
                self._last_sz = 0

        if self._last_sz and (self._conf >= CONFIRM_FRAMES or self._held):
            flags |= F_TARGET
            if self._held:
                flags |= F_HOLD
            self.dx   = self._last_dx
            self.size = self._last_sz
            self.dist = self._dist_cm(self._last_sz)
            if self.size >= NEAR_W_PX:
                flags |= F_NEAR
        else:
            self.dx, self.size, self.dist = 0, 0, 0
            flags |= F_NONE

        if not blobs:
            flags |= F_NONE

        self.flags = flags
        self.seq = (self.seq + 1) & 0xFF
        self.status_txt = "tgt=%d sz=%d d=%dcm red=%d%s" % (
            self.dx, self.size, self.dist, self.n_red,
            " HOLD" if self._held else "")
        return self

    def _dist_cm(self, w_px):
        """像素宽度 -> 距离(cm)。几何关系：dist = 球直径 * 焦距 / 像素宽度。"""
        if w_px <= 0:
            return 0
        d = BALLOON_DIAM_CM * FOCAL_PX / float(w_px)
        if d > DIST_MAX_CM:
            return DIST_MAX_CM
        if d < 1:
            return 0
        return int(d)

    def frame(self):
        """打成 10 字节，直接丢给 I2C 发出去。"""
        return pack(self.dx, self.size, self.flags, self.ped_dx,
                    self.n_red, self.dist, self.seq)

    def status(self):
        return self.status_txt

    # ---------- 给台面调参用的辅助 ----------

    def center_lab(self, img):
        """画面正中心一小块的 LAB 均值 —— 标定时把球对准十字，直接念这组数。"""
        r = (IMG_W // 2 - 8, IMG_H // 2 - 8, 16, 16)
        try:
            st = img.get_statistics(roi=r)
            return (_val(st, "l_mean"), _val(st, "a_mean"), _val(st, "b_mean"))
        except Exception:
            return None
