# ===========================================================================
#  balloon_detect.py —— 认气球：**不认颜色，只认"是不是个球"**
#
#      TARGET = "ANY"   任何颜色的气球都认（默认，现在要的就是这个）
#      TARGET = "RED"   只认红球（比赛里要躲的那种），额外走一道色相判定
#
#  【三段流水线】
#    ① 扫描：ANY 用一组覆盖整圈色相的 LAB 阈值，把所有彩色块捞出来；
#             RED 只用红的那一段
#    ② 形状：比例 / 填充率 / 凸度 / 圆度 —— 圆和椭圆都认
#    ③ 判定：ANY 直接放行（**完全不看颜色**）；RED 才按色相 H 判红
#
#  【为什么 ANY 要好几段阈值】
#    红/橙/黄/绿/蓝/紫在 LAB 的 (A,B) 平面上散在不同象限，一个矩形框框不住；
#    而白墙、灰地、地板反光正好落在 A≈0 B≈0 的中间，必须排除掉。
#    所以拆成 4 段（暖色 / 冷色 / 黄 / 蓝），find_blobs 一次传进去取并集。
#
#  【看打印】
#    #010 ... 气球=1   颜色 3 -> 形状 1 -> 见过 1
#        颜色 N   扫描捞上来多少块
#        形状 M   通过形状判定的多少块
#        见过 K   最终认定的球（ANY 模式下 = 形状通过的）
#    被筛掉的会带原因和具体参数，卡在哪一关一眼看得出。
#
#  【判据全部可调】都在下面 ② 形状 那一段。
# ===========================================================================

import sys
import time

# ============================== 开关 ==============================
TARGET      = "ANY"    # "ANY" = 任何颜色的气球；"RED" = 只认红球
COLOR_SPACE = "RGB"    # TARGET="RED" 时才有用："RGB" 按色相，LAB 按 A/B
PRINT_EVERY = 10
DRAW        = True
IMG_W, IMG_H = 160, 120

# ---------- ① 扫描 ----------
#  ANY 模式：覆盖整圈色相的四段，白/灰（A≈B≈0）落在缝里，进不来
TH_ANY = [
    (20, 100, 15, 127, -128, 85),      # 暖色：红 / 粉 / 橙 / 紫
    (20, 100, -128, -15, -128, 85),    # 冷色：绿 / 青
    (35, 100, -128, 127, 25, 127),     # 黄（B 高）
    (20, 100, -128, 127, -128, -25),   # 蓝（B 低）
]
#  RED 模式：只要红的
TH_RED = [
    (15, 100, 25, 127, -25, 65),       # 正红 / 朱红
    (30, 100, 12, 90, -35, 45),        # 玫红 / 粉红 / 淡红
]

# ---------- ② 形状（圆、椭圆都要认）----------
#  气球两种形态：
#    圆形  ：外接框接近正方形，填充率 ≈ π/4 = 0.785，圆度 ≈ 1.0
#    椭圆  ：外接框能到 1.6:1 左右，填充率仍是 ≈ 0.785，圆度略低
#  所以判据不是"必须接近正方形"，而是"**凸的、实心的、不太长条**"。
MIN_PIXELS   = 24      # 16 太灵敏（噪点也被收进来）；24 够看 2.5m，误判明显少
MIN_W_PX     = 6       # 4 太松；6 在 2.5m 处（球约 8px）仍然认得，噪点挡掉不少
MAX_ASPECT   = 2.6     # 椭圆气球最长 ~1.6~2，切到边会更大，给到 2.6
MIN_EXTENT   = 0.42    # 填充率下限：圆/椭圆 ≈0.785；低于这个多半是碎块/分叉
MAX_EXTENT   = 0.95    # 太高（≈1.0）= 一整块方形，像纸片/色卡
MIN_SOLIDITY = 0.80    # 凸度：气球是凸的，色带/墙角/凹形不是（没有这接口就跳过）
MIN_ROUNDNESS = 0.35   # 圆度：圆=1，椭圆略低（没有这接口就跳过）
USE_ROUNDNESS = True   # 圆度要算周长，略费；觉得卡就关掉

# —— 远距离（小球）单独一档 ——
#   球远到只剩 6~10 像素宽时，凸度/圆度/填充率全是噪声，
#   硬套大球的判据只会把远处的球全筛掉。所以小球只查"不太长条"。
SMALL_W_PX        = 14     # 短边小于这个 = 远处小球
SMALL_MAX_ASPECT  = 3.2    # 小球被切边、被压缩，比例放更宽
SMALL_MIN_EXTENT  = 0.26   # 小球填充率天生偏低
X_STRIDE     = 1       # 原 2 —— 跳像素扫会直接漏掉小目标，代价是慢一点
Y_STRIDE     = 1

# ---------- ③-RGB ----------
RGB_V_MIN   = 50
RGB_S_MIN   = 0.30
HUE_MAX     = 20.0
HUE_WRAP    = 335.0
CONSERVATIVE = True

# ---------- ③-LAB ----------
LAB_L_MIN = 30
LAB_A_MIN = 25
LAB_B_MAX = 50

# ---------- 其它 ----------
CONFIRM_FRAMES = 3
BALLOON_DIAM_CM = 18.0
FOCAL_PX        = 112.0
NEAR_W_PX       = 55
CENTER_BAND     = 12

# 按 TARGET 选扫描阈值（放在这里是为了 probe() 和主循环共用）
SCAN_TH = TH_ANY if TARGET == "ANY" else TH_RED

try:
    _ms = time.ticks_ms
except AttributeError:
    _ms = lambda: int(time.time() * 1000)


# ============================== 兼容层 ==============================
def _dump(tag, e):
    print("!!! [%s] 出错：%s" % (tag, e))
    try:
        sys.print_exception(e)
    except Exception:
        pass


def _bv(obj, name, default=0):
    """取一个值：**是函数就调用，是普通值就直接返回**。

    ★ 这就是 'int' object isn't callable 的正解：
      4.x 的 b.w 是函数 -> b.w()
      5.x 的 b.w 可能直接是整数 -> 再调就炸
    """
    v = getattr(obj, name, default)
    if callable(v):
        try:
            return v()
        except Exception:
            return default
    return v


def b_x(b):
    return _bv(b, "x")


def b_y(b):
    return _bv(b, "y")


def b_w(b):
    return _bv(b, "w")


def b_h(b):
    return _bv(b, "h")


def b_px(b):
    return _bv(b, "pixels")


def b_den(b):
    return _bv(b, "density", 1.0)


def b_cx(b):
    return _bv(b, "cx", b_x(b) + b_w(b) // 2)


def b_rect(b):
    r = getattr(b, "rect", None)
    if r is not None:
        if callable(r):
            try:
                return r()
            except Exception:
                pass
        else:
            try:
                if len(r) == 4:
                    return r
            except Exception:
                pass
    return (b_x(b), b_y(b), b_w(b), b_h(b))


def _draw(img, name, coords, **kw):
    fn = getattr(img, name, None)
    if fn is None:
        return False
    try:
        fn(tuple(coords), **kw)
        return True
    except Exception:
        pass
    try:
        fn(*coords, **kw)
        return True
    except Exception:
        return False


def _label(img, x, y, text, color=(255, 255, 255)):
    try:
        img.draw_string((x, y), text, color=color)
        return True
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color=color)
        return True
    except Exception:
        return False


def _shrink(rect, ratio=0.25):
    x, y, w, h = rect
    dx, dy = int(w * ratio), int(h * ratio)
    if w - 2 * dx < 2 or h - 2 * dy < 2:
        dx = dy = 0
    return (x + dx, y + dy, w - 2 * dx, h - 2 * dy)


def _lab(img, rect):
    try:
        st = img.get_statistics(roi=_shrink(rect))
        return (_bv(st, "l_mean"), _bv(st, "a_mean"), _bv(st, "b_mean"))
    except Exception:
        return None


def _pixel(img, x, y):
    try:
        return img.get_pixel((x, y), rgbtuple=True)
    except Exception:
        pass
    try:
        return img.get_pixel(x, y, rgbtuple=True)
    except Exception:
        return None


def _rgb_mean(img, rect):
    x, y, w, h = rect
    r = g = b = 0.0
    n = 0
    for fx in (0.30, 0.50, 0.70):
        for fy in (0.30, 0.50, 0.70):
            c = _pixel(img, int(x + w * fx), int(y + h * fy))
            if c is None:
                continue
            try:
                r += c[0]; g += c[1]; b += c[2]; n += 1
            except Exception:
                continue
    if n == 0:
        return None
    return (r / n, g / n, b / n)


def _hsv(r, g, b):
    mx = max(r, g, b)
    mn = min(r, g, b)
    d = mx - mn
    if mx <= 0:
        return (0.0, 0.0, 0.0)
    s = d / float(mx)
    if d == 0:
        return (0.0, s, mx)
    if mx == r:
        h = 60.0 * (((g - b) / float(d)) % 6)
    elif mx == g:
        h = 60.0 * ((b - r) / float(d)) + 120.0
    else:
        h = 60.0 * ((r - g) / float(d)) + 240.0
    return (h % 360.0, s, mx)


# ============================== 判定 ==============================
def classify_rgb(rgb):
    if rgb is None:
        return CONSERVATIVE, "采不到像素", 0.0, 0.0, 0.0
    h, s, v = _hsv(rgb[0], rgb[1], rgb[2])
    if v < RGB_V_MIN:
        return CONSERVATIVE, "太暗(V=%d)" % int(v), h, s, v
    if s < RGB_S_MIN:
        return False, "不饱和(S=%.2f，白/灰)" % s, h, s, v
    if h <= HUE_MAX or h >= HUE_WRAP:
        return True, "H=%.0f°" % h, h, s, v
    return False, "H=%.0f° 不是红" % h, h, s, v


def classify_lab(lab):
    if lab is None:
        return CONSERVATIVE, "量不出颜色"
    l, a, b = lab
    if l < LAB_L_MIN:
        return CONSERVATIVE, "太暗"
    if a >= LAB_A_MIN and b <= LAB_B_MAX:
        return True, "A高B低"
    if a <= LAB_A_MIN - 10:
        return False, "A太低(偏橙/灰)"
    if b >= LAB_B_MAX + 15:
        return False, "B太高(偏橙/黄)"
    return CONSERVATIVE, "灰区"


def shape_ok(b):
    """圆/椭圆都认。返回 (通过?, 理由, 尺寸说明)。

    不要求"接近正方形"——椭圆气球本来就是长的。
    真正在挡的是：长条、空心/碎块、方形纸片、不够凸。
    """
    w, h = b_w(b), b_h(b)
    px = b_px(b)
    if px < MIN_PIXELS:
        return False, "像素太少", "%dpx" % px
    if min(w, h) < MIN_W_PX or h <= 0:
        return False, "太窄", "%dx%d" % (w, h)

    aspect = float(max(w, h)) / float(min(w, h))
    extent = px / float(w * h)          # 外接框填充率，自己算，不依赖 density()
    small = min(w, h) < SMALL_W_PX
    desc = "%dx%d 比例%.2f 实心%.2f%s" % (
        w, h, aspect, extent, " (远/小)" if small else "")

    max_aspect = SMALL_MAX_ASPECT if small else MAX_ASPECT
    min_extent = SMALL_MIN_EXTENT if small else MIN_EXTENT

    if aspect > max_aspect:
        return False, "太长条", desc
    if extent < min_extent:
        return False, "太稀疏(碎块/分叉)", desc
    if extent > MAX_EXTENT:
        return False, "太满(像方纸片)", desc

    if small:
        # 远处的球：像素太少，凸度/圆度量出来全是噪声，直接放行
        return True, "", desc

    sol = _bv(b, "solidity", None)
    if sol is not None:
        desc += " 凸度%.2f" % sol
        if sol < MIN_SOLIDITY:
            return False, "不够凸(有凹口)", desc

    if USE_ROUNDNESS:
        rnd = _bv(b, "roundness", None)
        if rnd is not None:
            desc += " 圆度%.2f" % rnd
            if rnd < MIN_ROUNDNESS:
                return False, "不够圆(边缘太碎)", desc

    return True, "", desc


# ============================== 开机自检 ==============================
def banner():
    print("")
    print("========== balloon_detect 启动 ==========")
    try:
        import os
        print("uname:", os.uname())
    except Exception as e:
        print("uname 读不到:", e)


def _try(tag, fn):
    """跑一个探测项，打 OK/FAIL，返回结果。"""
    try:
        v = fn()
        print("  [OK  ] %-28s %s" % (tag, v if v is not None else ""))
        return v
    except Exception as e:
        print("  [FAIL] %-28s %s" % (tag, e))
        return None


def probe(img):
    print("---------- 能力自检 ----------")
    _try("img.width()", lambda: img.width())
    _try("img.get_statistics()", lambda: _bv(img.get_statistics(), "l_mean"))
    _try("draw_cross(元组)", lambda: img.draw_cross((IMG_W // 2, IMG_H // 2)))
    _try("draw_cross(分开)", lambda: img.draw_cross(IMG_W // 2, IMG_H // 2))
    _try("get_pixel(rgbtuple)", lambda: _pixel(img, IMG_W // 2, IMG_H // 2))

    blobs = _try("find_blobs", lambda: img.find_blobs(
        SCAN_TH, roi=(0, 0, IMG_W, IMG_H),
        pixels_threshold=MIN_PIXELS, area_threshold=MIN_PIXELS,
        merge=True, margin=8))

    if blobs:
        b = blobs[0]
        _try("blob.x()", lambda: b.x())
        _try("blob.w()", lambda: b.w())
        _try("blob.h()", lambda: b.h())
        _try("blob.pixels()", lambda: b.pixels())
        _try("blob.density()", lambda: b.density())
        _try("blob.cx()", lambda: b.cx())
        _try("blob.rect()", lambda: b.rect())
        # ★ 兼容层的结果
        print("  ---- 兼容层读出来 ----")
        print("  x=%s y=%s w=%s h=%s px=%s den=%.2f rect=%s"
              % (b_x(b), b_y(b), b_w(b), b_h(b), b_px(b), b_den(b), b_rect(b)))

        # 哪种写法真的能用？
        try:
            b.w()
            print("  blob 取值：函数式 (4.x)")
        except Exception:
            print("  blob 取值：属性式 (5.x)  -> 这也是刚才 'int' object isn't callable 的来源")
    else:
        print("  （画面里没有彩色块，跳过 blob 取值探测）")

    print("------------------------------")
    print("目标：%s" % ("任何颜色的气球" if TARGET == "ANY" else "只认红球"))
    print("判定方式：%s" % ("不看颜色（形状过了就算）" if TARGET == "ANY" else COLOR_SPACE))
    if COLOR_SPACE == "RGB":
        print("  V>=%d 且 S>=%.2f 且 (H<=%.0f° 或 H>=%.0f°)"
              % (RGB_V_MIN, RGB_S_MIN, HUE_MAX, HUE_WRAP))
    else:
        print("  L>=%d 且 A>=%d 且 B<=%d" % (LAB_L_MIN, LAB_A_MIN, LAB_B_MAX))
    print("  形状：大球 比例<=%.1f 实心 %.2f~%.2f 凸度>=%.2f 圆度>=%.2f"
          % (MAX_ASPECT, MIN_EXTENT, MAX_EXTENT, MIN_SOLIDITY, MIN_ROUNDNESS))
    print("        小球(<%dpx) 比例<=%.1f 实心>=%.2f（凸度/圆度跳过）"
          % (SMALL_W_PX, SMALL_MAX_ASPECT, SMALL_MIN_EXTENT))
    print("  扫描：stride=%d  最小像素=%d  最小宽=%dpx"
          % (X_STRIDE, MIN_PIXELS, MIN_W_PX))


def open_cam():
    try:
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=1500)
        img = sensor.snapshot()
        print("摄像头: sensor 模块")
        return (lambda: sensor.snapshot()), img
    except Exception as e:
        print("sensor 路线失败:", e)

    import csi
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QQVGA)
    img = cam.snapshot(time=1500)
    print("摄像头: csi 模块")
    return (lambda: cam.snapshot()), img


# ============================== 主循环 ==============================
def main():
    banner()

    try:
        snap, first = open_cam()
    except Exception as e:
        _dump("打开摄像头", e)
        return

    probe(first)
    print("开始找气球：扫描 -> 形状 -> %s" % ("不看颜色" if TARGET == "ANY" else COLOR_SPACE))
    print("")

    n = 0
    t0 = _ms()
    errs = 0
    confirm = 0
    cx, cy = IMG_W // 2, IMG_H // 2

    while True:
        n += 1

        try:
            img = snap()
        except Exception as e:
            if errs < 3:
                _dump("snapshot", e)
            errs += 1
            time.sleep_ms(50)
            continue

        stage1 = stage2 = 0
        reds = []
        dropped = []

        try:
            found = img.find_blobs(SCAN_TH,
                                   roi=(0, 0, IMG_W, IMG_H),
                                   x_stride=X_STRIDE, y_stride=Y_STRIDE,
                                   pixels_threshold=MIN_PIXELS,
                                   area_threshold=MIN_PIXELS,
                                   merge=True, margin=8)
        except Exception as e:
            if errs < 3:
                _dump("find_blobs", e)
            errs += 1
            found = []

        stage1 = len(found)

        # ---- 每个 blob 单独 try：一个坏了不影响其它 ----
        for b in found:
            try:
                ok, why, sdesc = shape_ok(b)
                if not ok:
                    dropped.append((b, "形状", "%s  [%s]" % (why, sdesc)))
                    continue
                stage2 += 1

                lab = _lab(img, b_rect(b))

                if TARGET == "ANY":
                    # 不判颜色：只把 RGB/HSV 念出来给你看，不参与判定
                    rgb = _rgb_mean(img, b_rect(b))
                    if rgb is None:
                        info = "RGB=取不到"
                    else:
                        h, s, v = _hsv(rgb[0], rgb[1], rgb[2])
                        info = "RGB=(%3d,%3d,%3d) H=%3.0f° S=%.2f" % (
                            rgb[0], rgb[1], rgb[2], h, s)
                    yes, why = True, "任意颜色"
                elif COLOR_SPACE == "RGB":
                    rgb = _rgb_mean(img, b_rect(b))
                    yes, why, h, s, v = classify_rgb(rgb)
                    if rgb is None:
                        info = "RGB=取不到"
                    else:
                        info = "RGB=(%3d,%3d,%3d) H=%3.0f° S=%.2f V=%d" % (
                            rgb[0], rgb[1], rgb[2], h, s, v)
                else:
                    yes, why = classify_lab(lab)
                    info = "LAB=L%3d A%4d B%4d" % (lab if lab else (0, 0, 0))

                if yes:
                    reds.append((b, info, why, sdesc))
                else:
                    dropped.append((b, "判定", why))
            except Exception as e:
                if errs < 3:
                    _dump("单个 blob", e)
                errs += 1

        target = None
        if reds:
            try:
                    reds.sort(key=lambda x: b_w(x[0]), reverse=True)
            except Exception:
                pass
            target = reds[0]
            confirm += 1
        else:
            confirm = 0

        # ---- 画 ----
        if DRAW:
            try:
                for b, _i, _w, _s in reds:
                    _draw(img, "draw_rectangle", b_rect(b), color=(255, 0, 0), thickness=2)
                for b, _s, _why in dropped[:3]:
                    _draw(img, "draw_rectangle", b_rect(b), color=(80, 80, 80), thickness=1)
                _draw(img, "draw_cross", (cx, cy))
                if target is not None:
                    b = target[0]
                    dist = FOCAL_PX * BALLOON_DIAM_CM / max(1.0, float(b_w(b)))
                    _label(img, b_x(b), max(0, b_y(b) - 10), "%dcm" % int(dist))
            except Exception as e:
                if errs < 3:
                    _dump("画框", e)
                errs += 1

        # ---- 刷数字 ----
        if n % PRINT_EVERY:
            continue

        try:
            now = _ms()
            fps = PRINT_EVERY * 1000.0 / max(1, now - t0)
            t0 = now

            if target is None:
                print("#%03d  fps=%4.1f  没看到气球   颜色 %d -> 形状 %d -> 见过 0"
                      % (n, fps, stage1, stage2))
                if stage1 == 0:
                    print("      → 颜色这关就没了：扫描阈值太小 / 太暗 / 画面里没有红的")
                else:
                    dropped.sort(key=lambda x: b_w(x[0]), reverse=True)
                    for b, seg, why in dropped[:3]:
                        print("      被【%s】筛掉: %dx%d px=%d %s"
                              % (seg, b_w(b), b_h(b), b_px(b), why))
                continue

            b, info, why, sdesc = target
            dx = b_cx(b) - cx
            dist = FOCAL_PX * BALLOON_DIAM_CM / max(1.0, float(b_w(b)))
            angle = dx * 57.3 / FOCAL_PX

            tags = []
            if abs(dx) <= CENTER_BAND:
                tags.append("正前方")
            if b_w(b) >= NEAR_W_PX:
                tags.append("很近")
            if confirm >= CONFIRM_FRAMES:
                tags.append("已确认")

            print("#%03d  fps=%4.1f  气球=%d   颜色 %d -> 形状 %d -> 见过 %d"
                  % (n, fps, len(reds), stage1, stage2, len(reds)))
            print("      dx=%+dpx 角=%+5.1f° 宽=%dpx 距离≈%dcm [%s]  %s  (%s)"
                  % (dx, angle, b_w(b), int(dist), " ".join(tags), info, why))
            print("      形状: %s" % sdesc)
        except Exception as e:
            if errs < 3:
                _dump("打印", e)
            errs += 1


main()
