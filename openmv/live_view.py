# ===========================================================================
#  live_view.py —— 一直看着，而且**出错也不死**
#
#  两件事：
#    ① 开跑前先做一遍「能力探测」：固件版本、width() 是不是函数、
#       statistics 是函数还是整数、draw_cross 吃不吃元组、find_blobs 行不行
#    ② 之后一直循环。中间任何一步出错，**只打印一次错误 + 行号**，然后继续跑，
#       不会像之前那样整个脚本直接停掉
#
#  ★ 所以如果还是不行：把 IDE 下面那一整段（尤其是 "File ... line N" 和
#    "TypeError ..." 那几行）发我，就能精确定位了。
#
#  怎么看：画面里红框=红球，绿框=可扎，中间十字=量 LAB 的点；
#          每 15 帧刷一行 fps / 红数 / 非红数 / 中心 LAB。
# ===========================================================================

import sys
import time

# ============================== 只看这几行 ==============================
FPS_EVERY  = 15
DRAW       = True
IMG_W, IMG_H = 160, 120

TH_RED = [(45, 95, 25, 75, -20, 35)]
TH_OTHER = [
    (55, 100, 5, 60, 35, 95),
    (65, 100, -20, 30, 30, 95),
    (45, 95, -70, -10, 0, 60),
    (45, 95, -45, 20, -90, -20),
    (40, 90, 5, 60, -75, -10),
]
TH_ALL = TH_RED + TH_OTHER

MIN_PIXELS = 40
MERGE      = 8
RED_A_MIN  = 25
RED_B_MAX  = 35
RED_L_MIN  = 45

try:
    _ms = time.ticks_ms
except AttributeError:
    _ms = lambda: int(time.time() * 1000)


# ============================== 兼容层 ==============================
def _val(obj, name, default=0):
    """statistics 字段：4.x 是函数，5.x 是整数，两种都吃。"""
    v = getattr(obj, name, default)
    if callable(v):
        try:
            return v()
        except Exception:
            return default
    return v


def _lab(img, rect):
    try:
        st = img.get_statistics(roi=rect)
        return (_val(st, "l_mean"), _val(st, "a_mean"), _val(st, "b_mean"))
    except Exception:
        return (0, 0, 0)


# ---- blob 取值兼容层：4.x 是函数，5.x 可能直接是属性 ----
def _bv(obj, name, default=0):
    """是函数就调，是值就直接用。"""
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


def _is_red(img, rect):
    l, a, b = _lab(img, rect)
    if l < RED_L_MIN:
        return True
    return a >= RED_A_MIN and b <= RED_B_MAX


def _dump(e):
    """打印完整 traceback（含行号）。MicroPython 有 sys.print_exception，
    没有就退化成一行。"""
    try:
        sys.print_exception(e)
    except Exception:
        print("    异常:", e)


def _draw(img, name, coords, **kw):
    """draw_xxx：5.0 要元组，4.x 要分开；两个都试，画不上就算了。"""
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


# ============================== 开机自检 ==============================
def banner():
    print("")
    print("========== live_view 启动 ==========")
    try:
        import os
        print("uname:", os.uname())
    except Exception as e:
        print("uname 读不到:", e)
    try:
        print("version:", sys.version)
    except Exception:
        pass


def probe(img):
    """把每样能力单独试一遍，全部打印出来 —— 这一段就是诊断书。"""
    print("---------- 能力探测 ----------")

    # ① 尺寸
    try:
        print("width() =", img.width(), "  height() =", img.height())
    except Exception as e:
        print("width()/height() 失败:", e)

    # ② statistics 是函数还是整数
    try:
        st = img.get_statistics()
        v = getattr(st, "l_mean", None)
        print("get_statistics ok；l_mean 是", "函数(4.x)" if callable(v) else "整数(5.x)", "=", _val(st, "l_mean"))
    except Exception as e:
        print("get_statistics 失败:", e)

    # ③ draw_cross 吃哪种写法
    for tag in ("元组", "分开"):
        try:
            if tag == "元组":
                img.draw_cross((IMG_W // 2, IMG_H // 2))
            else:
                img.draw_cross(IMG_W // 2, IMG_H // 2)
            print("draw_cross 吃【%s】写法" % tag)
            break
        except Exception as e:
            print("draw_cross 【%s】写法失败:" % tag, e)

    # ④ find_blobs + blob 访问器
    try:
        blobs = img.find_blobs([(0, 100, -128, 127, -128, 127)],
                               roi=(0, 0, IMG_W, IMG_H),
                               pixels_threshold=MIN_PIXELS,
                               area_threshold=MIN_PIXELS,
                               merge=True, margin=MERGE)
        print("find_blobs ok，块数 =", len(blobs))
        if blobs:
            b = blobs[0]
            print("blob 访问器: x=%d y=%d w=%d h=%d px=%d"
                  % (b_x(b), b_y(b), b_w(b), b_h(b), b_px(b)))
    except Exception as e:
        print("find_blobs 失败:", e)
    print("------------------------------")


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
        print("!!! 摄像头打不开")
        _dump(e)
        return

    probe(first)

    print("开始循环。红框=红球，绿框=可扎，十字=量 LAB 的点")
    print("")

    n = 0
    t0 = _ms()
    errs = 0
    cx, cy = IMG_W // 2, IMG_H // 2

    while True:
        n += 1

        # ---- 取图 ----
        try:
            img = snap()
        except Exception as e:
            if errs < 3:
                print("!!! snapshot 出错：")
                _dump(e)
            errs += 1
            time.sleep_ms(50)
            continue

        # ---- 找 blob ----
        reds, others = [], []
        try:
            blobs = img.find_blobs(TH_ALL,
                                   roi=(0, 0, IMG_W, IMG_H),
                                   x_stride=2, y_stride=2,
                                   pixels_threshold=MIN_PIXELS,
                                   area_threshold=MIN_PIXELS,
                                   merge=True, margin=MERGE)
            for b in blobs:
                if b_px(b) < MIN_PIXELS:
                    continue
                if _is_red(img, b_rect(b)):
                    reds.append(b)
                else:
                    others.append(b)
        except Exception as e:
            if errs < 3:
                print("!!! find_blobs / blob 出错：")
                _dump(e)
            errs += 1

        # ---- 画 ----
        if DRAW:
            try:
                for b in reds:
                    _draw(img, "draw_rectangle", b_rect(b), color=(255, 0, 0), thickness=2)
                for b in others:
                    _draw(img, "draw_rectangle", b_rect(b), color=(0, 255, 0), thickness=1)
                _draw(img, "draw_cross", (cx, cy))
            except Exception as e:
                if errs < 3:
                    print("!!! 画框出错：")
                    _dump(e)
                errs += 1

        # ---- 刷数字 ----
        if n % FPS_EVERY == 0:
            now = _ms()
            fps = FPS_EVERY * 1000.0 / max(1, now - t0)
            t0 = now
            l, a, bb = _lab(img, (cx - 6, cy - 6, 12, 12))
            print("fps=%4.1f  红=%d 非红=%d   中心 LAB = L%3d A%4d B%4d"
                  % (fps, len(reds), len(others), l, a, bb))


main()
