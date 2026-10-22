/*
 * ============================================================================
 *  pd_light_start.ino —— 光敏启动测试
 *
 *  ★本版只验证 A3 光敏电阻的启动逻辑，不含巡线、不驱动电机：
 *    · 不读 A1/A2 巡线传感器
 *    · 不初始化、不驱动 D2/D4/D5/D6 电机引脚
 *    · 没有 PD、直行、右转并线、左弯计数、终点刹车
 *    收到光信号后只在 D13 和串口上提示，轮子不会动。
 *
 *  接线：光敏模块 S -> A3，V -> 5V，G -> GND
 *        状态灯用板载 D13；扩展板占了 D13 就把 STATUS_LED_PIN 改 255
 *
 *  流程：上电 -> 校准 800ms 环境光 -> D13 慢闪等待
 *        -> 手电照 A3：读数越过阈值，或 200ms 内涨幅够大
 *        -> 连续保持 80ms 确认 -> D13 三下快闪并常亮 -> 串口打印结果
 *
 *  串口：115200，每 200ms 一行 raw / base / thr / d / rise，用来现场标阈值。
 *
 *  ⚠ 本程序完全不碰电机引脚。车上还接着 L298P 时，测试前请断开电机电源
 *    或把四轮架空：驱动板输入悬空可能让轮子抽动。
 * ============================================================================
 */

/* ========================== ① 可调参数区 ========================== */

// —— 状态灯：等待时慢闪，收到信号三下快闪后常亮 ——
//    扩展板占用了 D13 就改成 255（这时没有灯提示，只能看串口）
#define STATUS_LED_PIN        13

// —— 串口调试 ——
#ifndef LIGHT_DEBUG_SERIAL
#define LIGHT_DEBUG_SERIAL    1     // 1 = 115200 打印 raw/base/thr/d/rise；0 = 关闭
#endif
#define LIGHT_PRINT_MS        200   // 打印间隔(ms)

// —— 光敏电阻（A3）——
#define LIGHT_PIN             A3    // 模拟量输出；A1/A2 是巡线口，本程序不用

// 采样与校准
#define LIGHT_OVERSAMPLE      8     // 每次读数取几遍平均（ADC 有噪声）
#define LIGHT_SAMPLE_MS       5     // 等待时的轮询间隔(ms)
#define LIGHT_CALIB_MS        800   // 上电校准环境光的时长（也留出放手时间）
#define LIGHT_CALIB_GAP_MS    4

// 触发判据
#ifndef TRIGGER_ON_BRIGHT        // 台面仿真可用 -D 覆盖
#define TRIGGER_ON_BRIGHT     1   // 1 = 变亮触发（手电）；0 = 变暗触发（挡光）
#endif
// ★实测：环境光 base≈325，手电照到 raw≈557，也就是 d≈230；1 秒内噪声峰峰约 25。
//   取 100 的跳变量：对噪声有 4 倍余量，对手电信号有 2.3 倍余量，两头都稳。
#define LIGHT_DELTA_MIN       100 // 至少比基线跳变这么多（0~1023）
#define LIGHT_DELTA_RATIO_PCT 20  // 或跳变 >= 基线的这个百分比，两者取大
#define LIGHT_STABLE_MS       80  // 信号要持续这么久才算数（防手电一晃）

// ★快速上升沿判据
//   TEMT6000 在亮环境接近饱和——环境 base=478 时，手机手电只能把 raw 再顶高
//   约 79，而绝对阈值要 100，永远差一点。但“照上去”是一瞬间的事，环境光是慢变的，
//   所以再加一条：短窗口内涨幅够大也算收到信号。
//   实测：手电在 200ms 内能把 raw 拉高 40~80；环境光一分钟才变几个数。
#define LIGHT_RISE_MS         200 // 上升沿观察窗口(ms)
#define LIGHT_RISE_MIN        40  // 窗口内涨这么多就算“被照到”

// 环境光慢跟随（等待期间）
#define LIGHT_TRACK_MS        200
#define LIGHT_TRACK_ALPHA_PCT 10
#define LIGHT_TRACK_GUARD     30  // 当前值离阈值不到这个距离就不跟，别把信号吃掉

// 等太久警告
#define LIGHT_WAIT_WARN_MS    60000 // 等这么久还没信号就快闪警告，然后继续等

/* ========================== ② 光信号检测（A3 光敏） ========================== */

int  g_baseline  = 0;      // 环境光基线
int  g_threshold = 0;      // 触发阈值
int  g_noise     = 0;      // 校准时环境光的抖动幅度
int  g_lightRaw  = 0;      // 最近一次读数（调试）
int  g_lightRise = 0;      // 上升沿窗口内的涨幅（调试 + 判据）

// 去抖 / 跟随 / 打印的计时状态（全局量，方便复位）
unsigned long g_sigSinceMs = 0;
int           g_winMin     = 1023;  // 上升沿窗口内的最小值
unsigned long g_winMs      = 0;     // 窗口起点
unsigned long g_warnAtMs   = 0;
unsigned long g_trackAtMs  = 0;
unsigned long g_printAtMs  = 0;

void lightStartReset();     // 前置声明

// 读一次光敏：LIGHT_OVERSAMPLE 遍取平均（ADC 有噪声）
int lightReadRaw() {
  long sum = 0;
  for (uint8_t i = 0; i < LIGHT_OVERSAMPLE; i++) sum += analogRead(LIGHT_PIN);
  return (int)(sum / LIGHT_OVERSAMPLE);
}

// 按基线算触发阈值：跳变量取 LIGHT_DELTA_MIN 和基线百分比里较大的那个
int lightThresholdFor(int base) {
  int delta = LIGHT_DELTA_MIN;
  int ratio = (int)((long)base * LIGHT_DELTA_RATIO_PCT / 100);   // long，防溢出
  if (ratio > delta) delta = ratio;
  int thr = TRIGGER_ON_BRIGHT ? (base + delta) : (base - delta);
  // ★环境本身很亮时 base+delta 顶穿 1023 -> 永远等不到信号。
  //   自动把跳变量减半：宁可灵敏一点，也不要永远不触发。
  if (TRIGGER_ON_BRIGHT && thr > 1000) thr = base + (delta / 2);
  return constrain(thr, 0, 1023);
}

// 上电先量环境光：基线、抖动幅度、触发阈值
void calibrateAmbient() {
  long sum = 0;
  int  mn = 1023, mx = 0;
  uint16_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < LIGHT_CALIB_MS) {
    int v = lightReadRaw();
    sum += v; n++;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    delay(LIGHT_CALIB_GAP_MS);
  }
  g_baseline = (n > 0) ? (int)(sum / n) : 0;
  g_noise    = mx - mn;
  g_threshold = lightThresholdFor(g_baseline);

#if LIGHT_DEBUG_SERIAL
  Serial.print(F("[light] 校准完成 base="));
  Serial.print(g_baseline);
  Serial.print(F(" 抖动="));
  Serial.print(g_noise);
  Serial.print(F(" 阈值="));
  Serial.print(g_threshold);
  Serial.print(F(" 判据="));
  Serial.println(TRIGGER_ON_BRIGHT ? F("变亮触发") : F("变暗触发"));
  if (TRIGGER_ON_BRIGHT && g_threshold >= 1010) {
    Serial.println(F("[light] ★警告：环境太亮，阈值顶到天花板 —— 手电照不亮它。"));
    Serial.println(F("        对策：改成 TRIGGER_ON_BRIGHT 0 用挡光触发，或给传感器加遮光筒"));
  }
  if (g_noise >= LIGHT_DELTA_MIN / 2) {
    Serial.println(F("[light] ★警告：环境光本身抖得厉害 —— 加大 LIGHT_DELTA_MIN"));
  }
#endif
  lightStartReset();       // 校准完，去抖/跟随的计时状态一起归零
}

// 复位去抖/跟随状态：校准完、或者要重新等一次时调用
void lightStartReset() {
  g_sigSinceMs = 0;
  g_winMin     = 1023;
  g_winMs      = millis();
  g_warnAtMs   = millis() + LIGHT_WAIT_WARN_MS;
  g_trackAtMs  = millis();
  g_printAtMs  = millis();
}

// 非阻塞版：调一次检查一遍。返回 true = 光信号已确认
bool lightStartPoll() {
  unsigned long now = millis();
  g_lightRaw = lightReadRaw();

  /* ---- 快速上升沿：窗口内涨幅 ---- */
  if (now - g_winMs >= LIGHT_RISE_MS) { g_winMs = now; g_winMin = 1023; }
  if (g_lightRaw < g_winMin) g_winMin = g_lightRaw;
  g_lightRise = g_lightRaw - g_winMin;

  /* ---- 去抖：信号要连续保持 LIGHT_STABLE_MS 才算数 ---- */
  bool absSig = TRIGGER_ON_BRIGHT ? (g_lightRaw >= g_threshold)
                                  : (g_lightRaw <= g_threshold);
  // 变亮触发时，绝对值 或 快速上升沿 任一满足都算
  bool sig = absSig || (TRIGGER_ON_BRIGHT && g_lightRise >= LIGHT_RISE_MIN);

  if (sig) {
    if (g_sigSinceMs == 0) g_sigSinceMs = now;
    if (now - g_sigSinceMs >= LIGHT_STABLE_MS) {        // ★确认
#if STATUS_LED_PIN != 255
      for (uint8_t i = 0; i < 3; i++) {                 // 三下快闪 = 收到
        digitalWrite(STATUS_LED_PIN, HIGH); delay(60);
        digitalWrite(STATUS_LED_PIN, LOW);  delay(60);
      }
      digitalWrite(STATUS_LED_PIN, HIGH);               // 之后常亮，表示已触发
#endif
#if LIGHT_DEBUG_SERIAL
      Serial.print(F("[light] ★收到光信号，触发！ raw="));
      Serial.print(g_lightRaw);
      Serial.print(F(" base="));
      Serial.print(g_baseline);
      Serial.print(F(" 用时="));
      Serial.print(now);
      Serial.println(F("ms"));
#endif
      return true;
    }
  } else {
    g_sigSinceMs = 0;                                   // 断了就重新计
  }

  /* ---- 环境光慢跟随：只在「离阈值还远」时跟，免得把信号自己吃掉 ---- */
  if (now - g_trackAtMs >= LIGHT_TRACK_MS) {
    g_trackAtMs = now;
    bool farFromThreshold = TRIGGER_ON_BRIGHT ? (g_lightRaw < g_threshold - LIGHT_TRACK_GUARD)
                                              : (g_lightRaw > g_threshold + LIGHT_TRACK_GUARD);
    if (farFromThreshold) {
      // ★必须用 long：AVR 的 int 只有 16 位，base=495 时 495*90=44550 会溢出成负数，
      //   于是阈值算成 0、raw>=0 恒成立 —— 没照手电也会立刻触发（2026-10-18 实测踩到）。
      g_baseline  = (int)(((long)g_baseline * (100 - LIGHT_TRACK_ALPHA_PCT) +
                           (long)g_lightRaw * LIGHT_TRACK_ALPHA_PCT) / 100);
      g_threshold = lightThresholdFor(g_baseline);   // 基线动了，阈值跟着动
    }
  }

  /* ---- 状态灯：慢闪 = 还在等人照 ---- */
#if STATUS_LED_PIN != 255
  digitalWrite(STATUS_LED_PIN, ((now / 500) & 1) ? HIGH : LOW);
#endif

  /* ---- 等太久给个警告（快闪一下，然后继续等）---- */
  if (now >= g_warnAtMs) {
    g_warnAtMs = now + LIGHT_WAIT_WARN_MS;
#if STATUS_LED_PIN != 255
    for (uint8_t i = 0; i < 5; i++) {
      digitalWrite(STATUS_LED_PIN, HIGH); delay(50);
      digitalWrite(STATUS_LED_PIN, LOW);  delay(50);
    }
#endif
#if LIGHT_DEBUG_SERIAL
    Serial.println(F("[light] 还没等到光信号（继续等）。检查手电方向/亮度、LIGHT_DELTA_MIN 是否过大"));
#endif
  }

  /* ---- 周期打印，方便现场调阈值 ---- */
#if LIGHT_DEBUG_SERIAL
  if (now - g_printAtMs >= LIGHT_PRINT_MS) {
    g_printAtMs = now;
    Serial.print(F("raw="));   Serial.print(g_lightRaw);
    Serial.print(F(" base=")); Serial.print(g_baseline);
    Serial.print(F(" thr="));  Serial.print(g_threshold);
    Serial.print(F(" d="));    Serial.print(g_lightRaw - g_baseline);
    Serial.print(F(" rise=")); Serial.print(g_lightRise);
    Serial.println(sig ? F("  <-- 信号中") : F(""));
  }
#endif
  return false;
}

// 阻塞版：一直等到光信号（setup() 里调它）
void waitForLightStart() {
#if LIGHT_DEBUG_SERIAL
  Serial.println(F("[light] 等光信号：拿手电照 A3 光敏（人不用碰车）"));
#endif
  while (!lightStartPoll()) delay(LIGHT_SAMPLE_MS);
}

/* ========================== ③ 初始化与主循环 ========================== */

void setup() {
#if STATUS_LED_PIN != 255
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
#endif

  pinMode(LIGHT_PIN, INPUT);

#if LIGHT_DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("=== pd_light_start：光敏启动测试（不巡线、不驱动电机）==="));
#endif

  calibrateAmbient();       // 这 800ms 也正好留出放手时间
  waitForLightStart();      // 收到信号前一直等

#if LIGHT_DEBUG_SERIAL
  Serial.println(F("[light] 测试完成：光信号已确认。程序停在提示状态，电机未驱动。"));
#endif
}

void loop() {
  // 只做光敏启动测试：收到信号后不再做任何事，避免误驱动电机。
}
