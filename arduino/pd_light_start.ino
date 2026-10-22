/*
 * ============================================================================
 *  pd_light_start.ino —— 双路巡线小车（比赛版 + 光信号启动）
 *  本版 = pd_v3.0+1.01 原样 + 光敏启动（A3）；pd_v3.0+1.01 保持不动。
 *  控制逻辑与 pd_v3.0+1.01 完全一致，只在 setup() 前加了光信号等待。
 *  带完整原理注释的原版（`.backup/` 已随 2026-10-17 整理删除，本文件注释为准）。
 *
 *  流程：上电 -> 等 START_DELAY_MS -> 直行进场（忽略出发区边框线，扫到边线提前右转）
 *        -> 右转并线 -> 上线补偿 300ms -> PD 巡线 + 左弯计数（逆时针 4 个左弯）
 *        -> 第 4 弯与前面完全同路 -> 贴稳后两路全白 FINISH_WHITE_MS -> 反脉冲刹车停车
 *        ★没有“左转进场”（不进场地，停在线上）；要进场得另外加一段。
 *
 *  接线：巡迹 S1 左 -> A1、S2 右 -> A2、V -> 5V、G -> GND
 *        L298P A 通道（D2 方向 / D6 调速）= 左侧两轮
 *              B 通道（D4 方向 / D5 调速）= 右侧两轮
 *
 *  五段分工：① 直行 doStraight()：开环，只数跨过几根线
 *            ② 右转并线 doTurnRight()：绕右轮划弧，任一路压线即并入，找不到就一直转
 *            ③ 上线补偿 mergeSettle()：限速 + 修正打折 + 按方向记忆带一把
 *            ④ 巡线 pdControl()：三档偏差 PD；两路全白交 whiteSearch() 划弧找回
 *            ⑤ 左弯 updateCorner()：ARC -> PIVOT -> SEARCH 三拍，真弯看“总白线时长”
 *
 *  ★改代码前必看：
 *    · 前两拍有硬上限 CORNER_TIMEOUT_MS(500ms ≈ 90°)：绝不允许原地盲转过半圈，
 *      否则探头会扫回来时那根线，车顺着旧线反着走。
 *    · 同一个弯“重转”上限 CORNER_MAX_RETRIES = 1，超了交给丢线找回。
 *    · 光信号启动已并入（A3）；赛前只需标定 LIGHT_DELTA_MIN、左转进场。
 *
 *  停车原因看 D13 闪灯：1 闪 = 总时间保护；2 闪 = 正常终点。
 *
 *  ★光信号启动：LIGHT_START_ENABLED = 1，光敏模块 S -> A3。
 *    上电先校准环境光 800ms，然后等手电信号才起步；
 *    调试时把 LIGHT_START_ENABLED 改 0，退回 START_DELAY_MS 自动起步。
 * ============================================================================
 */
/* ========================== ① 可调参数区 ========================== */

// —— 传感器 ——
#define BLACK_LEVEL HIGH                    // 检测到黑线时模块输出的电平（拿不准先烧 line_sensor_test）
#define PROBES_CENTERED_BOTH_ON_LINE 1      // 1 = 骑在线上时两路都压线（本车，探头间距 12mm < 线宽）
                                            //   0 = 两路都不压线（老装法）。填错则丢线找回和终点全乱
#define WHITE_MEANS_LOST (PROBES_CENTERED_BOTH_ON_LINE != 0)

// —— 电机 ——
#define A_CHANNEL_IS_LEFT 1                 // 1 = A 通道(D2/D6) 接左侧两轮（本车实测）
#define LEFT_MOTOR_INVERT  0                // 实际转向与程序相反时改成 1
#define RIGHT_MOTOR_INVERT 0

// —— 巡线速度与 PD ——
#define BASE_SPEED         150   // 巡线基础速度
#define MIN_SPEED          60    // 最小有效速度：低于它电机不转，setMotor 直接给 0（防堵转）
#define MAX_SPEED          220   // 单轮最大速度限幅
#define KP                 30.0  // 比例：单路压线(偏差±1)修正量 = ±30。回正慢就加大，画龙就是过头
#define KD                 25.0  // 微分：状态切换时的阻尼。过大会高频抖动
#define MAX_CORRECTION     140   // 修正量限幅（当前最多用到 55，留余量）
#define SAMPLE_INTERVAL_MS 5     // 巡线控制周期(ms)：固定周期 Kd 才有意义

// —— 进场：等待 + 直行 + 右转 ——
#define START_DELAY_MS        2000  // 上电后等这么久起步
#define START_STRAIGHT_SPEED  150   // 直行速度
#define START_STRAIGHT_MS     1500  // 直行最长时长：调到“到点时车头正好在边线附近”
#define START_IGNORE_LINES    1     // 直行先忽略几根线：出发区边框线 = 1
#define START_TURN_SPEED      150   // 右转外轮（左轮）速度
#define START_TURN_INNER_SPEED 0    // 右转内轮（右轮）速度：0 = 绕右轮划弧；-150 = 原地右转
                                    //   ★|值| < MIN_SPEED 会被当成 0

// —— 上线补偿（并线成功后那一段）——
#define MERGE_SETTLE_MS    300   // 补偿期最长时长；期内骑正就提前结束
#define MERGE_SETTLE_SPEED 120   // 补偿期基准速度
#define MERGE_SETTLE_GAIN  0.5   // 补偿期 PD 修正打折系数
#define MERGE_BIAS         25    // 固定偏置：朝“线最后在哪一侧”带一把，压住回正过头

// —— 丢线（两路全白）找回 ——
// 两个探头都没信息时，唯一能用的是“上次看到线在哪一侧”
#define WHITE_SEARCH_SPEED    140  // 找回时的基准速度
#define WHITE_SEARCH_BIAS     70   // 刚丢线时的转向力度
#define WHITE_SEARCH_MAX_BIAS 140  // 力度上限
#define WHITE_SEARCH_RAMP_MS  500  // 加到上限所用时间，按平方曲线（线性一丢线就顶到地板太猛）

// —— 左弯（逆时针绕行，四个直角弯全是左弯）——
#define LEFT_TURN_TARGET      4     // 数满就不再找弯，进终点流程

// 起转判据①“只左路压线够久”：★默认关。直道摆动就会满足 -> 原地左拧出赛道
#define CORNER_LEFT_HOLD_ENABLE 0   // 1 = 打开（有直道误判风险）
#define CORNER_STUCK_LEFT_MS  180   // 判据①的时间门槛(ms)，仅在上面 = 1 时有效

// 起转判据②“线从左侧（或正下方）消失”：主判据，两路全白要连续白够门槛才算真丢线。
// 窗口里 whiteSearch 已经在朝线消失那侧划弧，所以真弯是“先划弧再硬拧”，不比起转晚多少
#define CORNER_LOST_CONFIRM_MS 60   // ★调法：直道还误判就加大(80/100)；真弯冲过弯就减小(40)
#define CORNER_LOST_SAMPLES   2     // 周期数下限，再挡一层单周期毛刺

// —— ★预判左弯：起转前先“减速 + 轻左带”，专治“冲过弯一大段才开始转” ——
// 它只是预判：线一回到中间/跑到右侧就立刻退出，代价只是慢一下，不会转错
#define CORNER_PRE_ENABLE     1     // 1 = 启用预判（推荐）；0 = 退回旧行为
#define CORNER_PRE_MS         80    // “只左路压线”持续这么久进预判。直线老是在减速就加大
#define CORNER_PRE_INNER      80    // 预判期左轮（内轮）速度
#define CORNER_PRE_OUTER      150   // 预判期右轮（外轮）速度（平均 ~115，比 BASE 慢）
#define CORNER_PRE_COOLDOWN_MS 250  // 退出预判后这么久内不再进预判
#define CORNER_COOLDOWN_MS    450   // 出弯后这么久内不再检测新弯
#define CORNER_ARC_MS         50    // 第一拍“弧线切入”时长：两轮都滚 = 平顺不打滑
#define CORNER_ARC_INNER      70    // 弧线拍左轮速度 ★这一拍保持正转，给车前进量把弯切进去
#define CORNER_PIVOT_INNER   -90    // ★第二拍“原地拧”左轮速度：**负 = 往后退**，旋转中心移进车体
                                    //   ★死区：要么填 0、要么 ≤ -60，填 -30 无效
                                    //   ★调法：还想更紧就 -120/-150；转过头就往回收 -60/-70
                                    //   ★电流：内轮反转 + 外轮 210 是电流最大的一拍，电池弱的
                                    //     话电压被拉低会把 UNO 拉复位（车突然停 2 秒再从头来）
#define CORNER_OUTER_SPEED    210   // 外轮（右轮）速度：弧线切入和原地拧都用它
#define CORNER_MIN_TURN_MS    100   // 起转后至少转这么久才允许判“转完”
#define CORNER_REAL_MIN_MS    200   // ★算“真弯”的最短**总白线时长** = 起转前白的 + 转弯中白的
                                    //   （不是转弯用时）。定太高会漏计真弯（弯数到不了 4、车一直绕）
#define CORNER_RETRY_MS       1200  // 同一个弯的“重转窗口”：出弯后这么久内又进弯 = 没转干净
#define CORNER_MAX_RETRIES    1     // ★同一个弯最多允许重转几次，超了关掉重转窗口交给丢线找回。
                                    //   没有上限会越拧越偏，最后车头转过 180° 顺着来时那根线反着走
#define CORNER_EXIT_GRACE_MS  30    // 完成判据的保持窗口：扫得快时“骑正”可能只有一两个周期
#define CORNER_TIMEOUT_MS     500   // ★前两拍时长上限(≈90°)：到点切第三拍“边前进边左转”。绝不能
                                    //   原地盲转过半圈，否则会扫回来时那根线反着走
                                    //   ★调法：老要靠第三拍才找到线就加大(600~700)，还转过头就减(350~400)
#define CORNER_SEARCH_INNER   90    // 第三拍左轮（慢转，让线在探头下停久一点）
#define CORNER_SEARCH_OUTER   180   // 第三拍右轮（快转，车头继续往左扫）。方向只朝左，绝不反向

// —— 终点：没有“终点段”，就是“两路全白即停” ——
// 第 4 弯和前面 3 个弯走完全一样的路，唯一附加规则是“线没了就刹车”。两个门槛都为了别误停：
#define FINISH_ARM_MS         300   // ① 出弯后线要连续看到这么久才算“重新贴稳”，才开始认终点
                                    //    （没这一步，刚出弯那一下的短暂全白会让车在弯里就刹停）
#define FINISH_WHITE_MS       300   // ② 贴稳之后全白持续这么久 = 真到终点线（灰段约 25cm 宽）
#define FINISH_RECOVER_BIAS   45    // ③ 白着时朝“线最后在哪一侧”轻带一把：车自己漂出去能扫回来
#define FINISH_BRAKE_MS       80      // 停之前给一下反脉冲：停得死、不往前溜
#define FINISH_BRAKE_SPEED    120     // ★必须 >= MIN_SPEED(60)，否则 setMotor 当 0，等于没刹
#define LINE_END_TIMEOUT_MS   5000    // 线消失兜底：任何一路连续这么久扫不到就停车（不看弯数）

// —— 总运行时间保护（所有“永不停车”的兜底）——
#define RUN_SAFETY_TIMEOUT_MS 90000 // 从起步算起超过这么久就停车（0 = 关闭）

// —— 停车原因指示：停车后 D13 闪 N 下（闪 N 下 -> 停 1.2 秒 -> 再重复）——
//   1 闪 = 总时间保护；2 闪 = 正常终点。扩展板占了 D13 就改 255 关掉
#define STATUS_LED_PIN        13

// —— 光信号启动（光敏电阻，A3）——
//   规则一(二)/三.3：必须非接触启动，人不能进场。
//   1 = 等光信号才起步（比赛用）；0 = 退回 START_DELAY_MS 自动起步（调试用）
#ifndef LIGHT_START_ENABLED
#define LIGHT_START_ENABLED   1     // ★当前 = 1：等光信号才起步（比赛档）
                                    //   台面调试想跳过等待：-DLIGHT_START_ENABLED=0
#endif
#ifndef LIGHT_DEBUG_SERIAL
#define LIGHT_DEBUG_SERIAL    1     // 1 = 串口 115200 打印 raw/base/thr/d（调阈值用）
#endif
#define LIGHT_PRINT_MS        200   // 打印间隔(ms)

// 接线：光敏模块 S -> A3（模拟量输出；A1/A2 已被巡线占用，A4/A5 给 OpenMV）
#define LIGHT_PIN             A3

// 采样与校准
#define LIGHT_OVERSAMPLE      8     // 每次读数取几遍平均（ADC 有噪声）
#define LIGHT_SAMPLE_MS       5     // 等待时的轮询间隔(ms)
#define LIGHT_CALIB_MS        800   // 上电校准环境光的时长（也留出放手时间）
#define LIGHT_CALIB_GAP_MS    4

// 触发判据
#ifndef TRIGGER_ON_BRIGHT        // 台面仿真可用 -D 覆盖
#define TRIGGER_ON_BRIGHT     1   // 1 = 变亮触发（手电）；0 = 变暗触发（挡光）
#endif
// ★2026-10-18 实测：手机手电照不亮它。原因是阈值取「固定值 和 基线百分比」的较大者：
//     base=700 时 delta = max(250, 700×50%) = 350 -> 阈值 1050 > ADC 上限 1023，
//     永远触发不了。现在两个都调小，并且顶到天花板时自动折半。
// ★2026-10-18 实测（A3 接对之后）：环境光 base≈325，手电照到 raw≈557，
//   也就是 d≈230；1 秒内噪声峰峰约 25。
//   取 100 的跳变量：对噪声有 4 倍余量，对 230 的手电信号有 2.3 倍余量，两头都稳。
#define LIGHT_DELTA_MIN       100 // 至少比基线跳变这么多（0~1023）
#define LIGHT_DELTA_RATIO_PCT 20  // 或跳变 >= 基线的这个百分比，两者取大
#define LIGHT_STABLE_MS       80  // 信号要持续这么久才算数（防手电一晃）

// ★快速上升沿判据（2026-10-18 加）
//   背景：TEMT6000 在亮环境接近饱和——环境 base=478 时，手机手电只能把 raw 再顶高
//   约 79，而绝对阈值要 100，永远差一点。但"照上去"是一瞬间的事，环境光是慢变的，
//   所以再加一条：**短窗口内涨幅够大**也算收到信号。
//   实测：手电在 200ms 内能把 raw 拉高 40~80；环境光一分钟才变几个数。
#define LIGHT_RISE_MS         200 // 上升沿观察窗口(ms)
#define LIGHT_RISE_MIN        40  // 窗口内涨这么多就算"被照到"

// 环境光慢跟随（等待期间）
#define LIGHT_TRACK_MS        200
#define LIGHT_TRACK_ALPHA_PCT 10
#define LIGHT_TRACK_GUARD     30  // 当前值离阈值不到这个距离就不跟，别把信号吃掉

// 等太久警告
#define LIGHT_WAIT_WARN_MS    60000 // 等这么久还没信号就快闪警告，然后继续等

/* ========================== ② 引脚定义 ========================== */

const uint8_t SENSOR_LEFT_PIN  = A1;   // 左路（A1 是模拟口，当数字口用）
const uint8_t SENSOR_RIGHT_PIN = A2;   // 右路

#if A_CHANNEL_IS_LEFT
const uint8_t LEFT_MOTOR_DIR  = 2;   // 左侧两轮：A 通道方向脚
const uint8_t LEFT_MOTOR_PWM  = 6;   // 左侧两轮：A 通道调速脚
const uint8_t RIGHT_MOTOR_DIR = 4;   // 右侧两轮：B 通道方向脚
const uint8_t RIGHT_MOTOR_PWM = 5;   // 右侧两轮：B 通道调速脚
#else
const uint8_t LEFT_MOTOR_DIR  = 4;
const uint8_t LEFT_MOTOR_PWM  = 5;
const uint8_t RIGHT_MOTOR_DIR = 2;
const uint8_t RIGHT_MOTOR_PWM = 6;
#endif

const uint8_t MOTOR_FORWARD_LEVEL  = HIGH;   // 本扩展板前进时方向脚均为 HIGH
const uint8_t MOTOR_BACKWARD_LEVEL = LOW;

/* ========================== ②.5 光信号启动（A3 光敏） ========================== */
// 与 car_full 的光敏启动段保持一致（2026-10-22 对齐，实现一字不差）：
// 自适应基线 + 快速上升沿 + 阈值触顶自动折半。等待期间电机已在 setup() 里 carStop()。

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
      digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#if LIGHT_DEBUG_SERIAL
      Serial.print(F("[light] ★收到光信号，起步！ raw="));
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
      //   于是阈值算成 0、raw>=0 恒成立 —— 没照手电也会立刻起步（2026-10-18 实测踩到）。
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
#if !LIGHT_START_ENABLED
#if LIGHT_DEBUG_SERIAL
  Serial.println(F("[light] LIGHT_START_ENABLED = 0：不等待，直接起步（调试模式）"));
#endif
  return;
#else
#if LIGHT_DEBUG_SERIAL
  Serial.println(F("[light] 等光信号启动：拿手电照传感器（人不要进场地）"));
#endif
  while (!lightStartPoll()) delay(LIGHT_SAMPLE_MS);
#endif
}

/* ========================== ③ 状态量 ========================== */

enum Phase : uint8_t {                 // 大阶段
  PHASE_IDLE = 0,                      // 停下来了（闪灯报停车原因）
  PHASE_STRAIGHT,                      // 直行进场
  PHASE_TURN_RIGHT,                    // 右转并线
  PHASE_LINE                           // 巡线（含左弯、终点）
};

enum CornerState : uint8_t {           // 左弯的三拍
  CORNER_NONE = 0,                     // 没在转弯
  CORNER_ARC,                          // 拍 1：弧线切入（两轮都滚）
  CORNER_PIVOT,                        // 拍 2：原地拧（内轮收 0）
  CORNER_SEARCH                        // 拍 3：边前进边左转的搜索弧
};

enum StopCode : uint8_t {
  STOP_NONE = 0,                       // 没停过
  STOP_SAFETY,                         // 1 闪：超过总时间保护
  STOP_FINISH                          // 2 闪：正常终点（黑线走完）
};

uint8_t g_phase      = PHASE_IDLE;
uint8_t g_cornerState = CORNER_NONE;
uint8_t g_stopCode   = STOP_NONE;

bool  g_black[2]      = { false, false };  // 左、右两路是否检测到黑线
int8_t g_lastSide     = 0;                 // 最后看到线在哪一侧：-1 只有左路 / 0 居中或全白 / +1 只有右路
float g_lastError     = 0.0;               // 上一次偏差（算微分用）
float g_lastDeflectDir = 0.0;              // 最近一次“真正的偏转方向”：只由单路压线刷新
                                           // （不能用 g_lastError：居中的 0 会把方向抹掉）

unsigned long g_lastSampleMs   = 0;    // 上次控制采样时刻（巡线固定 5ms 周期用）
unsigned long g_runStartMs     = 0;    // 本次运行起步时刻（总时间保护用）
unsigned long g_phaseStartMs   = 0;    // 进场阶段（直行 / 右转）开始时刻
unsigned long g_sideSinceMs    = 0;    // 当前这一侧（g_lastSide）已经持续多久的起点

// —— 进场 ——
uint8_t  g_linesPassed   = 0;          // 直行时完整跨过了几根线（压线->离开才算一次）
bool     g_straightOnLine = false;     // 直行时当前是否正压着线
bool     g_turnGateOpen  = false;      // 右转闸门：旧线离开探头后才开始认新线

// —— 巡线 ——
unsigned long g_lostSinceMs    = 0;    // 开始丢线（两路全白）的时刻，0 = 当前没丢线
unsigned long g_lastLineSeenMs = 0;    // 最后一次“任一路压线”的时刻（线消失兜底用），0 = 还没看到过
unsigned long g_settleUntilMs  = 0;    // 上线补偿期截止时刻，0 = 不在补偿期
float         g_settleDir      = 0.0;  // 补偿期的偏置方向：+1 = 往右带（线最后在右），-1 = 往左带

// —— 左弯 ——
uint8_t  g_turnCount        = 0;       // 已完成的“真弯”次数
bool     g_cornerCounts     = false;   // 本次左弯转成功后要不要计数（假弯/重转不计数）
unsigned long g_cornerStartMs      = 0; // 本次左弯开始时刻
unsigned long g_cornerCenteredMs   = 0; // 本次左弯里最近一次“骑正在线上”的时刻，0 = 还没有
unsigned long g_cornerCooldownUntilMs = 0; // 左弯冷却截止时刻
unsigned long g_cornerRetryUntilMs = 0;    // 同一个弯的重转窗口截止时刻
uint8_t  g_cornerRetryCnt   = 0;       // 这个弯已经“重转”了几次（超过 CORNER_MAX_RETRIES 就放手）
bool     g_cornerSawLine    = false;   // 本次左弯里是否扫到过线（老装法的完成判据用）
uint8_t  g_cornerLostCnt    = 0;       // “两路全白且线不在右侧”连续采样计数
uint8_t  g_cornerAbsent8    = 0;       // 起转那一刻“线已经白了多久”(ms/4)：判真弯用（见 finishCorner）
bool     g_cornerPre        = false;   // 是否在“预判左弯”段（减速 + 轻左带）
unsigned long g_cornerPreCooldownUntilMs = 0; // 预判冷却截止时刻

// —— 终点（只有“全白即停”，没有状态机）——
bool          g_finishWatch        = false; // 第 4 弯已完成，正在“等重新贴稳线 -> 认终点”
bool          g_finishArmed        = false; // 已经贴稳过：此后全白 FINISH_WHITE_MS 就是终点
unsigned long g_finishWhiteSinceMs = 0;     // 全白从什么时候开始（0 = 现在有线）
unsigned long g_finishLineSinceMs  = 0;     // 贴稳判定用：线连续看到的起始时刻（0 = 当前没线）
bool          g_braking            = false; // 正在反脉冲刹车（刹完就停车）
unsigned long g_brakeUntilMs       = 0;     // 反脉冲刹到什么时候

/* ========================== ④ 电机原语 ========================== */

void setMotor(uint8_t pinDir, uint8_t pinPwm, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (abs(speed) < MIN_SPEED) {      // 低于最小有效速度：直接给 0，避免堵转发热
    analogWrite(pinPwm, 0);
    return;
  }

  digitalWrite(pinDir, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_BACKWARD_LEVEL);
  analogWrite(pinPwm, abs(speed));
}

// 左右轮一起给（参数顺序就是“左, 右”，不要按内外轮记）
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (LEFT_MOTOR_INVERT)  leftSpeed  = -leftSpeed;
  if (RIGHT_MOTOR_INVERT) rightSpeed = -rightSpeed;

  setMotor(LEFT_MOTOR_DIR,  LEFT_MOTOR_PWM,  leftSpeed);
  setMotor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM, rightSpeed);
}

// 只停车，不改停车原因（上电初始化也用它）
void carStop() { g_phase = PHASE_IDLE; setMotorSpeeds(0, 0); }

// 停车并记下原因（code 见 StopCode）：之后 D13 按 code 闪灯，数闪几下就知道为什么停
void carStop(uint8_t code) {
  carStop();
  g_stopCode = code;
}

// 停车后的闪灯：闪 code 下 -> 停 1.2 秒 -> 再重复（STATUS_LED_PIN = 255 时不做任何事）
void updateStopBlink() {
#if STATUS_LED_PIN != 255
  if (g_stopCode == STOP_NONE) return;

  static unsigned long nextMs = 0;
  static uint8_t step = 0;                       // 一组 = 亮/灭交替，共 code 次“亮”
  unsigned long now = millis();
  if (now < nextMs) return;

  if (step >= (uint8_t)(g_stopCode * 2)) {       // 一组闪完，长停一下再重复，便于数
    digitalWrite(STATUS_LED_PIN, LOW);
    step   = 0;
    nextMs = now + 1200;
    return;
  }

  bool ledOn = (step % 2) == 0;                  // 先亮一下好数
  digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
  step++;
  nextMs = now + (ledOn ? 120 : 200);
#endif
}

/* ========================== ⑤ 传感器、偏差与“线在哪一侧” ========================== */

// 读两路传感器（0 = 左，1 = 右），返回检测到黑线的路数（0 / 1 / 2）
int readLineSensors() {
  g_black[0] = (digitalRead(SENSOR_LEFT_PIN)  == BLACK_LEVEL);
  g_black[1] = (digitalRead(SENSOR_RIGHT_PIN) == BLACK_LEVEL);

  int onLine = 0;
  for (uint8_t i = 0; i < 2; i++) {
    if (g_black[i]) onLine++;
  }
  return onLine;
}

// 维护“线最后在哪一侧”和它持续了多久 —— 左弯两条判据、丢线找回都靠它。
//   -1 = 只有左路压线；+1 = 只有右路压线；0 = 居中或全白
// ★ 全白时保持原值：这时唯一能用的信息就是“上次线在哪边”，保持还能防止计时被打断
void updateLineSide(unsigned long now, int onLine) {
  int8_t side = g_lastSide;                    // 全白 / 两路都压：先保持原值

  if (onLine == 1)          side = g_black[0] ? -1 : +1;
  else if (onLine == 2)     side = 0;          // 骑正在线上（居中）
  else if (!WHITE_MEANS_LOST) side = 0;        // 老装法：全白 = 居中

  if (side != g_lastSide) {
    g_lastSide    = side;
    g_sideSinceMs = now;
  }
}

// 偏差：只有左路压线 = -1、只有右路压线 = +1、其余 = 0（全白见 WHITE_MEANS_LOST）
// 只有单路压线时才刷新方向记忆：0（居中/全白）不覆盖它，否则方向会被抹掉
float computeError(int onLine) {
  if (onLine == 1) {
    float error = g_black[0] ? -1.0 : +1.0;
    g_lastDeflectDir = (error < 0.0) ? -1.0 : +1.0;
    return error;
  }
  if (onLine == 0) return WHITE_MEANS_LOST ? g_lastError : 0.0;
  return 0.0;
}

// 车头是否“正骑在线上”（左弯的出弯判据、上线补偿的提前结束都用它）
bool centeredNow(int onLine) {
  if (PROBES_CENTERED_BOTH_ON_LINE) return (onLine == 2);       // 两路都压线
  return (onLine == 0) && g_cornerSawLine;                      // 老装法：两路都不压线
}

/* ========================== ⑥ 巡线控制（PD）与丢线找回 ========================== */

// PD：偏差只有三档，修正量 = KP*偏差 + KD*偏差变化，限幅后叠加到左右轮
//（correction > 0 -> 左轮快右轮慢 -> 车往右拐，也就是朝“线在右边”的方向修）
void pdControl(int onLine) {
  float error      = computeError(onLine);
  float derivative = error - g_lastError;
  g_lastError      = error;

  float correction = KP * error + KD * derivative;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  setMotorSpeeds((int)(BASE_SPEED + correction), (int)(BASE_SPEED - correction));
}

// 找回力度：从 WHITE_SEARCH_BIAS 按平方曲线加到 WHITE_SEARCH_MAX_BIAS（越找不到压得越狠）
int whiteSearchBias(unsigned long elapsedMs) {
  if (elapsedMs >= WHITE_SEARCH_RAMP_MS) return WHITE_SEARCH_MAX_BIAS;

  long span = (long)WHITE_SEARCH_MAX_BIAS - (long)WHITE_SEARCH_BIAS;
  return WHITE_SEARCH_BIAS +
         (int)(span * (long)elapsedMs * (long)elapsedMs /
               ((long)WHITE_SEARCH_RAMP_MS * (long)WHITE_SEARCH_RAMP_MS));
}

// 两路全白 = 丢线：朝方向记忆压回去，力度随时间加大，直到又扫到黑线为止。
// ★ 内轮有 MIN_SPEED 地板、不许停转：内轮压到 0 就成了原地打转，线落在圈外永远扫不到
void whiteSearch(unsigned long now) {
  if (g_lostSinceMs == 0) g_lostSinceMs = now;      // 记录这次丢线的起点（力度递增从这儿算）

  int leftSpeed, rightSpeed;
  if (g_lastDeflectDir == 0.0) {                    // 本次运行还没偏转过：没有方向可依，直着找
    leftSpeed = rightSpeed = WHITE_SEARCH_SPEED;
  } else {
    int bias = whiteSearchBias(now - g_lostSinceMs);
    if (g_lastDeflectDir < 0.0) bias = -bias;       // 线最后在左 -> 往左压（右轮快），反之往右压
    leftSpeed  = constrain(WHITE_SEARCH_SPEED + bias, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(WHITE_SEARCH_SPEED - bias, MIN_SPEED, MAX_SPEED);
  }
  setMotorSpeeds(leftSpeed, rightSpeed);
}

/* ========================== ⑦ 上线补偿（刚并线那 300ms） ========================== */

// 补偿期内线滑出探头：朝最后看到线的一侧划弧扫回来，不算丢线（这是右转那套轮速）
void mergeSettleArc() {
  float dir = (g_settleDir != 0.0) ? g_settleDir : +1.0;   // 没记忆就按并入前的右转方向继续划
  if (dir < 0.0) setMotorSpeeds(START_TURN_INNER_SPEED, START_TURN_SPEED);
  else           setMotorSpeeds(START_TURN_SPEED, START_TURN_INNER_SPEED);
}

// 刚上线时车头是斜的，直接交给 PD 会第一拍猛修、一摆线就滑出去。补偿期里：
//   限速 + PD 修正打折 + 按“线最后在哪一侧”固定带一把（MERGE_BIAS），骑正了立刻结束
void mergeSettle(int onLine) {
  if (centeredNow(onLine)) {                 // 已经骑正在线上：补偿到此为止，交回 PD
    g_settleUntilMs = 0;
    pdControl(onLine);
    return;
  }

  if (WHITE_MEANS_LOST && onLine == 0) {     // 期内又滑出探头：划弧扫回来（不算丢线）
    mergeSettleArc();
    return;
  }

  float error      = computeError(onLine);
  float derivative = error - g_lastError;
  g_lastError      = error;

  float correction = MERGE_SETTLE_GAIN * (KP * error + KD * derivative);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  correction += MERGE_BIAS * g_settleDir;

  setMotorSpeeds((int)(MERGE_SETTLE_SPEED + correction),
                 (int)(MERGE_SETTLE_SPEED - correction));
}

/* ========================== ⑧ 左弯：起转判据 + 三拍动作 + 计数 ========================== */

// 起转：两条判据谁先满足用谁 —— ① 只左路压线够久（★默认关，直道摆动会误判成弯）
//                              ② 线从左侧/正下方消失（主判据）：两路全白够 CORNER_LOST_CONFIRM_MS
//                                 且最后看到线时不在右侧（骑在线上冲过弯变全白时最后是居中）
// ★确认窗口里主循环走 whiteSearch()，车已在朝线消失那侧划弧，窗口结束才切硬拧三拍；
//   窗口内线回来了就放回 PD —— 整套动作可逆，直道误判不会变成原地左拧
bool checkCornerStart(unsigned long now, int onLine) {
  if (g_turnCount >= LEFT_TURN_TARGET) {              // 4 个弯都数满了
    // ★但“不许再转弯”不成立：出弯时车头往往还差 20~40°，前三个弯就是靠“重转”补的；
    //   最后一个弯一律不许再进的话，车会没转完就直着冲出去（实测踩过）。放行条件：
    //     ① 还在重转窗口内（= 刚才那个弯没转干净）  ② 终点还没开始认线（!g_finishArmed）
    if (!(now < g_cornerRetryUntilMs && !g_finishArmed)) {
      g_cornerLostCnt = 0;
      return false;
    }
  }
  if (now < g_cornerCooldownUntilMs ||                // 冷却期：刚出弯，别马上又判定
      (g_settleUntilMs != 0 && now < g_settleUntilMs)) {  // 上线补偿期：别把找线动作记成弯
    g_cornerLostCnt = 0;
    return false;
  }

#if CORNER_LEFT_HOLD_ENABLE
  bool leftHoldTooLong = (onLine == 1) && (g_lastSide < 0) &&
                         (now - g_sideSinceMs >= CORNER_STUCK_LEFT_MS);
#else
  bool leftHoldTooLong = false;                       // 判据①默认关：直道误判的根源
#endif

  bool lostToLeft = (onLine == 0) && WHITE_MEANS_LOST && (g_lastSide <= 0);
  // “线已经白了多久”：从最后一次任一路看到线算起（updateLineSeen() 每周期维护）
  unsigned long absentMs = (lostToLeft && g_lastLineSeenMs != 0)
                             ? (now - g_lastLineSeenMs) : 0;
  g_cornerLostCnt = lostToLeft ? (uint8_t)(g_cornerLostCnt + 1) : 0;
  bool lostConfirmed = lostToLeft &&
                       (absentMs >= CORNER_LOST_CONFIRM_MS) &&
                       (g_cornerLostCnt >= CORNER_LOST_SAMPLES);

  if (leftHoldTooLong || lostConfirmed) {
    g_cornerState   = CORNER_ARC;                    // 从第一拍“弧线切入”开始
    g_cornerCounts  = (now >= g_cornerRetryUntilMs); // 重转窗口内 = 同一个弯，照转但不计数
    g_cornerStartMs = now;
    g_cornerCenteredMs = 0;
    g_cornerSawLine = false;
    g_cornerLostCnt = 0;
    g_cornerPre = false;                             // 已经真起转了，预判段结束
    g_cornerAbsent8 = (uint8_t)(absentMs / 4 > 255 ? 255 : absentMs / 4);
    return true;
  }
  return false;
}

// 预判左弯：只左路压线够 CORNER_PRE_MS 就先“慢速 + 轻左带”，把车带进随时能起转的姿态。
// 真起转不在这里决定；线回到中间/跑到右侧就立刻退出。丢线也不归这里（先交给 whiteSearch）
void updateCornerPre(unsigned long now, int onLine) {
  bool leftHold = (onLine == 1) && (g_lastSide < 0);   // 线只挂在左探头上

  if (g_cornerPre) {
    if (!leftHold) {                                   // 假弯：线回来了 -> 退出预判
      g_cornerPre = false;
      g_cornerPreCooldownUntilMs = now + CORNER_PRE_COOLDOWN_MS;
      return;                                          // 本周期不控制，下周期回正常 PD
    }
  } else {
#if CORNER_PRE_ENABLE
    if (g_turnCount >= LEFT_TURN_TARGET) return;                     // 弯都跑完了，不再预判
    if (now < g_cornerCooldownUntilMs || now < g_cornerPreCooldownUntilMs) return;
    if (g_settleUntilMs != 0 && now < g_settleUntilMs) return;        // 上线补偿期不掺和
    if (!(leftHold && (now - g_sideSinceMs >= CORNER_PRE_MS))) return;
    g_cornerPre = true;                                              // 进预判
#else
    return;
#endif
  }

  setMotorSpeeds(CORNER_PRE_INNER, CORNER_PRE_OUTER);
}

// 转完一次左弯：判真弯（要不要计数）+ 设冷却/重转窗口 + 让 PD 平稳接管
void finishCorner(unsigned long now, unsigned long elapsed) {
  // ★“真弯”判据 = 线**总共**白了多久（起转前 + 转弯中）够长，不是“转弯用了多久”：
  //   出弯判据放宽后真弯也可能只转 150ms，假弯总共只白几十毫秒，按总时长分辨才不会漏计
  bool realCorner = g_cornerCounts &&
                    (elapsed >= CORNER_MIN_TURN_MS) &&
                    (((unsigned long)g_cornerAbsent8 * 4 + elapsed) >= CORNER_REAL_MIN_MS);
  g_cornerState = CORNER_NONE;

  if (realCorner) {
    g_turnCount++;
    g_cornerRetryCnt = 0;                          // 真弯转成了：重转计数清零
    if (g_turnCount >= LEFT_TURN_TARGET) {          // 第 4 弯完成：和前面几个弯一样继续巡线，
                                                    // 只是挂上“全白即停”的监视（见 ⑩）
      g_finishWatch        = true;
      g_finishArmed        = false;
      g_finishLineSinceMs  = 0;
      g_finishWhiteSinceMs = 0;
    }
  } else {
    g_cornerRetryCnt++;                             // 这次是重转（没计数的那个）
  }

  g_cornerCooldownUntilMs = now + CORNER_COOLDOWN_MS;
  // ★重转次数用完：重转窗口一并作废，这个弯不再拧，车头没对准也交给 PD / 丢线找回。
  //   否则“没转干净 -> 又丢线 -> 又进弯”会连着来，越拧越偏，最后顺着来时那根线反着走
  if (g_cornerRetryCnt > CORNER_MAX_RETRIES) {
    g_cornerCooldownUntilMs = now + CORNER_RETRY_MS;
  }
  g_cornerRetryUntilMs    = now + CORNER_RETRY_MS;
  g_lastError             = 0.0;
  g_lostSinceMs           = 0;      // 丢线计时清零：找回力度从温和档重新往上加
  // 方向记忆 = 此刻车头的偏斜方向：单路压线就记住线在哪一侧，两路都压（对准了）就清掉。
  // ★全白时保持原值、千万别清：这是丢线找回唯一能用的信息，清成 0 车就直着冲出去
  if (g_black[0] != g_black[1])   g_lastDeflectDir = g_black[0] ? -1.0 : +1.0;
  else if (g_black[0])            g_lastDeflectDir = 0.0;
}

// 左弯动作：三拍，每拍一个明确状态
//   拍 1 ARC   ：两轮都在滚（内 CORNER_ARC_INNER / 外 CORNER_OUTER_SPEED），平顺切入不打滑
//   拍 2 PIVOT ：内轮按 CORNER_PIVOT_INNER 收（默认 -90 = 倒转），旋转中心移进车体内侧硬拧
//   拍 3 SEARCH：边前进边左转，一直扫到线为止；车头只朝左转、绝不反向
//   ★第三拍内轮保持正转、别改负的：它要“往前走一点、扩大扫描范围”，把落在拧的圈外的线
//     找回来；内轮一反转就变成原地打转，线在圈外时转多少圈都扫不到
void updateCorner(unsigned long now, int onLine) {
  unsigned long elapsed = now - g_cornerStartMs;

  if (g_cornerState == CORNER_ARC && elapsed >= CORNER_ARC_MS)     g_cornerState = CORNER_PIVOT;
  if (g_cornerState != CORNER_SEARCH && elapsed >= CORNER_TIMEOUT_MS) g_cornerState = CORNER_SEARCH;

  switch (g_cornerState) {
    case CORNER_ARC:    setMotorSpeeds(CORNER_ARC_INNER, CORNER_OUTER_SPEED);    break;
    case CORNER_PIVOT:  setMotorSpeeds(CORNER_PIVOT_INNER, CORNER_OUTER_SPEED);  break;
    default:            setMotorSpeeds(CORNER_SEARCH_INNER, CORNER_SEARCH_OUTER); break;
  }

  // ★完成判据：任一个探头扫到线就算转完，立刻交回 PD。敢这么宽是因为起转时线已白够
  //   CORNER_LOST_CONFIRM_MS，探头是干净的，扫到的只可能是新线（来时那根在车身后方，
  //   要转过头 200° 以上才够得着，这里最多拧 500ms ≈ 90°）。剩下的偏斜交给 PD
  if (onLine > 0) g_cornerSawLine = true;
  if (centeredNow(onLine)) g_cornerCenteredMs = now;

  bool centeredRecently = (g_cornerCenteredMs != 0) &&
                          (now - g_cornerCenteredMs <= CORNER_EXIT_GRACE_MS);
  bool foundLine = centeredRecently || (onLine > 0);

  if (foundLine && elapsed >= CORNER_MIN_TURN_MS) finishCorner(now, elapsed);
}

/* ========================== ⑨ 进场：直行 -> 右转并线 ========================== */

// 开始本次运行：清空所有状态量，进入直行段
void startRun() {
  g_turnCount      = 0;
  g_linesPassed    = 0;
  g_straightOnLine = false;
  g_turnGateOpen   = false;

  g_black[0] = g_black[1] = false;
  g_lastSide = 0;
  g_lastError = 0.0;
  g_lastDeflectDir = 0.0;

  g_lostSinceMs    = 0;
  g_lastLineSeenMs = 0;
  g_settleUntilMs  = 0;
  g_settleDir      = 0.0;

  g_cornerState  = CORNER_NONE;
  g_cornerCounts = false;
  g_cornerStartMs = 0;
  g_cornerCenteredMs = 0;
  g_cornerCooldownUntilMs = 0;
  g_cornerRetryUntilMs = 0;
  g_cornerRetryCnt = 0;
  g_cornerSawLine = false;
  g_cornerLostCnt = 0;
  g_cornerAbsent8 = 0;
  g_cornerPre     = false;
  g_cornerPreCooldownUntilMs = 0;

  g_finishWatch    = false;
  g_finishArmed    = false;
  g_finishWhiteSinceMs = 0;
  g_finishLineSinceMs  = 0;
  g_braking        = false;
  g_brakeUntilMs   = 0;
  g_stopCode       = STOP_NONE;

  g_phase        = PHASE_STRAIGHT;
  g_phaseStartMs = millis();
  g_runStartMs   = g_phaseStartMs;
  g_sideSinceMs  = g_phaseStartMs;
  g_lastSampleMs = g_phaseStartMs;

  setMotorSpeeds(START_STRAIGHT_SPEED, START_STRAIGHT_SPEED);
}

// 切到右转并线
void enterTurnRight(unsigned long now, int onLine) {
  g_phase        = PHASE_TURN_RIGHT;
  g_phaseStartMs = now;
  // 找线闸门：这一刻若探头还压着线（提前转的情况必然如此，正要右转上这根线），
  // 先等它离开探头，之后再扫到的才算“并上了边线”，免得一转就宣布成功、车头还是斜的
  g_turnGateOpen = (onLine == 0);
  setMotorSpeeds(START_TURN_SPEED, START_TURN_INNER_SPEED);
}

// 直行段：不做任何转向修正（纯开环），只数线 —— 压线->离开才算完整跨过一根
//   忽略掉 START_IGNORE_LINES 根后，再见到的就是目标边线 -> 提前右转；等不到走满计时兜底
void doStraight(unsigned long now, int onLine) {
  setMotorSpeeds(START_STRAIGHT_SPEED, START_STRAIGHT_SPEED);

  bool anyBlack = (onLine > 0);
  if (anyBlack) {
    if (g_linesPassed >= START_IGNORE_LINES) {   // 这就是第 2 根线（目标边线）
      enterTurnRight(now, onLine);               // 提前转（正式场地那段灰口子看不见，实际走计时）
      return;
    }
    g_straightOnLine = true;                     // 还压着“忽略段”的那根线
  } else if (g_straightOnLine) {
    g_straightOnLine = false;
    g_linesPassed++;                             // 完整跨过一根
  }

  if (now - g_phaseStartMs >= START_STRAIGHT_MS) enterTurnRight(now, onLine);  // 计时兜底
}

// 并线成功：切巡线，设上线补偿期和左弯冷却
void enterLineFollow(unsigned long now, int onLine) {
  g_phase       = PHASE_LINE;
  g_lastSide    = (onLine == 1) ? (g_black[0] ? -1 : +1) : 0;
  g_sideSinceMs = now;
  g_lastError   = computeError(onLine);      // 顺手刷新方向记忆
  g_lostSinceMs = 0;
  g_lastLineSeenMs = now;                    // 线消失兜底从“并进来的这一刻”开始算

  g_settleUntilMs = now + MERGE_SETTLE_MS;   // 上线补偿：只对“线在某一侧”记方向
  g_settleDir     = (onLine == 1) ? (g_black[0] ? -1.0 : +1.0) : 0.0;

  g_cornerCooldownUntilMs = now + CORNER_COOLDOWN_MS;   // 刚并进来，稍后再检测左弯
  g_cornerLostCnt = 0;
}

// 右转段：绕右轮划小弧；闸门打开后任一路扫到黑线即并入；找不到就一直转（不停车）
void doTurnRight(unsigned long now, int onLine) {
  setMotorSpeeds(START_TURN_SPEED, START_TURN_INNER_SPEED);

  bool anyBlack = (onLine > 0);
  if (!g_turnGateOpen) {
    if (!anyBlack) g_turnGateOpen = true;    // 旧线已离开探头，从现在起认新线
    return;
  }
  if (anyBlack) enterLineFollow(now, onLine);
  // 没扫到就一直这么转下去 —— 没有超时停车，只有 RUN_SAFETY_TIMEOUT_MS 总保护能停
}

/* ========================== ⑩ 终点 ========================== */

// 线消失兜底：巡线阶段任何一路连续 LINE_END_TIMEOUT_MS 都扫不到黑线 -> 认为线走完了，停车。
// 这条不看弯数，是“弯数没数准 / 在弯里把线彻底丢了”时的最后保险
void updateLineSeen(unsigned long now, int onLine) {
  if (onLine > 0) {                          // 有任一路压线：刷新时刻
    g_lastLineSeenMs = now;
    return;
  }
  if (g_lastLineSeenMs != 0 &&
      (now - g_lastLineSeenMs > LINE_END_TIMEOUT_MS)) {
    carStop(STOP_FINISH);
  }
}

// ★终点监视：第 4 弯之后不下发任何特殊控制（车就是正常巡线，代码里没有“终点段”状态），
//   只干两件事：① 线连续看到 FINISH_ARM_MS 才算“重新贴稳” —— 刚出弯那一下车头还歪、
//   可能短暂全白，绝不算终点，否则会在弯里就刹停；② 贴稳后全白持续 FINISH_WHITE_MS
//   = 真到线头 -> 反脉冲刹车。返回 true = 本周期已接管，主循环别再往下走
bool finishWatchUpdate(unsigned long now, int onLine) {
  if (!g_finishWatch) return false;

  if (onLine > 0) {                       // 有线：交回正常 PD
    if (g_finishLineSinceMs == 0) g_finishLineSinceMs = now;
    if (!g_finishArmed && (now - g_finishLineSinceMs >= FINISH_ARM_MS)) {
      g_finishArmed = true;               // 贴稳了：从现在起，全白才算终点
    }
    g_finishWhiteSinceMs = 0;
    return false;
  }

  // 全白
  g_finishLineSinceMs = 0;
  if (!g_finishArmed) return false;       // 还没贴稳过：当普通丢线处理（whiteSearch，和别的弯一样）

  if (g_finishWhiteSinceMs == 0) g_finishWhiteSinceMs = now;
  if (now - g_finishWhiteSinceMs >= FINISH_WHITE_MS) {
    g_braking      = true;                // 白够久了 = 真到线头：反脉冲刹车，刹完停车（D13 闪 2 下）
    g_brakeUntilMs = now;
    g_finishWatch  = false;
    return true;
  }
  // 还白着：朝“线最后在哪一侧”轻带一把
  //   ★不死直着走：车自己摆出去也会全白，死直着走白够时间就误停，带一把线就扫回来了；
  //     线真到头（灰段）时这把力扫不到东西、力度也小，车照样停在灰段里
  int dir  = (g_lastDeflectDir < 0.0) ? -1 : ((g_lastDeflectDir > 0.0) ? 1 : 0);
  int bias = FINISH_RECOVER_BIAS * dir;
  setMotorSpeeds(WHITE_SEARCH_SPEED + bias, WHITE_SEARCH_SPEED - bias);
  return true;
}

/* ========================== ⑪ 初始化与主循环 ========================== */

// 上电：电机先停住 -> 等 START_DELAY_MS -> 起步
void setup() {
  pinMode(SENSOR_LEFT_PIN, INPUT);
  pinMode(SENSOR_RIGHT_PIN, INPUT);

  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  carStop();                              // 先确保电机静止，避免上电瞬间乱跑
#if STATUS_LED_PIN != 255
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
#endif

#if LIGHT_START_ENABLED
  // 光信号启动：先校准环境光（这 800ms 也正好留出放手时间），再等手电信号
  pinMode(LIGHT_PIN, INPUT);
#if LIGHT_DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("=== pd_light_start：光信号启动模式 ==="));
#endif
  calibrateAmbient();
  waitForLightStart();
#if STATUS_LED_PIN != 255
  digitalWrite(STATUS_LED_PIN, LOW);      // 起步后灭灯，后面停车闪灯接管
#endif
#else
  // 调试模式：沿用原来的自动延时起步
  delay(START_DELAY_MS);
#endif

  startRun();
}

// 主循环：每周期一次“采样 -> 分派 -> 控制”
void loop() {
  if (g_phase == PHASE_IDLE) {            // 停下来了：闪 D13 报停车原因
    updateStopBlink();
    return;
  }

  unsigned long now = millis();

  /* ---- 终点刹车：反脉冲 FINISH_BRAKE_MS -> 停车 ---- */
  if (g_braking) {
    if (now - g_brakeUntilMs >= FINISH_BRAKE_MS) { g_braking = false; carStop(STOP_FINISH); }
    else { setMotorSpeeds(-FINISH_BRAKE_SPEED, -FINISH_BRAKE_SPEED); }
    return;
  }

  // 进场段（直行 / 右转）每个 loop 都采样：车扫过线只有一两毫秒，按控制周期读会漏掉；
  // 巡线段按固定的 SAMPLE_INTERVAL_MS 控制周期走（固定周期 Kd 才有意义）
  if (g_phase != PHASE_STRAIGHT && g_phase != PHASE_TURN_RIGHT) {
    if (now - g_lastSampleMs < SAMPLE_INTERVAL_MS) return;
    g_lastSampleMs = now;
  }

  // 总时间保护：所有“永不停车”的兜底
  if (RUN_SAFETY_TIMEOUT_MS > 0 && now - g_runStartMs > RUN_SAFETY_TIMEOUT_MS) {
    carStop(STOP_SAFETY);
    return;
  }

  int onLine = readLineSensors();
  updateLineSide(now, onLine);

  /* ---- 进场段 ---- */
  if (g_phase == PHASE_STRAIGHT)   { doStraight(now, onLine);   return; }
  if (g_phase == PHASE_TURN_RIGHT) { doTurnRight(now, onLine);  return; }

  /* ---- 线消失兜底（不看弯数） ---- */
  updateLineSeen(now, onLine);
  if (g_phase == PHASE_IDLE) return;      // 上面可能已经停车

  /* ---- 左弯 ---- */
  if (g_cornerState != CORNER_NONE) { updateCorner(now, onLine); return; }
  if (checkCornerStart(now, onLine)) { updateCorner(now, onLine); return; }
  updateCornerPre(now, onLine);           // 预判左弯：必要时先减速 + 轻左带（减小过冲）
  if (g_cornerPre) return;                // 预判期已自己控制轮速

  /* ---- 终点监视：先等贴稳线，之后全白 = 终点 ---- */
  if (finishWatchUpdate(now, onLine)) return;

  /* ---- 上线补偿期 ---- */
  if (g_settleUntilMs != 0 && now < g_settleUntilMs) { mergeSettle(onLine); return; }

  /* ---- 丢线：按方向记忆一直找回，直到又扫到黑线 ---- */
  if (WHITE_MEANS_LOST && onLine == 0) { whiteSearch(now); return; }
  g_lostSinceMs = 0;                      // 又看到线了：清掉丢线计时，交回 PD

  /* ---- PD 巡线 ---- */
  pdControl(onLine);
}
