/*
 * ============================================================================
 *  motor_test.ino —— 全车电机（四个轮子）测试程序
 * ----------------------------------------------------------------------------
 *  硬件平台：Arduino UNO R3
 *            + Keyes L298P 电机驱动扩展板（KE0163）
 *            + 4 个直流减速电机（左右各两个）
 *
 *  【接线表】（默认值，可在“② 引脚定义”中修改）
 *  ---------------------------------------------------------------------------
 *   L298P 扩展板 A 组（右侧两轮）：方向 D2，调速/PWM D6
 *                                  M3 -> A1，M4 -> A2
 *   L298P 扩展板 B 组（左侧两轮）：方向 D4，调速/PWM D5
 *                                  M1 -> B2，M2 -> B1
 *  ---------------------------------------------------------------------------
 *   说明：L298P 只有 2 个驱动通道，同一通道上的两个电机是并联的，
 *         所以 M1 与 M2 只能同步转动、M3 与 M4 只能同步转动，
 *         测试时按“组”（通道）验证，而不是单独验证某一颗电机。
 *
 *  【安全提示】
 *    1. 接线、拔插电机前请断开电源（拔掉 USB 线或把扩展板拨码开关拨到 OFF）。
 *    2. 电池正负极切勿接反，否则可能烧毁电机驱动扩展板。
 *    3. 上传程序前请拔掉蓝牙模块（蓝牙占用 D0/D1 串口，会导致下载失败）。
 *    4. 测试前请把小车架空，让四个轮子悬空，防止小车突然冲出去。
 *    5. 电机“嗡嗡”响却不转属于堵转，会迅速发热，请立即按 s 停车检查。
 *
 *  【使用方法】
 *    1. 用支架或盒子把小车架起来，确认四轮离地；
 *    2. 打开串口监视器，波特率选择 9600；
 *    3. 发送下面的单个字符命令：
 *         1 / 2 ：A 组（右侧 M3、M4）正转 / 反转
 *         3 / 4 ：B 组（左侧 M1、M2）正转 / 反转
 *         5 / 6 ：左右同时正转（小车前进）/ 反转（小车后退）
 *         7 / 8 ：原地左转 / 原地右转
 *         9     ：PWM 速度爬坡测试（两组同步 60 -> 220 -> 60）
 *         a     ：自动全流程测试（依次测完全部电机，无需手动干预）
 *         s     ：立即停车（测试过程中也随时有效）
 *         + / - ：测试速度加 / 减 10
 *         m     ：重新打印命令菜单
 *    4. 判定标准：正转时该侧两个轮子都朝车头方向转，反转时都朝车尾方向转；
 *       若某个电机转向相反，把“① 可调参数区”中对应的 INVERT 开关改成 1。
 * ============================================================================
 */

/* ========================== ① 可调参数区 ========================== */

// —— 电机安装方向补偿 ——
// 若某个电机实际转向与程序相反（正转变成后退），把对应的值改为 1 即可。
#define LEFT_MOTOR_INVERT  0
#define RIGHT_MOTOR_INVERT 0

// —— 速度参数（PWM 取值 0 ~ 255）——
#define TEST_SPEED       150    // 默认测试速度，可用 + / - 在线调整
#define MIN_SPEED        60     // 最小有效速度：低于此值电机通常转不动，直接给 0 避免堵转发烫
#define MAX_SPEED        220    // 单轮最大速度限幅

// —— 时间参数 ——
#define STEP_RUN_MS      1500   // 单个测试步骤的持续时间(ms)
#define STEP_PAUSE_MS    600    // 两步之间的停顿时间(ms)，方便观察轮子是否停稳
#define START_DELAY_MS   2000   // 上电后等待时间(ms)，留出确认接线、看清串口的时间

// —— 爬坡测试参数 ——
#define RAMP_MIN         60     // 爬坡起始速度
#define RAMP_MAX         220    // 爬坡最高速度
#define RAMP_STEP        5      // 每一步调整的速度值
#define RAMP_INTERVAL_MS 120    // 每一步的持续时间(ms)

/* ========================== ② 引脚定义 ========================== */

// L298P 扩展板：A 组为右侧电机（M3、M4），B 组为左侧电机（M1、M2）
const uint8_t RIGHT_MOTOR_DIR = 2;   // A 组方向控制脚（IN）
const uint8_t RIGHT_MOTOR_PWM = 6;   // A 组速度控制脚（EN）
const uint8_t LEFT_MOTOR_DIR  = 4;   // B 组方向控制脚（IN）
const uint8_t LEFT_MOTOR_PWM  = 5;   // B 组速度控制脚（EN）

// 电机正转/反转对应的方向脚电平（本扩展板前进时两只方向脚均为 HIGH）
const uint8_t MOTOR_FORWARD_LEVEL  = HIGH;
const uint8_t MOTOR_BACKWARD_LEVEL = LOW;

/* ========================== ③ 全局变量 ========================== */

int g_speed = TEST_SPEED;   // 当前测试速度，可用 + / - 调整

/* ========================== ④ 电机原语（与 PD.ino 一致） ========================== */

/*
 * 设置单个电机的转速与方向
 * speed：-255 ~ +255，正值正转、负值反转、0 停止
 */
void setMotor(uint8_t pinDir, uint8_t pinPwm, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (abs(speed) < MIN_SPEED) {          // 速度太小电机转不动，直接关掉，避免堵转发烫
    analogWrite(pinPwm, 0);
    return;
  }

  digitalWrite(pinDir, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_BACKWARD_LEVEL);
  analogWrite(pinPwm, abs(speed));
}

/*
 * 设置左右两侧轮速
 * leftSpeed / rightSpeed：-255 ~ +255，同一侧的两个电机同步转动
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (LEFT_MOTOR_INVERT)  leftSpeed  = -leftSpeed;
  if (RIGHT_MOTOR_INVERT) rightSpeed = -rightSpeed;

  setMotor(LEFT_MOTOR_DIR,  LEFT_MOTOR_PWM,  leftSpeed);
  setMotor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM, rightSpeed);
}

void carStop() { setMotorSpeeds(0, 0); }   // 停止

/* ========================== ⑤ 测试辅助函数 ========================== */

/*
 * 保持当前电机状态 ms 毫秒
 * 期间只响应 's' / 'S'：立即停车并返回 false；其它按键一律忽略，避免测试中误触发
 */
bool holdMotorState(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        carStop();
        Serial.println(F(">> 收到 s，已停车"));
        return false;
      }
    }
  }
  return true;
}

/*
 * 执行一个测试步骤：先打印步骤说明和预期现象，再驱动电机 ms 毫秒后停车
 * title  ：步骤名称
 * expect ：预期现象，方便对照检查
 * leftSpeed / rightSpeed：本步骤的左右轮速度（带符号，0 表示该侧不动）
 * 返回 false 表示测试过程中被 s 命令中断
 */
bool runStep(const __FlashStringHelper* title, const __FlashStringHelper* expect,
             int leftSpeed, int rightSpeed, unsigned long ms) {
  Serial.println();
  Serial.print(F("---- ")); Serial.print(title); Serial.println(F(" ----"));
  Serial.print(F("     预期：")); Serial.println(expect);
  Serial.print(F("     左右轮速度 L=")); Serial.print(leftSpeed);
  Serial.print(F("  R=")); Serial.print(rightSpeed);
  Serial.print(F("，持续 ")); Serial.print(ms); Serial.println(F(" ms（随时可按 s 停车）"));

  setMotorSpeeds(leftSpeed, rightSpeed);
  if (!holdMotorState(ms)) return false;   // 中途被 s 打断

  carStop();
  delay(STEP_PAUSE_MS);                    // 停顿一下，便于确认轮子已停稳
  return true;
}

/* ========================== ⑥ 各测试项目 ========================== */

// —— 分组测试：验证一条通道上的两个电机 ——
bool testRightForward()  { return runStep(F("A 组（右侧 M3、M4）正转"), F("右前轮、右后轮都朝车头方向转"),
                                          0,   g_speed, STEP_RUN_MS); }
bool testRightBackward() { return runStep(F("A 组（右侧 M3、M4）反转"), F("右前轮、右后轮都朝车尾方向转"),
                                          0,  -g_speed, STEP_RUN_MS); }
bool testLeftForward()   { return runStep(F("B 组（左侧 M1、M2）正转"), F("左前轮、左后轮都朝车头方向转"),
                                          g_speed,  0, STEP_RUN_MS); }
bool testLeftBackward()  { return runStep(F("B 组（左侧 M1、M2）反转"), F("左前轮、左后轮都朝车尾方向转"),
                                         -g_speed,  0, STEP_RUN_MS); }

// —— 组合动作：验证四个电机是否同向配合 ——
bool testBothForward()   { return runStep(F("左右同时正转（小车前进）"), F("四个轮子都朝车头方向转"),
                                          g_speed,  g_speed, STEP_RUN_MS); }
bool testBothBackward()  { return runStep(F("左右同时反转（小车后退）"), F("四个轮子都朝车尾方向转"),
                                         -g_speed, -g_speed, STEP_RUN_MS); }
bool testSpinLeft()      { return runStep(F("原地左转（左轮反转、右轮正转）"), F("左侧轮子向后转、右侧轮子向前转，车头向左摆"),
                                         -g_speed,  g_speed, STEP_RUN_MS); }
bool testSpinRight()     { return runStep(F("原地右转（左轮正转、右轮反转）"), F("左侧轮子向前转、右侧轮子向后转，车头向右摆"),
                                          g_speed, -g_speed, STEP_RUN_MS); }

// —— PWM 速度爬坡：验证调速是否平滑、低速能否顺利启动 ——
bool testSpeedRamp() {
  Serial.println();
  Serial.println(F("---- PWM 速度爬坡测试（两组同步） ----"));
  Serial.print(F("     预期：速度从 ")); Serial.print(RAMP_MIN);
  Serial.print(F(" 升到 "));            Serial.print(RAMP_MAX);
  Serial.println(F(" 再降回来，轮子由慢到快、再由快到慢，中间不卡顿、不堵转"));

  bool ok = true;
  for (int s = RAMP_MIN; s <= RAMP_MAX && ok; s += RAMP_STEP) {
    setMotorSpeeds(s, s);
    Serial.print(F("     速度 = ")); Serial.println(s);
    ok = holdMotorState(RAMP_INTERVAL_MS);
  }
  for (int s = RAMP_MAX; s >= RAMP_MIN && ok; s -= RAMP_STEP) {
    setMotorSpeeds(s, s);
    Serial.print(F("     速度 = ")); Serial.println(s);
    ok = holdMotorState(RAMP_INTERVAL_MS);
  }

  carStop();
  return ok;
}

/*
 * 自动全流程测试：把上面所有项目依次跑一遍
 * 中途按 s 会立即停车并结束整个流程
 */
void autoTest() {
  Serial.println();
  Serial.println(F("===== 自动全流程测试开始（随时可按 s 停止） ====="));

  bool ok = true;
  ok = ok && testRightForward();
  ok = ok && testRightBackward();
  ok = ok && testLeftForward();
  ok = ok && testLeftBackward();
  ok = ok && testBothForward();
  ok = ok && testBothBackward();
  ok = ok && testSpinLeft();
  ok = ok && testSpinRight();
  ok = ok && testSpeedRamp();

  carStop();
  Serial.println();
  if (ok) {
    Serial.println(F("===== 自动全流程测试结束，检查要点 ====="));
    Serial.println(F(" 1) 4 个轮子都能转，没有不转、卡顿或异响的电机；"));
    Serial.println(F(" 2) 正转时 4 个轮子都朝车头方向转（若某一侧相反，改 INVERT 开关）；"));
    Serial.println(F(" 3) 前进/后退/原地左右转的方向与菜单描述一致；"));
    Serial.println(F(" 4) 爬坡时转速随速度值平滑变化，低速段不抖动、不堵转。"));
    Serial.println(F("========================================"));
  } else {
    Serial.println(F(">> 测试被 s 中断，电机会保持停止"));
  }
}

// —— 串口命令菜单 ——
void printMenu() {
  Serial.println();
  Serial.println(F("================= 全车电机测试 ================="));
  Serial.println(F(" 1 : A 组（右侧 M3、M4）正转"));
  Serial.println(F(" 2 : A 组（右侧 M3、M4）反转"));
  Serial.println(F(" 3 : B 组（左侧 M1、M2）正转"));
  Serial.println(F(" 4 : B 组（左侧 M1、M2）反转"));
  Serial.println(F(" 5 : 左右同时正转（小车前进）"));
  Serial.println(F(" 6 : 左右同时反转（小车后退）"));
  Serial.println(F(" 7 : 原地左转"));
  Serial.println(F(" 8 : 原地右转"));
  Serial.println(F(" 9 : PWM 速度爬坡测试"));
  Serial.println(F(" a : 自动全流程测试"));
  Serial.println(F(" s : 立即停车"));
  Serial.println(F(" + / - : 测试速度 ±10"));
  Serial.println(F(" m : 重新打印本菜单"));
  Serial.print(F("当前测试速度 = ")); Serial.println(g_speed);
  Serial.println(F("==============================================="));
}

/* ========================== ⑦ 初始化 ========================== */

void setup() {
  Serial.begin(9600);   // 与示例文档保持一致

  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);

  carStop();            // 先确保电机静止，避免上电瞬间乱跑

  Serial.println(F("=== 全车电机测试（UNO R3 + L298P） ==="));
  Serial.println(F("请先把小车架空、四轮悬空，再开始测试！"));
  printMenu();

  delay(START_DELAY_MS);   // 留出读提示、确认接线的时间
}

/* ========================== ⑧ 主循环：串口命令解析 ========================== */

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  switch (cmd) {
    // —— 单组测试：A 组 = 右侧（M3、M4），B 组 = 左侧（M1、M2）——
    case '1': testRightForward();  break;
    case '2': testRightBackward(); break;
    case '3': testLeftForward();   break;
    case '4': testLeftBackward();  break;

    // —— 组合动作 ——
    case '5': testBothForward();   break;
    case '6': testBothBackward();  break;
    case '7': testSpinLeft();      break;
    case '8': testSpinRight();     break;

    // —— 调速 / 爬坡 ——
    case '9': testSpeedRamp();     break;
    case '+':
      g_speed = min(g_speed + 10, MAX_SPEED);
      Serial.print(F(">> 当前测试速度 = ")); Serial.println(g_speed);
      break;
    case '-':
      g_speed = max(g_speed - 10, MIN_SPEED);
      Serial.print(F(">> 当前测试速度 = ")); Serial.println(g_speed);
      break;

    // —— 自动全流程 / 停车 / 菜单 ——
    case 'a': autoTest();          break;
    case 's': carStop(); Serial.println(F(">> 已停车")); break;
    case 'm': printMenu();         break;

    default: break;   // 忽略换行、空格等其它字符
  }
}

/* ============================================================================
 *  【常见问题】
 *   - 某个轮子反向：把对应的 LEFT_MOTOR_INVERT / RIGHT_MOTOR_INVERT 改成 1，
 *     或者把该电机的两根线在 A1/A2（B1/B2）端子上对调；
 *   - 电机不转或叫声大：多为供电不足，请使用 7~9V 电池而非仅靠 USB 供电；
 *   - 只有一个电机不转：先对调同一通道上的两个电机插头，确认是电机问题还是板子问题；
 *   - 四个轮子都不转：检查扩展板拨码开关是否打到 ON、电池是否装好；
 *   - 转速忽快忽慢：确认电池电量充足，避免与大功率外设共用一路电源。
 * ========================================================================== */
