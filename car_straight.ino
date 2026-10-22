/*
 * ============================================================================
 *  car_straight.ino —— 四轮直行
 * ----------------------------------------------------------------------------
 *  硬件：Arduino UNO R3 + Keyes L298P 电机驱动扩展板（KE0163）
 *
 *  【本车接线】A 组 = 左侧，B 组 = 右侧
 *    A1 = 左前轮，A2 = 左后轮 —— 并联在 A 通道：方向 D2，调速 D6
 *    B1 = 右前轮，B2 = 右后轮 —— 并联在 B 通道：方向 D4，调速 D5
 *
 *  【关键限制】同一组的两台电机拿到的是同一路驱动信号，程序无法只改其中一台：
 *    - A1 与 A2 的速度/方向必须一致，B1 与 B2 必须一致（写成不一致会编译报错）；
 *    - 某台轮子实际转向反了（接线极性/安装方向相反），只能对调它的两根线。
 *
 *  【动作】上电后先停 2 秒（留时间放稳小车），随后四轮正转直行。
 * ============================================================================
 */

/* ====================== ① 引脚定义（A 组 = 左侧，B 组 = 右侧） ====================== */

// A 组（左侧）
#define A1_DIR_PIN  2      // A1（左前轮）方向脚（IN）
#define A1_PWM_PIN  6      // A1（左前轮）调速脚（EN）
#define A2_DIR_PIN  2      // A2（左后轮）方向脚，与 A1 同通道
#define A2_PWM_PIN  6      // A2（左后轮）调速脚，与 A1 同通道

// B 组（右侧）
#define B1_DIR_PIN  4      // B1（右前轮）方向脚（IN）
#define B1_PWM_PIN  5      // B1（右前轮）调速脚（EN）
#define B2_DIR_PIN  4      // B2（右后轮）方向脚，与 B1 同通道
#define B2_PWM_PIN  5      // B2（右后轮）调速脚，与 B1 同通道

// 方向脚电平：本扩展板前进时方向脚为 HIGH，后退为 LOW
#define MOTOR_FORWARD_LEVEL   HIGH
#define MOTOR_BACKWARD_LEVEL  LOW

/* ====================== ② 每个电机的速度与方向 ====================== */

// 速度：0 ~ 255；方向：1 = 正转（车轮朝前滚），-1 = 反转
#define A1_SPEED   150
#define A1_DIR     1
#define A2_SPEED   150
#define A2_DIR     1
#define B1_SPEED   150
#define B1_DIR     1
#define B2_SPEED   150
#define B2_DIR     1

// 编译期检查：A1/A2、B1/B2 各并联在同一通道，组内速度/方向必须一致
#if (A1_SPEED != A2_SPEED) || (A1_DIR != A2_DIR)
#error "A1 与 A2 并联在同一通道：速度/方向必须一致；若某台转向相反，请对调它的两根线"
#endif
#if (B1_SPEED != B2_SPEED) || (B1_DIR != B2_DIR)
#error "B1 与 B2 并联在同一通道：速度/方向必须一致；若某台转向相反，请对调它的两根线"
#endif

/* ====================== ③ 电机驱动 ====================== */

// 驱动一台电机（实际驱动的是它所在的整组通道）：速度 0~255，方向 1/-1
void driveMotor(uint8_t dirPin, uint8_t pwmPin, int speed, int dir) {
  if (speed <= 0) {          // 0 = 停
    analogWrite(pwmPin, 0);
    return;
  }
  digitalWrite(dirPin, dir >= 0 ? MOTOR_FORWARD_LEVEL : MOTOR_BACKWARD_LEVEL);
  analogWrite(pwmPin, constrain(speed, 0, 255));
}

/* ====================== ④ 运动动作 ====================== */

// 四轮正转，直线前进
void carForward() {
  driveMotor(A1_DIR_PIN, A1_PWM_PIN, A1_SPEED, A1_DIR);   // 左侧：左前 + 左后
  driveMotor(A2_DIR_PIN, A2_PWM_PIN, A2_SPEED, A2_DIR);
  driveMotor(B1_DIR_PIN, B1_PWM_PIN, B1_SPEED, B1_DIR);   // 右侧：右前 + 右后
  driveMotor(B2_DIR_PIN, B2_PWM_PIN, B2_SPEED, B2_DIR);
}

// 四轮反转，直线后退
void carBackward() {
  driveMotor(A1_DIR_PIN, A1_PWM_PIN, A1_SPEED, -A1_DIR);
  driveMotor(A2_DIR_PIN, A2_PWM_PIN, A2_SPEED, -A2_DIR);
  driveMotor(B1_DIR_PIN, B1_PWM_PIN, B1_SPEED, -B1_DIR);
  driveMotor(B2_DIR_PIN, B2_PWM_PIN, B2_SPEED, -B2_DIR);
}

// 四轮停止
void carStop() {
  analogWrite(A1_PWM_PIN, 0);   // 左通道
  analogWrite(B1_PWM_PIN, 0);   // 右通道
}

/* ====================== ⑤ 主程序 ====================== */

void setup() {
  pinMode(A1_DIR_PIN, OUTPUT);
  pinMode(A1_PWM_PIN, OUTPUT);
  pinMode(A2_DIR_PIN, OUTPUT);
  pinMode(A2_PWM_PIN, OUTPUT);
  pinMode(B1_DIR_PIN, OUTPUT);
  pinMode(B1_PWM_PIN, OUTPUT);
  pinMode(B2_DIR_PIN, OUTPUT);
  pinMode(B2_PWM_PIN, OUTPUT);

  carStop();       // 上电先确保四轮静止
  delay(2000);     // 留时间放稳小车、摆正车头
  carForward();    // 四轮正转直行
}

void loop() {
  // 空：PWM 会保持速度，无需重复输出
}
