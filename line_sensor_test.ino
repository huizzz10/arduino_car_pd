/*
 * ============================================================================
 *  line_sensor_test.ino —— 巡线传感器极性测试
 *  目的：确认巡线传感器“检测到黑线”时输出的是 1（高电平）还是 0（低电平）
 * ----------------------------------------------------------------------------
 *  硬件平台：Arduino UNO R3 + 三路巡迹传感器模块（与 car_pd_line 小车同一套接线）
 *
 *  【接线】（若实际接线不同，只改下面“① 引脚定义”里的三个引脚即可）
 *      三路巡迹模块      S1 左 -> A1      S2 中 -> A2      S3 右 -> D3
 *                        V -> 5V          G -> GND
 *
 *  【使用方法】
 *      1) 烧录本程序，打开串口监视器（波特率 9600）；
 *      2) 把探头分别放到白纸 / 黑线上，串口会即时打印每一次电平变化；
 *      3) 也可自动判断：探头对准白纸时在串口输入字母 w，
 *         对准黑线时输入字母 b，程序会自动对比并给出结论。
 *
 *  【判读方法】某一路“白纸=1、黑线=0” -> 检测黑线输出低电平 0；
 *             某一路“白纸=0、黑线=1” -> 检测黑线输出高电平 1。
 *
 *  【结论怎么用】回到 pd代码中，把 BLACK_LEVEL 改成对应值：
 *      检测黑线输出 0 -> #define BLACK_LEVEL LOW
 *      检测黑线输出 1 -> #define BLACK_LEVEL HIGH
 *
 *  【提示】
 *      - 探头离地 1~6mm 检测最可靠，悬空太远会一直输出“未检测”；
 *      - 黑白数值区分不明显时，用小螺丝刀调模块上的灵敏度电位器；
 *      - 模块指示灯可作参考：多数模块检测到黑线时指示灯亮（以实测为准）。
 * ============================================================================
 */

/* ========================== ① 引脚定义 ========================== */

// 三路巡迹传感器（与 car_pd_line.ino 一致）
const uint8_t SENSOR_LEFT_PIN  = A1;
const uint8_t SENSOR_MID_PIN   = A2;
const uint8_t SENSOR_RIGHT_PIN = 3;

// L298P 扩展板电机引脚：测试期间保持停转，防止小车意外跑动
// （没接电机驱动板时，这段可以删掉）
const uint8_t MOTOR_PINS[4] = { 2, 4, 5, 6 };

/* ========================== ② 可调参数 ========================== */
#define HEARTBEAT_MS 2000   // 无变化时每隔多久打印一次当前状态(ms)

/* ========================== ③ 全局变量 ========================== */
const uint8_t SENSOR_COUNT = 3;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {
  SENSOR_LEFT_PIN, SENSOR_MID_PIN, SENSOR_RIGHT_PIN
};
const char* SENSOR_NAMES[SENSOR_COUNT] = { "S1左(A1)", "S2中(A2)", "S3右(D3)" };

int lastLevel[SENSOR_COUNT];    // 上一次读到的电平，-1 表示还没读过
int whiteLevel[SENSOR_COUNT];   // 按 w 记录的白纸状态，-1 表示未记录
int blackLevel[SENSOR_COUNT];   // 按 b 记录的黑线状态，-1 表示未记录

uint32_t g_lastSerialMs = 0;    // 最近一次收到串口输入的时间
uint32_t g_lastPrintMs  = 0;    // 最近一次打印状态的时间

/* ========================== ④ 工具函数 ========================== */

// 读三路电平到 levels[]（元素为 0 或 1）
void readLevels(int levels[]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    levels[i] = digitalRead(SENSOR_PINS[i]);
  }
}

// 打印一行三路状态：S1左(A1)=1 S2中(A2)=1 S3右(D3)=1
void printLevels(const int levels[]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i > 0) Serial.print(' ');
    Serial.print(SENSOR_NAMES[i]);
    Serial.print('=');
    Serial.print(levels[i]);
  }
  Serial.println();
}

// 比较白纸/黑线两次记录，给出“检测黑线输出 1 还是 0”的结论
void compareAndReport() {
  if (whiteLevel[0] < 0 || blackLevel[0] < 0) {
    Serial.println(F("（白纸按 w、黑线按 b 各记录一次后，这里会自动给出结论）"));
    return;
  }

  uint8_t votesLow  = 0;   // 判定“检测黑线输出 0”的路数
  uint8_t votesHigh = 0;   // 判定“检测黑线输出 1”的路数

  Serial.println(F("==== 自动判断结果 ===="));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(F(": 白纸="));
    Serial.print(whiteLevel[i]);
    Serial.print(F(", 黑线="));
    Serial.print(blackLevel[i]);
    if (whiteLevel[i] == blackLevel[i]) {
      Serial.println(F("  -> 两次相同（此路可能没动过）"));
    } else if (blackLevel[i] == LOW) {
      Serial.println(F("  -> 检测黑线输出 0 (LOW)"));
      votesLow++;
    } else {
      Serial.println(F("  -> 检测黑线输出 1 (HIGH)"));
      votesHigh++;
    }
  }

  if (votesLow > 0 && votesHigh == 0) {
    Serial.println(F("结论: 本模块检测黑线输出 低电平 0 (LOW)"));
    Serial.println(F("      请把 car_pd_line.ino 改成 #define BLACK_LEVEL LOW"));
  } else if (votesHigh > 0 && votesLow == 0) {
    Serial.println(F("结论: 本模块检测黑线输出 高电平 1 (HIGH)"));
    Serial.println(F("      请保持 car_pd_line.ino 中 #define BLACK_LEVEL HIGH"));
  } else if (votesLow == 0 && votesHigh == 0) {
    Serial.println(F("未观察到任何一路变化：请确认探头确实贴在白纸/黑线上（离地 1~6mm），"));
    Serial.println(F("必要时调节灵敏度电位器后，重新按 w 和 b 记录。"));
  } else {
    Serial.println(F("各路结论不一致：可能个别探头没对正，请重新按 w 和 b 记录一次。"));
  }
}

// 串口命令：w=记录白纸状态，b=记录黑线状态
void handleSerialInput() {
  bool gotInput = false;
  while (Serial.available() > 0) {
    char c = Serial.read();
    gotInput = true;
    if (c == 'w' || c == 'W') {
      readLevels(whiteLevel);
      Serial.print(F("已记录【白纸】状态: "));
      printLevels(whiteLevel);
      compareAndReport();
    } else if (c == 'b' || c == 'B') {
      readLevels(blackLevel);
      Serial.print(F("已记录【黑线】状态: "));
      printLevels(blackLevel);
      compareAndReport();
    } else if (c != '\r' && c != '\n') {
      Serial.println(F("（输入 w 记录白纸状态，输入 b 记录黑线状态）"));
    }
  }
  if (gotInput) {
    g_lastSerialMs = millis();
    g_lastPrintMs  = millis();
  }
}

/* ========================== ⑤ setup / loop ========================== */

void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    lastLevel[i]  = -1;
    whiteLevel[i] = -1;
    blackLevel[i] = -1;
  }

  // 电机保持停转，避免测试时小车乱跑
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_PINS[i], LOW);
  }

  Serial.println(F("=== 巡线传感器极性测试：检测黑线输出 1 还是 0 ==="));
  Serial.println(F("判读：白纸=1 且 黑线=0 -> 输出 0；白纸=0 且 黑线=1 -> 输出 1"));
  Serial.println(F("自动判断：探头对准白纸时输入 w，对准黑线时输入 b"));
  Serial.println(F("--------------------------------------------------"));

  readLevels(lastLevel);
  Serial.print(F("初始状态: "));
  printLevels(lastLevel);
}

void loop() {
  handleSerialInput();

  int levels[SENSOR_COUNT];
  readLevels(levels);

  bool changed = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (levels[i] != lastLevel[i]) {
      Serial.print(F("[变化] "));
      Serial.print(SENSOR_NAMES[i]);
      Serial.print(F(": "));
      Serial.print(lastLevel[i]);
      Serial.print(F(" -> "));
      Serial.println(levels[i]);
      lastLevel[i] = levels[i];
      changed = true;
    }
  }

  if (changed) {
    Serial.print(F("  当前 "));
    printLevels(levels);
    g_lastPrintMs = millis();
  } else if (millis() - g_lastPrintMs >= HEARTBEAT_MS &&
             millis() - g_lastSerialMs >= HEARTBEAT_MS) {
    g_lastPrintMs = millis();
    Serial.print(F("[运行中] "));
    printLevels(levels);
  }
}
