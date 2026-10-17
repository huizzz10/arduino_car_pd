# Arduino 巡线 + OpenMV 辨色小车

一台 Arduino UNO 四轮小车：先用两路红外沿黑线逆时针跑完赛道（4 个左弯），再用
OpenMV4 H7 Plus 摄像头在场地里找气球。**巡线部分能上车，气球部分还在 WIP。**

> 代码和本文档由 AI 编码助手生成，属于 vibe coding 项目；上电、上车前请自己再过一遍。

## 功能

- **巡线（UNO）**：上电等待 → 直行进场 → 右转并线 → PD 巡线 → 数 4 个左弯 → 线尾全白刹车。
- **辨色（OpenMV）**：找最近的非红气球，把水平偏移、大小、距离通过 UART 发给 UNO。
  红球要躲开——按规则扎破红球比赛立即终止。

## 硬件

| 部件 | 型号 |
| --- | --- |
| 主控 | Arduino UNO R3 |
| 电机驱动 | Keyes L298P 扩展板（KE0163） |
| 电机 | 4 个直流减速电机，左右各 2 个并联 |
| 巡迹 | 2 路红外（手头是 3 路模块，第 3 路不用） |
| 摄像头 | OpenMV4 H7 Plus，2.8mm 镜头 |
| 电源 | 7~9V 电池组（别只靠 USB，电机一转就拉低电压） |

## 接线

| 信号 | 引脚 |
| --- | --- |
| 巡迹 S1（左）/ S2（右） | A1 / A2 |
| 左侧两轮（方向 / 调速） | D2 / D6 |
| 右侧两轮（方向 / 调速） | D4 / D5 |
| OpenMV P4（UART3 TX） | UNO RX（115200） |
| OpenMV GND | UNO GND（必接） |

P4/P5 同时是 UART3 和 I2C2，二选一；走 I2C 时 P4=SCL、P5=SDA，UNO 地址 `0x08`。

## 仓库结构

```
arduino/
├── pd_v3.0+1.01.ino      比赛版巡线主程序
├── car_straight.ino      最小直行示例
├── motor_test.ino        四轮电机测试
└── line_sensor_test.ino  巡迹极性测试
openmv/
├── balloon_core.py       辨色核心库（算法 + 参数 + 协议）
├── balloon_main.py       上车入口，部署时改名为 main.py
├── balloon_detect.py     调试版，按形状认球
├── live_view.py          诊断版，画框看颜色
└── minimal.py            最小测试，只开摄像头
```

## 快速开始

**Arduino**（`.ino` 要先放进同名文件夹，Arduino IDE 才能打开）：

1. `line_sensor_test.ino` 确认黑线是高电平还是低电平，把结果填进 `BLACK_LEVEL`；
2. `motor_test.ino` 架空测试四个轮子的转向和调速；
3. `car_straight.ino` 整车直线跑一段；
4. `pd_v3.0+1.01.ino` 上车跑赛道。停车后 D13 闪 1 下 = 超时保护，2 下 = 正常终点。

**OpenMV**（先按顺序逐个运行，最后再存进摄像头）：

1. `minimal.py` 看能不能出画面；
2. `live_view.py` 检查颜色框；
3. `balloon_detect.py` 只看形状能不能框到球；
4. 部署：先存 `balloon_core.py`，再把 `balloon_main.py` 存成 `main.py`。少存前者会 ImportError。

## 通信

摄像头每帧发 10 字节：`0xAA 0x56 | dx | size | flags | ped_dx | n_red | dist | seq | crc`。
`dx` 是目标相对画面中心的偏移（正 = 在右侧），`crc` 是 CRC-8/ATM（多项式 `0x07`、初值 `0x00`）。
详见 `balloon_core.py` 头部注释。

## 现阶段的坑

- `BLACK_LEVEL`、`PROBES_CENTERED_BOTH_ON_LINE`、`A_CHANNEL_IS_LEFT` 必须和实车一致，填错会丢线或画龙。
- `MIN_SPEED = 60` 是硬死区，`CORNER_PIVOT_INNER` 只能填 `0` 或 ≤ `-60`。
- 左弯有 `CORNER_TIMEOUT_MS` 硬上限，别原地盲转过半圈，否则探头会扫到旧线反着走。
- 气球颜色阈值只是估值，**现场灯光下必须重标**；红球阈值宁窄勿宽，误扎红球直接判负。
- UART 没有 ACK，自检通过不代表线接好了；TX/RX 接反或不共地会静默收不到。
- 弱电池会表现成“程序 bug”：大电流时 UNO 复位，车停两秒后从头再来。

## WIP

- UNO 侧收帧程序 `car_balloon.ino`（气球帧目前没有接收方）；
- 左转进场 + 扎球动作（现在巡线停在线尾就结束了）；
- 现场重标颜色阈值；
- 光信号启动、摄像头失联保护。

参数和调参细节都在源码头部注释里，README 不重复。
