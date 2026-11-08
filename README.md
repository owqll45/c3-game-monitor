# C3 Game Monitor

ESP32-C3 + ST7789 (320×240) 的 Cardputer 游戏监视器 + Xbox 手柄桥。

主机端固件：[cardputer-arcade](https://github.com/owqll45/cardputer-arcade)

## 特性

- **实时镜像**：Cardputer 上打开什么 app，这块屏就显示什么（桌面图标+名字 / 五子棋棋盘 / Pong 球场）
- **Xbox 手柄主机端**：NimBLE central 直连 Xbox Series X|S 手柄（BLE HID），按键掩码经 ESP-NOW 转发给 Cardputer，实现双机双人对战
- **五子棋监视**：棋盘、比分、趋势图、成就、嘲讽、作弊横幅全量镜像
- **Pong 联机**：手柄控制右挡板，比分/胜负同步显示
- 按键(GPIO9) 设置菜单：重搜主机 / 手柄配对 / 背光 / 关于

## 硬件接线

ST7789: SCK=10 MOSI=7 MISO=6 CS=1 DC=0 BL=13，按键 GPIO9，电量 ADC 分压。

## 构建

```bash
idf.py build        # ESP-IDF v5.4+
idf.py -p COMx flash
```

## 协议

与主机端通过 ESP-NOW 通信，包格式见 [cardputer-arcade 的 README](https://github.com/owqll45/cardputer-arcade#联机协议简述)。
