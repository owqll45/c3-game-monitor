/*
 * xbox_ble — Xbox Series X|S 手柄 BLE 主机端(ESP-IDF NimBLE)
 * 广播识别: appearance=964 / 名称含 Xbox / 厂商数据 06 00 00..
 * 服务: 0x1812 HID(报告特征 0x2A4D), 0x180F 电池(0x2A19)
 * 首次需手柄进入配对模式(长按配对键), 配对成功后地址存 NVS, 以后自动重连
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    XBOX_ST_IDLE = 0,   // 未启用
    XBOX_ST_SCANNING,   // 搜寻中
    XBOX_ST_FOUND,      // 已搜索到
    XBOX_ST_CONNECTING, // 正在连接
    XBOX_ST_CONNECTED,  // 已连接
    XBOX_ST_FAILED,     // 连接失败
} xbox_state_t;

typedef struct {
    bool a, b, x, y, lb, rb, view, menu, share, xbox, ls, rs;
    bool up, down, left, right;
    uint16_t lx, ly, rx, ry;  // 0..65535, 中心 32768
    uint16_t lt, rt;          // 0..1023
    uint32_t last_ms;         // 最近一次报告时间
    uint32_t reports;         // 累计报告数
} xbox_input_t;

void xbox_init(void);          // 起 BLE 协议栈
void xbox_start_pairing(void); // 设置里点: 开始搜寻并连接
void xbox_disconnect(void);    // 断开并停止搜寻
bool xbox_is_connected(void);
xbox_state_t xbox_state(void);
const xbox_input_t* xbox_get_input(void);
uint8_t xbox_battery(void);    // 0..100, 255=未知
bool xbox_has_saved(void);     // NVS 里有配对记录
bool xbox_need_pair_button(void); // 看到手柄但它是不可连接广播 → 需长按配对键
