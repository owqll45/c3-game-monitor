/*
 * gomoku_c3 — ESP32-C3 五子棋监视器(状态同步版)
 * 协议: Cardputer 每 500ms 广播 GMKBEAC+mac → 本机回 GMKHI1 → 主机回 GMKWELC
 *       → 主机每次画面变化发 GMKSTATE(0x20) 全量状态包 238B, 本机本地重绘
 *       (棋盘/光标/比分/趋势/嘲讽文本全在包里, 丢包自愈)
 *       心跳 GMKBEAT 1s; GMKBYE 断开; 3 秒无包判掉线
 * 按键(GPIO9): 只用于设置菜单, 短按=打开/下一项, 双击=确认, 三击=返回
 * 屏幕: ST7789 横屏 320x240, SCK=10 MOSI=7 MISO=6 CS=1 DC=0 BL=13
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "u8g2font.h"
#include "xbox_ble.h"

static const char* TAG = "mon";

// ---------------- 屏幕 ----------------
#define PIN_SCK  10
#define PIN_MOSI 7
#define PIN_MISO 6
#define PIN_CS   1
#define PIN_DC   0
#define PIN_BL   13
#define PIN_BTN  9

#define LCD_W 320
#define LCD_H 240

static spi_device_handle_t s_spi;

// 颜色: 帧缓冲用 8bit 调色板索引(省一半 RAM 给蓝牙), 调色板里放真实 565
typedef uint8_t color_t;
enum {
    CI_BLACK, CI_WHITE, CI_RED, CI_GREEN, CI_BLUE, CI_YELLOW, CI_CYAN,
    CI_GRAY, CI_DGRAY, CI_LGRAY, CI_ORANGE, CI_MAGENTA, CI_BG, CI_WOOD,
    CI_PANEL, CI_TREND, CI_TMID, CI_SELBG,
};
static const uint16_t kPal[256] = {
    [CI_BLACK]   = 0x0000,
    [CI_WHITE]   = 0xFFFF,
    [CI_RED]     = 0xF800,
    [CI_GREEN]   = 0x07E0,
    [CI_BLUE]    = 0x001F,
    [CI_YELLOW]  = 0xFFE0,
    [CI_CYAN]    = 0x07FF,
    [CI_GRAY]    = 0x7BEF,
    [CI_DGRAY]   = 0x39E7,
    [CI_LGRAY]   = 0xC618,
    [CI_ORANGE]  = 0xFC00,
    [CI_MAGENTA] = 0xF81F,
    [CI_BG]      = 0x3186,
    [CI_WOOD]    = 0xE79C,
    [CI_PANEL]   = 0x1082,
    [CI_TREND]   = 0x4208,
    [CI_TMID]    = 0x18E3,
    [CI_SELBG]   = 0x2104,
};

#define C_BLACK   CI_BLACK
#define C_WHITE   CI_WHITE
#define C_RED     CI_RED
#define C_GREEN   CI_GREEN
#define C_BLUE    CI_BLUE
#define C_YELLOW  CI_YELLOW
#define C_CYAN    CI_CYAN
#define C_GRAY    CI_GRAY
#define C_DGRAY   CI_DGRAY
#define C_LGRAY   CI_LGRAY
#define C_ORANGE  CI_ORANGE
#define C_MAGENTA CI_MAGENTA
#define C_BG      CI_BG     // Cardputer THEME_COLOR_BG 0x333333 → 565
#define C_WOOD    CI_WOOD   // 棋盘木色
#define C_PANEL   CI_PANEL  // 战报底
#define C_TREND   CI_TREND  // 趋势图边框
#define C_TMID    CI_TMID   // 趋势图中线
#define C_SELBG   CI_SELBG  // 菜单选中底

static inline uint16_t sw(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

static void lcd_write(const uint8_t* data, size_t len, int is_data)
{
    if (len == 0) return;
    gpio_set_level(PIN_DC, is_data ? 1 : 0);
    gpio_set_level(PIN_CS, 0);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(PIN_CS, 1);
}
static void lcd_cmd(uint8_t cmd) { lcd_write(&cmd, 1, 0); }
static void lcd_data1(uint8_t d) { lcd_write(&d, 1, 1); }

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];
    lcd_cmd(0x2A);
    d[0] = x0 >> 8; d[1] = x0; d[2] = x1 >> 8; d[3] = x1;
    lcd_write(d, 4, 1);
    lcd_cmd(0x2B);
    d[0] = y0 >> 8; d[1] = y0; d[2] = y1 >> 8; d[3] = y1;
    lcd_write(d, 4, 1);
    lcd_cmd(0x2C);
}

static void st7789_init(void)
{
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x3A); lcd_data1(0x55);
    lcd_cmd(0x36); lcd_data1(0x60); // 横屏 MX|MV
    lcd_cmd(0x20);                  // 反色关(此屏开反色会全反)
    lcd_cmd(0x13);
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// ---------------- 中文字体 ----------------
extern const uint8_t lgfx_efont_cn_10_b[];
extern const uint8_t lgfx_efont_cn_12[];
extern const uint8_t lgfx_efont_cn_14[];
#define F10B lgfx_efont_cn_10_b
#define F12  lgfx_efont_cn_12
#define F14  lgfx_efont_cn_14

// ---------------- 内存帧缓冲(320x240x1 = 75KB 索引色, 画完一次推屏, 不闪) ----------------
static uint8_t s_scr[LCD_W * LCD_H];
static uint16_t s_push[LCD_W * 4];

static void fb_px(int x, int y, color_t c)
{
    if ((unsigned)x < LCD_W && (unsigned)y < LCD_H) s_scr[y * LCD_W + x] = c;
}

static void fb_fill(int x, int y, int w, int h, color_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w <= 0 || h <= 0) return;
    for (int j = 0; j < h; j++) {
        uint8_t* row = s_scr + (y + j) * LCD_W + x;
        memset(row, c, w);
    }
}

static void fb_hline(int x, int y, int n, color_t c) { fb_fill(x, y, n, 1, c); }
static void fb_vline(int x, int y, int n, color_t c) { fb_fill(x, y, 1, n, c); }

static void fb_rect(int x, int y, int w, int h, color_t c)
{
    fb_hline(x, y, w, c);
    fb_hline(x, y + h - 1, w, c);
    fb_vline(x, y, h, c);
    fb_vline(x + w - 1, y, h, c);
}

static void fb_line(int x0, int y0, int x1, int y1, color_t c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1) {
        fb_px(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void fb_disc(int cx, int cy, int r, color_t c)
{
    for (int dy = -r; dy <= r; dy++) {
        int half = (int)sqrtf((float)(r * r - dy * dy));
        fb_hline(cx - half, cy + dy, 2 * half + 1, c);
    }
}

static void fb_circle(int cx, int cy, int r, color_t c)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        fb_px(cx + x, cy + y, c); fb_px(cx - x, cy + y, c);
        fb_px(cx + x, cy - y, c); fb_px(cx - x, cy - y, c);
        fb_px(cx + y, cy + x, c); fb_px(cx - y, cy + x, c);
        fb_px(cx + y, cy - x, c); fb_px(cx - y, cy - x, c);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

static uint8_t s_gbuf[20 * 20];
uint32_t g_push_cnt = 0;
uint32_t g_push_err = 0;
uint32_t g_render_cnt = 0;

// 透明背景画一个 glyph, 返回步进
static int fb_glyph(const uint8_t* font, int x, int ytop, uint32_t cp, color_t fg)
{
    u8g2_glyph_t g;
    if (u8g2_decode(font, cp, &g, s_gbuf) != 0) return 0;
    if (g.w > 20 || g.h > 20) return g.xadv; // 超出静态缓冲, 跳过
    int base = u8g2_font_baseline(font);
    int gx = x + g.xoff;
    int gy = ytop + base - g.yoff - g.h;
    for (int j = 0; j < g.h; j++)
        for (int i = 0; i < g.w; i++)
            if (s_gbuf[j * g.w + i]) fb_px(gx + i, gy + j, fg);
    return g.xadv;
}

static int fb_text(const uint8_t* font, int x, int y, const char* s, color_t fg)
{
    while (*s) {
        uint32_t cp = u8g2_utf8_next(&s);
        x += fb_glyph(font, x, y, cp, fg);
    }
    return x;
}

// 在区域 [x0, x0+w) 内水平居中
static void fb_text_cx(int x0, int w, int y, const char* s, const uint8_t* font, color_t fg)
{
    int tw = u8g2_str_width(font, s);
    int x = x0 + (w - tw) / 2;
    if (x < x0) x = x0;
    fb_text(font, x, y, s, fg);
}

static void push_screen(void)
{
    extern uint32_t g_push_cnt;
    extern uint32_t g_push_err;
    g_push_cnt++;
    lcd_set_window(0, 0, LCD_W - 1, LCD_H - 1);
    gpio_set_level(PIN_DC, 1);
    gpio_set_level(PIN_CS, 0);
    for (int y = 0; y < LCD_H; y += 4) {
        const uint8_t* src = s_scr + y * LCD_W;
        for (int i = 0; i < LCD_W * 4; i++) s_push[i] = sw(kPal[src[i]]);
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = LCD_W * 4 * 16;
        t.tx_buffer = s_push;
        esp_err_t e = spi_device_polling_transmit(s_spi, &t);
        if (e != ESP_OK) g_push_err++;
    }
    gpio_set_level(PIN_CS, 1);
}

// ================= 电量 ADC 自动探测 =================
static uint32_t now_ms(void);
// C3 自带电池但分压引脚未知: 扫 GPIO1..4, 找落在锂电池 1:2 分压区间的那个
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static int s_batt_gpio = -2;   // -2=未探测 -1=未找到 >=0 引脚
static int s_batt_pct = 255;   // 本机电量
static uint8_t s_host_pct = 255; // Cardputer 电量(状态包带过来)
static uint8_t s_net_flags = 0;  // 联机对战: bit0联机中 bit1键盘准备 bit2手柄准备 bit3已开局
static uint8_t s_cur_player = 1; // 当前该谁下 1=黑 2=白
static int s_cheat_mine = 0;     // 手柄方(本机)作弊辅助开关, 手柄 Y 键切换

static int adc_read_mv(int gpio)
{
    int raw = 0, mv = 0;
    if (!s_adc) return -1;
    if (adc_oneshot_read(s_adc, (adc_channel_t)gpio, &raw) != ESP_OK) return -1;
    if (s_cali && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return mv;
    return raw * 3100 / 4095; // 无校准表时按线性估算
}

static int mv_to_pct(int mv)
{
    if (mv <= 3000) return 0;
    if (mv >= 4200) return 100;
    if (mv <= 3700) return (mv - 3000) * 45 / 700;
    return 45 + (mv - 3700) * 55 / 500;
}

static void batt_init(void)
{
    adc_oneshot_unit_init_cfg_t u = { 0 };
    u.unit_id = ADC_UNIT_1;
    if (adc_oneshot_new_unit(&u, &s_adc) != ESP_OK) {
        ESP_LOGW(TAG, "adc init fail");
        s_adc = NULL;
        return;
    }
    adc_oneshot_chan_cfg_t c = { 0 };
    c.atten = ADC_ATTEN_DB_12;
    c.bitwidth = ADC_BITWIDTH_12;
    // 只配 GPIO2..4: GPIO0/1 是 LCD 的 DC/CS, 绝不能碰
    for (int g = 2; g <= 4; g++) adc_oneshot_config_channel(s_adc, (adc_channel_t)g, &c);

    // 校准表: 让 mV 是真实电压(C3 的 ADC 线性估算会偏高)
    adc_cali_curve_fitting_config_t cal = { 0 };
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_12;
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "adc cali fail, 用线性估算");
        s_cali = NULL;
    }

    int best = -1, best_mv = 0, best_span = 99999;
    for (int g = 2; g <= 4; g++) { // 跳过 GPIO0(DC) / GPIO1(CS)
        int mn = 99999, mx = 0, sum = 0;
        for (int i = 0; i < 8; i++) {
            int mv = adc_read_mv(g);
            if (mv < 0) mv = 0;
            if (mv < mn) mn = mv;
            if (mv > mx) mx = mv;
            sum += mv;
        }
        int avg = sum / 8, span = mx - mn;
        ESP_LOGI(TAG, "adc gpio%d avg=%dmV span=%dmV", g, avg, span);
        // 电池经分压: 引脚约 1.5~2.1V; 放宽到 1.4~2.25V 且要求读数稳定
        if (avg >= 1400 && avg <= 2250 && span < 120 && span < best_span) {
            best = g;
            best_mv = avg;
            best_span = span;
        }
    }
    s_batt_gpio = best;
    if (best >= 0) s_batt_pct = (uint8_t)mv_to_pct(best_mv * 2);
    ESP_LOGI(TAG, "battery gpio=%d pin=%dmV pct=%d", best, best_mv, s_batt_pct);
}

static void batt_update(void)
{
    if (s_batt_gpio < 0 || !s_adc) return;
    int sum = 0, n = 0;
    for (int i = 0; i < 6; i++) {
        int mv = adc_read_mv(s_batt_gpio);
        if (mv >= 0) { sum += mv; n++; }
    }
    if (!n) return;
    s_batt_pct = (uint8_t)mv_to_pct((sum / n) * 2);
}

// ================= 顶部彩色提示(队列, 逐条连贯显示) =================
#define TOAST_Q 4
typedef struct {
    char txt[40];
    color_t col;
    uint32_t dur; // 0 = 常驻直到被替换
} toast_item_t;

static toast_item_t s_tq[TOAST_Q];
static int s_tq_n = 0, s_tq_i = 0;
static uint32_t s_tq_at = 0;

static void toast_clear(void) { s_tq_n = 0; s_tq_i = 0; }

static void toast_push(const char* t, color_t col, uint32_t dur)
{
    if (s_tq_n >= TOAST_Q) { // 满了丢最旧
        for (int i = 1; i < s_tq_n; i++) s_tq[i - 1] = s_tq[i];
        s_tq_n--;
        if (s_tq_i > 0) s_tq_i--;
    }
    toast_item_t* it = &s_tq[s_tq_n++];
    strncpy(it->txt, t, sizeof(it->txt) - 1);
    it->txt[sizeof(it->txt) - 1] = 0;
    it->col = col;
    it->dur = dur;
    if (s_tq_n == 1) s_tq_at = now_ms();
}

static void toast_set(const char* t, color_t col, uint32_t dur)
{
    toast_clear();
    toast_push(t, col, dur);
}

static bool toast_active(void) { return s_tq_n > 0 && s_tq_i < s_tq_n; }

static bool toast_tick(void)
{
    if (!toast_active()) return false;
    toast_item_t* it = &s_tq[s_tq_i];
    if (it->dur && now_ms() - s_tq_at >= it->dur) {
        s_tq_i++;
        s_tq_at = now_ms();
        return true;
    }
    return false;
}

static void toast_draw(void)
{
    if (!toast_active()) return;
    toast_item_t* it = &s_tq[s_tq_i];
    int tw = u8g2_str_width(F12, it->txt) + 20;
    if (tw > LCD_W - 8) tw = LCD_W - 8;
    int bx = (LCD_W - tw) / 2, by = 4;
    fb_fill(bx, by, tw, 18, C_BLACK);
    fb_rect(bx, by, tw, 18, it->col);
    fb_text_cx(bx, tw, by + 2, it->txt, F12, it->col);
}

// ================= 电量图标 =================
static void draw_batt_icon(int x, int y, color_t col, int pct)
{
    const int w = 16, h = 9;
    fb_rect(x, y, w, h, col);
    fb_fill(x + w, y + 3, 2, 3, col);
    if (pct < 0) return;
    int bars = (pct * (w - 4) + 99) / 100;
    if (bars < 1) bars = 1;
    fb_fill(x + 2, y + 2, bars, h - 4, col);
}

static void draw_batt_row(void)
{
    // 主机(Cardputer)=绿色, 本机=白色, 两个贴在一起, 不显示百分比
    const int y = 222;
    draw_batt_icon(274, y, C_GREEN, s_host_pct == 255 ? -1 : s_host_pct);
    draw_batt_icon(292, y, C_WHITE, s_batt_pct == 255 ? -1 : s_batt_pct);
}

// ================= 信号强度 / 丢包 =================
static int s_rssi = -127;      // 对端信号 dBm
static int s_loss_pct = 0;     // 丢包率

static int rssi_bars(int v)
{
    if (v >= -60) return 4;
    if (v >= -68) return 3;
    if (v >= -76) return 2;
    if (v >= -88) return 1;
    return 0;
}

// 右下角: 4 格信号条 + dBm (靠最右边)
static void draw_signal_right(int xr, int y)
{
    int bars = rssi_bars(s_rssi);
    color_t c = bars >= 3 ? C_GREEN : (bars == 2 ? C_YELLOW : (bars == 1 ? C_ORANGE : C_RED));
    char buf[28];
    if (s_rssi <= -127) strcpy(buf, "--");
    else snprintf(buf, sizeof(buf), "%ddBm", s_rssi);
    int tw = u8g2_str_width(F10B, buf);
    int bw = 22;
    int x = xr - tw - bw - 4;      // 文字右端贴 xr
    for (int i = 0; i < 4; i++) {
        int h = 3 + i * 2;
        fb_fill(x + i * 6, y + (9 - h), 4, h, i < bars ? c : C_DGRAY);
    }
    fb_text(F10B, x + bw + 4, y, buf, s_rssi <= -127 ? C_DGRAY : c);
}

// 底部信息带(棋盘下方): 胜率条 + 事件日志
static char s_evt[40] = "等待对局...";
static uint32_t s_evt_ms = 0;

static void push_event(const char* txt)
{
    snprintf(s_evt, sizeof(s_evt), "%s", txt);
    s_evt_ms = now_ms();
}

// 右下角附加信息: 只保留丢包(延迟/性能已按用户要求移除)
static void draw_link_stats(void)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "丢%d%%", s_loss_pct);
    fb_text(F10B, 238, 222, buf, s_loss_pct > 10 ? C_RED : C_LGRAY);
}

// ================= ESP-NOW 客户端 + 状态包 =================
#define GMK_CH 1
#define GMK_STATE_LEN 240

// flags0 位
#define F0_OVER   1
#define F0_ANIM   2
#define F0_TAUNT  4
#define F0_PANEL  8
#define F0_THREAT 16
#define F0_REPLAY 32
#define F0_HELP   64
#define F0_RESULT 128
// flags1 位
#define F1_PVP    1
#define F1_ASSIST 2
#define F1_BLITZ  4
#define F1_RCONF  8
#define F1_AIMODE 16
#define F1_CHEAT  32   // 键盘方(对手)开启作弊辅助 → 本机顶部滚动横幅
#define F1_TAUNT_REMOTE 128 // 当前嘲讽是本机(手柄方)自己发的, 不显示在自己屏幕上

typedef struct {
    uint8_t flags0, flags1, winner, cur_player, difficulty, persona, mood;
    uint8_t cursor_x, cursor_y, move_count, last_r, last_c;
    uint8_t hint_r, hint_c;   // PVP 时复用 replay_idx/total 两个字节, 作弊提示点(255=无)
    uint8_t replay_idx, replay_total;
    uint8_t pw, aw, streak, best;
    uint8_t danger, cheat, blunders, min_danger, worst_no;
    uint16_t achv, achv_new;
    uint8_t clk1, clk2;
    uint16_t game_sec;
    int32_t score1, score2;
    uint8_t trend_count, trend_head;
    int8_t trend[24];
    uint8_t taunt_kind;
    char taunt[3][18];
    uint8_t threat[15][15];
    uint8_t board[15][15];
} gmk_state_t;

static gmk_state_t s_gs;
static volatile int s_gs_ready = 0;
static int s_have_state = 0;
static uint32_t s_anim_ms = 0;      // 胜利动画本地计时起点
static uint32_t s_last_render = 0;

static uint8_t s_host[6];
static volatile int s_linked = 0;
static uint32_t s_last_pkt_ms = 0;
static uint32_t s_hello_last = 0;

// ---- 多游戏镜像: 当前 app / 游戏画面切换(GMKAPP 0x10) ----
#define GID_LAUNCHER 0
#define GID_GOMOKU   1
#define GID_PONG     2
static uint8_t s_game_id = GID_GOMOKU;  // 默认五子棋, 兼容不发 GMKAPP 的旧主机
static uint8_t s_icon_id = 0;
static char s_app_name[17] = "Gomoku";

// Pong 镜像状态(PONGSTATE 0x21, 20B)
typedef struct {
    int16_t ball_x, ball_y;
    uint8_t p1y, p2y, score1, score2, flags, winner;
} pong_state_t;
static pong_state_t s_pong;
static int s_pong_have = 0;

// 当前画面是否有数据可渲染
static int view_ready(void)
{
    if (s_game_id == GID_GOMOKU) return s_have_state;
    if (s_game_id == GID_PONG) return s_pong_have;
    return 1; // Launcher 等: 收到 GMKAPP 就能画
}

static void parse_pong(const uint8_t* d)
{
    s_pong.ball_x = (int16_t)(d[4] | (d[5] << 8));
    s_pong.ball_y = (int16_t)(d[6] | (d[7] << 8));
    s_pong.p1y = d[8]; s_pong.p2y = d[9];
    s_pong.score1 = d[10]; s_pong.score2 = d[11];
    s_pong.flags = d[12]; s_pong.winner = d[13];
    s_pong_have = 1;
    s_gs_ready = 1;
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void add_peer(const uint8_t* mac)
{
    esp_now_peer_info_t p;
    memset(&p, 0, sizeof(p));
    memcpy(p.peer_addr, mac, 6);
    p.channel = GMK_CH;
    p.ifidx = WIFI_IF_STA;
    p.encrypt = false;
    esp_now_del_peer(mac);
    esp_now_add_peer(&p);
}

static void parse_state(const uint8_t* d)
{
    gmk_state_t* g = &s_gs;
    int prev_anim = g->flags0 & F0_ANIM;
    uint8_t prev_mc = g->move_count, prev_cheat = g->cheat, prev_over = g->flags0 & F0_OVER;
    int prev_taunt = g->flags0 & F0_TAUNT;
    g->flags0 = d[4]; g->flags1 = d[5]; g->winner = d[6]; g->cur_player = d[7];
    g->difficulty = d[8]; g->persona = d[9]; g->mood = d[10];
    g->cursor_x = d[11]; g->cursor_y = d[12]; g->move_count = d[13];
    g->last_r = d[14]; g->last_c = d[15]; g->replay_idx = d[16]; g->replay_total = d[17];
    // PVP 时 d[16]/d[17] 被主机复用成作弊提示点坐标(255=无)
    g->hint_r = (g->flags1 & F1_PVP) ? d[16] : 255;
    g->hint_c = (g->flags1 & F1_PVP) ? d[17] : 255;
    g->pw = d[18]; g->aw = d[19]; g->streak = d[20]; g->best = d[21];
    g->danger = d[22]; g->cheat = d[23]; g->blunders = d[24];
    g->min_danger = d[25]; g->worst_no = d[26];
    g->achv = (uint16_t)(d[27] | (d[28] << 8));
    g->achv_new = (uint16_t)(d[29] | (d[30] << 8));
    g->clk1 = d[31]; g->clk2 = d[32];
    g->game_sec = (uint16_t)(d[33] | (d[34] << 8));
    memcpy(&g->score1, d + 35, 4);
    memcpy(&g->score2, d + 39, 4);
    g->trend_count = d[43]; g->trend_head = d[44];
    memcpy(g->trend, d + 45, 24);
    g->taunt_kind = d[69];
    memcpy(g->taunt, d + 70, 54);
    g->taunt[0][17] = g->taunt[1][17] = g->taunt[2][17] = 0;
    for (int i = 0; i < 57; i++) {
        uint8_t v = d[124 + i];
        for (int k = 0; k < 4; k++) {
            int idx = i * 4 + k;
            if (idx >= 225) break;
            g->threat[idx / 15][idx % 15] = (v >> (k * 2)) & 3;
        }
    }
    for (int i = 0; i < 57; i++) {
        uint8_t v = d[181 + i];
        for (int k = 0; k < 4; k++) {
            int idx = i * 4 + k;
            if (idx >= 225) break;
            g->board[idx / 15][idx % 15] = (v >> (k * 2)) & 3;
        }
    }
    s_host_pct = d[238];
    s_net_flags = d[239];
    s_cur_player = d[7];
    // 胜利动画上升沿: 本地重新计时
    if ((g->flags0 & F0_ANIM) && !prev_anim) s_anim_ms = now_ms();

    // ---- 事件日志 ----
    if (g->move_count != prev_mc && g->move_count > 0 && g->last_r < 15 && g->last_c < 15) {
        char eb[40];
        int who = g->board[g->last_r][g->last_c];
        snprintf(eb, sizeof(eb), "%s落子 %c%d", who == 2 ? "白方" : "黑方",
                 'A' + g->last_c, g->last_r + 1);
        push_event(eb);
    }
    if (g->cheat > prev_cheat) push_event("对方开启作弊模式!");
    else if ((s_net_flags & 0x01) && (g->flags0 & F0_TAUNT) && !prev_taunt &&
             !(g->flags1 & F1_TAUNT_REMOTE)) push_event("对方发来嘲讽");
    else if ((g->flags0 & F0_OVER) && !prev_over) push_event("本局结束");

    // ---- 丢包率: 状态包序号跳变 ----
    {
        static uint8_t last_seq = 0;
        static int rx = 0, lost = 0;
        uint8_t sq = d[3];
        if (last_seq) {
            int gap = (int)(uint8_t)(sq - last_seq - 1);
            if (gap > 0 && gap < 30) lost += gap;
        }
        last_seq = sq;
        rx++;
        int tot = rx + lost;
        s_loss_pct = tot ? (lost * 100 / tot) : 0;
        if (rx > 3000) { rx /= 2; lost /= 2; }
    }

    s_have_state = 1;
    s_gs_ready = 1;
}

static void on_recv(const esp_now_recv_info_t* info, const uint8_t* d, int len)
{
    s_last_pkt_ms = now_ms();
    // 信号强度: 取对端发来的 GMK 包
    if (info && info->rx_ctrl && len >= 3 && memcmp(d, "GMK", 3) == 0)
        s_rssi = info->rx_ctrl->rssi;
    {
        static uint32_t n_beac = 0, n_welc = 0, n_state = 0, n_other = 0;
        if (len == 13 && memcmp(d, "GMKBEAC", 7) == 0) n_beac++;
        else if (len == 7 && memcmp(d, "GMKWELC", 7) == 0) n_welc++;
        else if (len == GMK_STATE_LEN) n_state++;
        else n_other++;
        static uint32_t last_log = 0;
        if (now_ms() - last_log > 5000) {
            last_log = now_ms();
            ESP_LOGI(TAG, "rx beac=%u welc=%u state=%u other=%u lastlen=%d",
                     (unsigned)n_beac, (unsigned)n_welc, (unsigned)n_state,
                     (unsigned)n_other, len);
        }
    }
    if (len == 13 && memcmp(d, "GMKBEAC", 7) == 0) {
        if (!s_linked) {
            memcpy(s_host, d + 7, 6);
            add_peer(s_host);
            if (now_ms() - s_hello_last > 800) {
                s_hello_last = now_ms();
                esp_now_send(s_host, (const uint8_t*)"GMKHI1", 6);
            }
        }
        return;
    }
    if (len == 7 && memcmp(d, "GMKWELC", 7) == 0) {
        if (!s_linked) {
            memcpy(s_host, info->src_addr, 6);
            add_peer(s_host);
            s_linked = 1;
            ESP_LOGI(TAG, "linked");
        }
        return;
    }
    if (len == 6 && memcmp(d, "GMKBYE", 6) == 0) {
        s_linked = 0;
        return;
    }
    if (len == 24 && d[0] == 'G' && d[1] == 'M' && d[2] == 0x10) {
        // GMKAPP: 前台 app 切换(名字+游戏id+图标id)
        s_game_id = d[4];
        s_icon_id = d[5];
        memcpy(s_app_name, d + 6, 16);
        s_app_name[16] = 0;
        s_gs_ready = 1;
        return;
    }
    if (len == 20 && d[0] == 'G' && d[1] == 'M' && d[2] == 0x21) {
        parse_pong(d);
        return;
    }
    if (len == GMK_STATE_LEN && d[0] == 'G' && d[1] == 'M' && d[2] == 0x20) {
        parse_state(d);
    }
}

static void wifi_espnow_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(GMK_CH, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    esp_now_register_recv_cb(on_recv);
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "C3 mac %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ================= 游戏画面渲染(Cardputer 布局放大版) =================
#define CELL 12
#define BPD  (CELL * 15)   // 180
#define BSX  70
#define BSY  30
#define LX   4             // 左栏
#define RX   (BSX + BPD + 6)  // 256 右栏
#define RW   (LCD_W - RX)     // 64

static const char* kPN[3] = { "稳健", "凶狠", "诡诈" };
static const color_t kPC[3] = { C_GREEN, C_RED, C_MAGENTA };

static void draw_trend(int x, int y, int w, int h)
{
    fb_rect(x, y, w, h, C_TREND);
    int mid = y + h / 2;
    fb_hline(x + 1, mid, w - 2, C_TMID);
    int cnt = s_gs.trend_count;
    if (cnt < 1) return;
    if (cnt > 24) cnt = 24;
    int px_ = -1, py_ = -1;
    for (int i = 0; i < cnt; i++) {
        int idx = (s_gs.trend_head - cnt + i + 24) % 24;
        int v = s_gs.trend[idx];
        int xx = x + 1 + i * (w - 2) / 23;
        int yy = mid - v * (h / 2 - 2) / 5;
        color_t col = (v >= 0) ? C_GREEN : C_RED;
        if (px_ >= 0) {
            fb_line(px_, py_, xx, yy, col);
            fb_line(px_, py_ + 1, xx, yy + 1, col);
        }
        fb_fill(xx - 1, yy - 1, 3, 3, col);
        px_ = xx; py_ = yy;
    }
}

static void draw_panel(void)
{
    int x = RX, w = RW;
    fb_text(F12, x, 4, "你还剩", C_WHITE);

    char buf[24];
    if (s_gs.danger >= 10) snprintf(buf, sizeof(buf), "%d", s_gs.danger);
    else if (s_gs.danger > 0) snprintf(buf, sizeof(buf), "%d", s_gs.danger);
    else snprintf(buf, sizeof(buf), "--");
    color_t dc = s_gs.danger > 0
        ? (s_gs.danger <= 3 ? C_RED : (s_gs.danger <= 10 ? C_ORANGE : C_YELLOW))
        : C_DGRAY;
    fb_text(F14, x + 2, 18, buf, dc);
    int tw = u8g2_str_width(F14, buf);
    fb_text(F12, x + 6 + tw, 24, "步", C_DGRAY);

    // 性格竖排(右侧竖条)
    int pid = s_gs.persona < 3 ? s_gs.persona : 0;
    int vx = x + w - 18;
    for (int i = 0; i < 2; i++) {
        char ch[4] = { kPN[pid][i * 3], kPN[pid][i * 3 + 1], kPN[pid][i * 3 + 2], 0 };
        fb_text(F14, vx, 4 + i * 18, ch, kPC[pid]);
    }

    draw_trend(x, 42, w, 26);

    snprintf(buf, sizeof(buf), "MOVE %d", s_gs.move_count);
    fb_text(F12, x, 72, buf, C_LGRAY);

    snprintf(buf, sizeof(buf), "作弊%d %s", s_gs.cheat,
             s_gs.mood == 1 ? "急" : (s_gs.mood == 2 ? "浪" : "稳"));
    fb_text(F12, x, 86, buf, C_MAGENTA);

    // 胜利嘲讽(3 行)时 Balance 区让位(Cardputer 同款逻辑)
    if (!((s_gs.flags0 & F0_TAUNT) && s_gs.taunt_kind == 3)) {
        int32_t s1 = s_gs.score1 > 0 ? s_gs.score1 : 0;
        int32_t s2 = s_gs.score2 > 0 ? s_gs.score2 : 0;
        int split = (int)((int64_t)(s1 + 1) * w / (s1 + s2 + 2));
        fb_fill(x, 132, split, 4, C_GREEN);
        fb_fill(x + split, 132, w - split, 4, C_RED);

        int32_t adv = s_gs.score2 - s_gs.score1;
        int ai_rate = 50 - (int)(adv / 400);
        if (ai_rate < 5) ai_rate = 5;
        if (ai_rate > 95) ai_rate = 95;
        snprintf(buf, sizeof(buf), "你%d%%", 100 - ai_rate);
        fb_text(F12, x + 2, 140, buf, (100 - ai_rate) >= 50 ? C_GREEN : C_RED);
    }
}

static void draw_overlays(void)
{
    // 胜利动画(本地计时复刻 Cardputer 三阶段)
    if (s_gs.flags0 & F0_ANIM) {
        uint32_t el = now_ms() - s_anim_ms;
        int bx = BSX + 8, by = BSY + BPD / 2 - 22, bw = BPD - 16;
        fb_fill(bx, by, bw, 44, C_BLACK);
        fb_rect(bx, by, bw, 44, C_YELLOW);
        if (el < 600) {
            fb_text_cx(bx, bw, by + 6, "你赢了！", F12, C_GREEN);
            fb_text_cx(bx, bw, by + 26, "WIN", F12, C_WHITE);
        } else if (el < 1200) {
            fb_text_cx(bx, bw, by + 6, "难度提升", F12, C_YELLOW);
            char stars[16];
            int n = s_gs.difficulty < 10 ? s_gs.difficulty + 1 : 10;
            if (n > 15) n = 15;
            for (int i = 0; i < n; i++) stars[i] = '*';
            stars[n] = 0;
            fb_text_cx(bx, bw, by + 26, stars, F12, C_YELLOW);
        } else {
            char b[32];
            snprintf(b, sizeof(b), "第 %d 星 开新局", s_gs.difficulty < 10 ? s_gs.difficulty + 1 : 10);
            fb_text_cx(bx, bw, by + 16, b, F12, C_CYAN);
        }
    }

    // R 重开确认
    if (s_gs.flags1 & F1_RCONF) {
        const char* tip = "再按R确认重开";
        int tw = u8g2_str_width(F10B, tip) + 8;
        int bx = BSX + (BPD - tw) / 2;
        fb_fill(bx, BSY + 3, tw, 14, C_BLACK);
        fb_rect(bx, BSY + 3, tw, 14, C_RED);
        fb_text(F10B, bx + 4, BSY + 5, tip, C_RED);
    }

    // 回放横幅
    if (s_gs.flags0 & F0_REPLAY) {
        char rb[24];
        snprintf(rb, sizeof(rb), "回放 %d/%d", s_gs.replay_idx, s_gs.replay_total);
        int tw = u8g2_str_width(F10B, rb) + 8;
        int bx = BSX + (BPD - tw) / 2;
        fb_fill(bx, BSY + 3, tw, 14, C_BLACK);
        fb_rect(bx, BSY + 3, tw, 14, C_CYAN);
        fb_text(F10B, bx + 4, BSY + 5, rb, C_CYAN);
    }

    // 战报结算
    if (s_gs.flags0 & F0_RESULT) {
        int w = 280, h = 160, x = 20, y = 40;
        fb_fill(x, y, w, h, C_PANEL);
        fb_rect(x, y, w, h, C_WHITE);
        fb_hline(x + 2, y + 28, w - 4, C_TREND);

        const char* title = s_gs.winner == 0 ? "平局"
            : (s_gs.flags1 & F1_PVP) ? (s_gs.winner == 1 ? "黑方胜" : "白方胜")
            : (s_gs.winner == 1 ? "你赢了" : "你输了");
        fb_text_cx(x, w, y + 8, title, F14,
                   s_gs.winner == 0 ? C_YELLOW : (s_gs.winner == 1 ? C_GREEN : C_RED));

        char b[48];
        snprintf(b, sizeof(b), "手数 %d  用时 %d:%02d", s_gs.move_count, s_gs.game_sec / 60, s_gs.game_sec % 60);
        fb_text(F12, x + 10, y + 36, b, C_WHITE);
        snprintf(b, sizeof(b), "AI作弊 %d 次  最险 %d 步", s_gs.cheat, s_gs.min_danger == 255 ? 0 : s_gs.min_danger);
        fb_text(F12, x + 10, y + 54, b, C_WHITE);
        snprintf(b, sizeof(b), "连胜 %d   最高 %d", s_gs.streak, s_gs.best);
        fb_text(F12, x + 10, y + 72, b, C_WHITE);
        if (s_gs.worst_no > 0) snprintf(b, sizeof(b), "失误 %d 手  最大第 %d 手", s_gs.blunders, s_gs.worst_no);
        else snprintf(b, sizeof(b), "失误 0 手  全程无破绽");
        fb_text(F12, x + 10, y + 90, b, s_gs.blunders == 0 ? C_GREEN : C_LGRAY);

        static const char* kAchv[6] = { "首胜", "反杀", "屠龙", "三连", "零作弊", "无失误" };
        int ax = x + 10, ay = y + 116;
        int any = 0;
        for (int i = 0; i < 6; i++) {
            if (!(s_gs.achv_new & (1 << i))) continue;
            int tw = u8g2_str_width(F12, kAchv[i]);
            if (ax + tw > x + w - 10) { ax = x + 10; ay += 18; }
            fb_text(F12, ax, ay, kAchv[i], C_YELLOW);
            ax += tw + 10;
            any = 1;
        }
        if (!any) fb_text(F12, x + 10, y + 120, "R 重开  V 回放  L 难度", C_DGRAY);
    }

    // 帮助
    if (s_gs.flags0 & F0_HELP) {
        int w = 264, h = 150, x = 28, y = 45;
        fb_fill(x, y, w, h, C_BLACK);
        fb_rect(x, y, w, h, C_YELLOW);
        fb_text(F14, x + 10, y + 6, "按键帮助", C_YELLOW);
        static const char* kHelp[] = {
            "方向/回车  走子落子",
            "X 危险标记  K AI性格",
            "G 面板      L 难度",
            "M 辅助      U 悔棋",
            "P 双人对战  V 回放",
            "T 限时60秒  R 重开",
            "H 本帮助",
        };
        for (int i = 0; i < 7; i++)
            fb_text(F12, x + 10, y + 28 + i * 16, kHelp[i], C_WHITE);
    }
}

// 底部信息带: 左侧胜率条(左栏下方) + 棋盘下方事件日志
// 只在联机对战时显示(用户要求), 平时留给 AI 的误/险/成就
static void draw_bottom_band(bool pvp_mode)
{
    if (!pvp_mode) return;

    const int wx = LX, ww = 62;
    int32_t s1 = s_gs.score1 > 0 ? s_gs.score1 : 0;
    int32_t s2 = s_gs.score2 > 0 ? s_gs.score2 : 0;
    int split = (int)((int64_t)(s1 + 1) * ww / (s1 + s2 + 2));
    int pct = (int)((int64_t)(s1 + 1) * 100 / (s1 + s2 + 2));

    fb_text(F10B, wx, 203, "胜率", C_DGRAY);
    fb_fill(wx, 214, ww, 5, C_DGRAY);
    fb_fill(wx, 214, split, 5, C_GREEN);
    fb_fill(wx + split, 214, ww - split, 5, C_RED);
    char buf[12];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    fb_text(F10B, wx, 222, buf, C_LGRAY);

    // 棋盘正下方: 事件日志(只有联机对战有远端事件)
    if (s_net_flags & 0x01) fb_text(F10B, BSX, 214, s_evt, C_LGRAY);
}

// 顶部滚动横幅: 对方开启作弊辅助时, 文字从左边缘进入, 一直向右滑出再重来
static void draw_cheat_banner(void)
{
    if (!(s_gs.flags1 & F1_CHEAT)) return;
    const int h = 13;
    fb_fill(0, 0, LCD_W, h, C_BLACK);
    const char* t = "对方玩家正在启用作弊模式  ★  ";
    int tw = u8g2_str_width(F10B, t);
    if (tw <= 0) return;
    // 单向循环: x 从 -tw(完全在左边看不见) 走到 LCD_W(完全滑出右边), 再回到 -tw
    int span = LCD_W + tw;
    int off = (int)((now_ms() / 22) % (uint32_t)span);
    int x = -tw + off;
    fb_text(F10B, x, 1, t, C_YELLOW);
}

static void render_pong(void);
static void render_appinfo(void);

static void render_state(void)
{
    // 多游戏分派: Pong / app图标+名字 / 五子棋
    if (s_game_id == GID_PONG) { render_pong(); return; }
    if (s_game_id != GID_GOMOKU) { render_appinfo(); return; }

    extern uint32_t g_render_cnt;
    g_render_cnt++;
    static int n = 0;
    if (++n == 1 || n % 100 == 0) {
        ESP_LOGI(TAG, "render #%d board_r0=%d,%d,%d mv=%d over=%d",
                 n, s_gs.board[7][6], s_gs.board[7][7], s_gs.board[7][8],
                 s_gs.move_count, s_gs.flags0 & F0_OVER);
    }
    s_last_render = now_ms();
    fb_fill(0, 0, LCD_W, LCD_H, C_BG);

    // 棋盘底 + 网格
    fb_fill(BSX, BSY, BPD, BPD, C_WOOD);
    for (int i = 0; i < 15; i++) {
        fb_hline(BSX + CELL / 2, BSY + i * CELL + CELL / 2, BPD - CELL, C_BLACK);
        fb_vline(BSX + i * CELL + CELL / 2, BSY + CELL / 2, BPD - CELL, C_BLACK);
    }

    // 棋子
    for (int r = 0; r < 15; r++) {
        for (int c = 0; c < 15; c++) {
            int cx = BSX + c * CELL + CELL / 2;
            int cy = BSY + r * CELL + CELL / 2;
            if (s_gs.board[r][c] == 1) {
                fb_disc(cx, cy, CELL / 2 - 1, C_BLACK);
            } else if (s_gs.board[r][c] == 2) {
                fb_disc(cx, cy, CELL / 2 - 1, C_WHITE);
                fb_circle(cx, cy, CELL / 2 - 1, C_BLACK);
            }
        }
    }

    // 威胁标记(红=AI成五 橙=AI成四 绿框=你成五)
    if ((s_gs.flags0 & F0_THREAT) && !(s_gs.flags0 & F0_REPLAY)) {
        for (int r = 0; r < 15; r++) {
            for (int c = 0; c < 15; c++) {
                if (s_gs.board[r][c] || s_gs.threat[r][c] == 0) continue;
                int mx = BSX + c * CELL + CELL / 2;
                int my = BSY + r * CELL + CELL / 2;
                if (s_gs.threat[r][c] == 3) fb_fill(mx - 3, my - 3, 7, 7, C_RED);
                else if (s_gs.threat[r][c] == 2) fb_fill(mx - 3, my - 3, 6, 6, C_ORANGE);
                else fb_rect(mx - 3, my - 3, 7, 7, C_GREEN);
            }
        }
    }

    // 最后一步高亮
    if (s_gs.last_r < 15 && s_gs.last_c < 15) {
        fb_rect(BSX + s_gs.last_c * CELL, BSY + s_gs.last_r * CELL, CELL, CELL, C_CYAN);
    }

    // 作弊提示点: 手柄方(本机)开了作弊, 主机算好的落点用黄圈标出来
    if ((s_gs.flags1 & F1_PVP) && s_cheat_mine && s_gs.hint_r < 15 && s_gs.hint_c < 15 &&
        s_gs.board[s_gs.hint_r][s_gs.hint_c] == 0) {
        int hx = BSX + s_gs.hint_c * CELL + CELL / 2;
        int hy = BSY + s_gs.hint_r * CELL + CELL / 2;
        fb_circle(hx, hy, CELL / 2 - 1, C_YELLOW);
        fb_circle(hx, hy, CELL / 2 - 3, C_YELLOW);
    }

    // 光标(回放时不画)
    if (!(s_gs.flags0 & F0_REPLAY)) {
        int cx = BSX + s_gs.cursor_x * CELL + CELL / 2;
        int cy = BSY + s_gs.cursor_y * CELL + CELL / 2;
        fb_rect(cx - CELL / 2, cy - CELL / 2, CELL, CELL, C_RED);
    }

    // ---------- 左栏 ----------
    char b[40];
    fb_text(F12, LX, 4, "五子棋", C_CYAN);

    bool pvp_mode = (s_gs.flags1 & F1_PVP) != 0;

    if (pvp_mode) {
        // 双人对战: 单人那套 Lv/星级/YOU/AI/WIN 全部隐藏
        fb_text(F12, LX, 20, "PVP", C_MAGENTA);
        fb_text(F12, LX, 38, (s_gs.flags0 & F0_OVER)
                    ? (s_gs.winner == 0 ? "平局" : (s_gs.winner == 1 ? "黑胜" : "白胜"))
                    : (s_gs.cur_player == 1 ? "黑方" : "白方"),
                C_WHITE);
        if (s_net_flags & 0x01) {
            char pb[24];
            if (!(s_net_flags & 0x08)) {
                snprintf(pb, sizeof(pb), "我%s 他%s", (s_net_flags & 0x04) ? "OK" : "--",
                         (s_net_flags & 0x02) ? "OK" : "--");
                fb_text(F10B, LX, 58, pb, ((s_net_flags & 0x06) == 0x06) ? C_GREEN : C_YELLOW);
                fb_text(F10B, LX, 72, "A:准备 X:骂", C_DGRAY);
            } else {
                fb_text(F10B, LX, 58, (s_cur_player == 2) ? ">>> 我方" : "对方回合",
                        (s_cur_player == 2) ? C_GREEN : C_LGRAY);
                fb_text(F10B, LX, 72, "X:嘲讽", C_MAGENTA);
            }
        } else {
            fb_text(F10B, LX, 58, "同机双人", C_DGRAY);
        }
        // 作弊辅助: 本机(手柄方)开关状态
        fb_text(F10B, LX, 88, s_cheat_mine ? "作弊中" : "Y:作弊",
                s_cheat_mine ? C_YELLOW : C_DGRAY);
    } else {
        snprintf(b, sizeof(b), "Lv.%d", s_gs.difficulty);
        fb_text(F12, LX, 20, b, C_WHITE);

        for (int row = 0; row < 2; row++) {
            char st[6];
            for (int i = 0; i < 5; i++) st[i] = (row * 5 + i < s_gs.difficulty) ? '*' : ' ';
            st[5] = 0;
            fb_text(F10B, LX, 36 + row * 14, st, C_YELLOW);
        }

        int total = s_gs.pw + s_gs.aw;
        int ai_rate = total > 0 ? s_gs.aw * 100 / total : 0;
        snprintf(b, sizeof(b), "YOU%2d", s_gs.pw);
        fb_text(F12, LX, 66, b, C_GREEN);
        snprintf(b, sizeof(b), "AI %2d", s_gs.aw);
        fb_text(F12, LX, 82, b, C_ORANGE);
        snprintf(b, sizeof(b), "WIN%2d%%", ai_rate);
        fb_text(F12, LX, 98, b, C_RED);

        const char* st;
        if (s_gs.flags0 & F0_REPLAY) st = "REPLAY";
        else if (s_gs.flags0 & F0_OVER) {
            if (s_gs.winner == 0) st = "DRAW";
            else if (s_gs.flags1 & F1_ASSIST) st = s_gs.winner == 2 ? "AI WIN" : "OPP WIN";
            else st = s_gs.winner == 1 ? "YOU WIN" : "AI WIN";
        } else {
            if (s_gs.flags1 & F1_ASSIST) st = s_gs.cur_player == 1 ? "INPUT P1" : "THINK..";
            else st = s_gs.cur_player == 1 ? "YOURTURN" : "THINK..";
        }
        fb_text(F12, LX, 114, st, C_WHITE);
        fb_text(F12, LX, 128, (s_gs.flags1 & F1_ASSIST) ? "Assist" : "Normal", C_LGRAY);
    }

    // ---------- 左下角密集状态区 ----------
    int hy = 166;
    fb_text(F10B, LX, hy, "[H]帮助", C_YELLOW);

    if (!pvp_mode) {
        snprintf(b, sizeof(b), "连胜%d 最%d", s_gs.streak, s_gs.best);
        fb_text(F10B, LX, hy + 13, b, s_gs.streak > 0 ? C_GREEN : C_DGRAY);
    }

    if (s_gs.flags1 & F1_BLITZ) {
        int cs = (s_gs.flags0 & F0_OVER) ? 0 : (s_gs.cur_player == 1 ? s_gs.clk1 : s_gs.clk2);
        snprintf(b, sizeof(b), "CLK %d", cs);
        fb_text(F10B, LX, hy + 26, b, (!(s_gs.flags0 & F0_OVER) && cs < 10) ? C_RED : C_CYAN);
    } else {
        snprintf(b, sizeof(b), "T %d:%02d", s_gs.game_sec / 60, s_gs.game_sec % 60);
        fb_text(F10B, LX, hy + 26, b, C_CYAN);
    }

    // AI 的误/险只在单人模式显示
    if (!pvp_mode) {
        snprintf(b, sizeof(b), "误%d 险%d", s_gs.blunders, s_gs.min_danger == 255 ? 0 : s_gs.min_danger);
        fb_text(F10B, LX, hy + 39, b, C_LGRAY);
    }

    // ---------- 右栏 ----------
    int ty0 = 10;
    if (s_gs.flags0 & F0_PANEL) {
        draw_panel();
        ty0 = 160;   // 面板占 y4..154, 嘲讽从 160 开始(避免压住胜率条)
    }

    // 嘲讽(胜利动画时不画; 自己发出去的嘲讽不显示在自己屏幕上)
    if ((s_gs.flags0 & F0_TAUNT) && !(s_gs.flags0 & F0_ANIM) &&
        !(s_gs.flags1 & F1_TAUNT_REMOTE)) {
        color_t tc = s_gs.taunt_kind == 4 ? C_CYAN
            : (s_gs.taunt_kind == 2 ? C_MAGENTA : (s_gs.taunt_kind == 1 ? C_ORANGE : C_RED));
        int nl = s_gs.taunt_kind == 3 ? 3 : 2;
        for (int i = 0; i < nl; i++) {
            if (!s_gs.taunt[i][0]) break;
            fb_text_cx(RX, RW, ty0 + i * 16, s_gs.taunt[i], F14, tc);
        }
    }

    // 右下角: 作弊提示(该下哪)
    if ((s_gs.flags1 & F1_PVP) && s_cheat_mine) {
        char hb[16];
        if (s_gs.hint_r < 15 && s_gs.hint_c < 15)
            snprintf(hb, sizeof(hb), "下 %c%d", 'A' + s_gs.hint_c, s_gs.hint_r + 1);
        else
            snprintf(hb, sizeof(hb), "等对方");
        fb_text(F12, RX, 192, hb, C_YELLOW);
    }

    draw_overlays();
    draw_bottom_band(pvp_mode);
    draw_signal_right(318, 204);
    draw_link_stats();
    draw_batt_row();
    draw_cheat_banner();
    toast_draw();
    push_screen();
}

// ================= Pong / app 图标画面 =================
// 图标与 Cardputer 同一份(40x40 RGB565, gomoku_c3/main/icons.c)
extern const uint16_t icon_generic_generated[1600];
extern const uint16_t icon_gomoku_generated[1600];
extern const uint16_t icon_pong_generated[1600];
static const uint16_t* const kIcons[3] = {
    icon_generic_generated, icon_gomoku_generated, icon_pong_generated
};

// 图标 565 色 → 帧缓冲调色板索引(先精确匹配, 再最近色)
static color_t pal_of(uint16_t c)
{
    for (int i = 0; i < 18; i++) if (kPal[i] == c) return (color_t)i;
    int best = 0, bd = 1 << 30;
    int r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
    for (int i = 0; i < 18; i++) {
        uint16_t p = kPal[i];
        int pr = (p >> 11) & 31, pg = (p >> 5) & 63, pb = p & 31;
        int d = (r - pr) * (r - pr) + (g - pg) * (g - pg) + (b - pb) * (b - pb);
        if (d < bd) { bd = d; best = i; }
    }
    return (color_t)best;
}

// 40x40 图标放大 2 倍画进帧缓冲(80x80)
static void fb_icon2x(int x, int y, const uint16_t* img)
{
    for (int j = 0; j < 40; j++)
        for (int i = 0; i < 40; i++)
            fb_fill(x + i * 2, y + j * 2, 2, 2, pal_of(img[j * 40 + i]));
}

// 非游戏 app(桌面等): 图标 + 名字
static void render_appinfo(void)
{
    s_last_render = now_ms();
    fb_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    fb_icon2x((LCD_W - 80) / 2, 36, kIcons[s_icon_id < 3 ? s_icon_id : 0]);
    fb_text_cx(0, LCD_W, 136, s_app_name, F14, C_CYAN);
    fb_text_cx(0, LCD_W, 166, "已连接 Cardputer", F12, C_DGRAY);
    toast_draw();
    push_screen();
}

// Pong: 逻辑 240x135 → 320x180 等比映射(y+30 居中, 上下黑条放比分/状态)
static void render_pong(void)
{
    s_last_render = now_ms();
    fb_fill(0, 0, LCD_W, LCD_H, C_BLACK);

    #define POY 30
    fb_rect(0, POY, LCD_W, 180, C_DGRAY);
    for (int y = POY + 4; y < POY + 176; y += 12) fb_vline(160, y, 7, C_DGRAY);

    // 挡板 4x20 → 5x27, 联机时右挡板(手柄方)青色
    fb_fill(5, POY + s_pong.p1y * 4 / 3, 5, 27, C_WHITE);
    fb_fill(310, POY + s_pong.p2y * 4 / 3, 5, 27, (s_pong.flags & 1) ? C_CYAN : C_WHITE);

    // 球 4x4 → 5x5
    if (s_pong.flags & 2) // 游戏中才画球
        fb_fill(s_pong.ball_x * 4 / 3, POY + s_pong.ball_y * 4 / 3, 5, 5, C_WHITE);

    // 顶部比分
    char sc[16];
    snprintf(sc, sizeof(sc), "%d : %d", s_pong.score1, s_pong.score2);
    fb_text_cx(0, LCD_W, 7, sc, F14, C_WHITE);

    // 底部状态行
    if (!(s_pong.flags & 2)) {
        fb_text_cx(0, LCD_W, 216, "PONG - 选择模式中", F12, C_GRAY);
    } else if (s_pong.flags & 4) {
        const char* wt;
        if (s_pong.flags & 1) wt = (s_pong.winner == 1) ? "键盘方获胜! Enter/A 重开" : "手柄方获胜! Enter/A 重开";
        else wt = (s_pong.winner == 1) ? "你赢了! Enter 重开" : "AI 获胜! Enter 重开";
        fb_text_cx(0, LCD_W, 216, wt, F12, C_YELLOW);
    } else {
        fb_text_cx(0, LCD_W, 216, (s_pong.flags & 1) ? "键盘(左) vs 手柄(右)" : "单人 vs AI", F12, C_GRAY);
    }
    toast_draw();
    push_screen();
}

// ================= 界面状态 =================
static int s_settings = 0;      // 设置菜单开着
static int s_sel = 0;           // 菜单选中项
static int s_bl_on = 1;
static uint32_t s_search_anim = 0;

#define MENU_N 4

static const char* xbox_state_text(void)
{
    switch (xbox_state()) {
    case XBOX_ST_SCANNING: return "搜寻中";
    case XBOX_ST_FOUND:    return "已找到";
    case XBOX_ST_CONNECTING: return "连接中";
    case XBOX_ST_CONNECTED:  return "已连接";
    case XBOX_ST_FAILED:   return "失败";
    default:               return "未连接";
    }
}

static color_t xbox_state_color(void)
{
    switch (xbox_state()) {
    case XBOX_ST_CONNECTED: return C_GREEN;
    case XBOX_ST_CONNECTING:
    case XBOX_ST_FOUND:     return C_CYAN;
    case XBOX_ST_SCANNING:  return C_YELLOW;
    case XBOX_ST_FAILED:    return C_RED;
    default:                return C_DGRAY;
    }
}

static void draw_status(void)
{
    fb_fill(0, 0, LCD_W, LCD_H, C_BLACK);

    // 标题
    fb_text_cx(0, LCD_W, 34, "五子棋监视器", F14, C_CYAN);
    fb_fill(120, 54, 80, 1, C_TREND);

    // 主机连接状态框
    int bx = 40, bw = LCD_W - 80, by = 70, bh = 54;
    fb_rect(bx, by, bw, bh, s_linked ? C_GREEN : C_DGRAY);
    if (s_linked) {
        fb_disc(bx + 14, by + 18, 4, C_GREEN);
        fb_text(F12, bx + 26, by + 10, "已连接 Cardputer", C_GREEN);
        fb_text(F12, bx + 26, by + 30, view_ready() ? "画面实时镜像中" : "等待画面数据...", C_GRAY);
    } else {
        fb_disc(bx + 14, by + 18, 4, C_YELLOW);
        fb_text(F12, bx + 26, by + 10, "正在搜寻 Cardputer", C_YELLOW);
        int nd = (int)((now_ms() / 400) % 4);
        char dots[5] = "    ";
        for (int i = 0; i < nd; i++) dots[i] = '.';
        fb_text(F12, bx + 26, by + 30, dots, C_YELLOW);
    }

    // Xbox 手柄状态
    char xb[40];
    // 只加密上但一条报告都没收到时, 老实显示"无数据", 别骗人说连好了
    const char* stx = (xbox_state() == XBOX_ST_CONNECTED && xbox_get_input()->reports == 0)
                          ? "已连接·无数据" : xbox_state_text();
    snprintf(xb, sizeof(xb), "Xbox 手柄: %s", stx);
    fb_text_cx(0, LCD_W, 142, xb, F12, xbox_state_color());
    if (xbox_is_connected()) {
        if (xbox_battery() == 255)
            snprintf(xb, sizeof(xb), "电量 --  按键数 %u", (unsigned)(xbox_get_input()->reports & 0xFFFF));
        else
            snprintf(xb, sizeof(xb), "电量 %d%%  按键数 %u", xbox_battery(),
                     (unsigned)(xbox_get_input()->reports & 0xFFFF));
        fb_text_cx(0, LCD_W, 160, xb, F10B, C_LGRAY);
        // 实时按键: 按下去这里立刻变(用来确认链路真的通了)
        const xbox_input_t* in = xbox_get_input();
        char kb[48] = "";
        if (in->a) strcat(kb, "A ");
        if (in->b) strcat(kb, "B ");
        if (in->x) strcat(kb, "X ");
        if (in->y) strcat(kb, "Y ");
        if (in->lb) strcat(kb, "LB ");
        if (in->rb) strcat(kb, "RB ");
        if (in->up) strcat(kb, "上");
        if (in->down) strcat(kb, "下");
        if (in->left) strcat(kb, "左");
        if (in->right) strcat(kb, "右");
        if (in->menu) strcat(kb, "菜单 ");
        if (in->view) strcat(kb, "视图 ");
        fb_text_cx(0, LCD_W, 176, kb[0] ? kb : "按键: 无输入", F10B, kb[0] ? C_GREEN : C_DGRAY);
    } else if (xbox_need_pair_button()) {
        fb_text_cx(0, LCD_W, 160, "手柄灯快闪: 长按配对键 3 秒", F12, C_YELLOW);
    } else if (xbox_has_saved()) {
        fb_text_cx(0, LCD_W, 160, "已记住配对, 手柄开机自动连", F10B, C_DGRAY);
    }

    fb_text_cx(0, LCD_W, 192, "短按按键: 打开设置", F12, C_GRAY);

    draw_signal_right(318, 204);
    draw_link_stats();

    draw_batt_row();
    toast_draw();
    push_screen();
}

static void draw_settings(void)
{
    fb_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    fb_text_cx(0, LCD_W, 18, "设置", F14, C_CYAN);
    fb_fill(120, 40, 80, 1, C_TREND);

    char xb[40];
    snprintf(xb, sizeof(xb), "Xbox: %s", xbox_state_text());
    const char* items[MENU_N] = {
        "重新搜索主机",
        xb,
        s_bl_on ? "背光: 开" : "背光: 关",
        "返回",
    };
    for (int i = 0; i < MENU_N; i++) {
        int y = 62 + i * 34;
        bool sel = (i == s_sel);
        if (sel) {
            fb_fill(36, y - 1, LCD_W - 72, 26, C_SELBG);
            fb_rect(36, y - 1, LCD_W - 72, 26, C_YELLOW);
            fb_fill(36, y - 1, 3, 26, C_YELLOW);
        }
        color_t fg = sel ? C_YELLOW : C_WHITE;
        if (i == 1) fg = sel ? C_YELLOW : xbox_state_color();
        fb_text_cx(0, LCD_W, y + 4, items[i], F12, fg);
    }

    fb_text_cx(0, LCD_W, 206, "短按: 下一项   双击: 确认", F10B, C_GRAY);
    fb_text_cx(0, LCD_W, 222, "三击: 返回", F10B, C_DGRAY);
    toast_draw();
    push_screen();
}

static void leave_menu(void)
{
    s_settings = 0;
    if (s_linked && view_ready()) render_state();
    else draw_status();
}

static void settings_action(void)
{
    if (s_sel == 0) {
        // 重新搜索主机
        if (s_linked) {
            esp_now_send(s_host, (const uint8_t*)"GMKBYE", 6);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        esp_now_del_peer(s_host);
        s_linked = 0;
        s_have_state = 0;
        s_pong_have = 0;
        s_settings = 0;
        draw_status();
    } else if (s_sel == 1) {
        // Xbox 手柄: 未连 → 开始搜寻配对; 已连 → 断开
        if (xbox_is_connected()) {
            xbox_disconnect();
            toast_set("已断开 Xbox 手柄", C_ORANGE, 1500);
        } else {
            toast_set("正在搜寻 Xbox 手柄...", C_YELLOW, 0);
            xbox_start_pairing();
        }
        draw_settings();
    } else if (s_sel == 2) {
        s_bl_on = !s_bl_on;
        gpio_set_level(PIN_BL, s_bl_on);
        draw_settings();
    } else {
        leave_menu();
    }
}

// ---------------- 按键 ----------------
static int btn_down = 0;
static uint32_t btn_press_ms = 0;
static int tap_count = 0;
static uint32_t tap_deadline = 0;

static void poll_button(void)
{
    int lv = gpio_get_level(PIN_BTN);
    uint32_t now = now_ms();
    {
        static int last_lv = 1;
        if (lv != last_lv) {
            last_lv = lv;
            ESP_LOGI(TAG, "btn level=%d", lv);
        }
    }
    if (!btn_down && lv == 0) {
        btn_down = 1;
        btn_press_ms = now;
    } else if (btn_down && lv == 1) {
        btn_down = 0;
        if (now - btn_press_ms < 500) {
            tap_count++;
            tap_deadline = now + 350;
        }
    }
    if (tap_count > 0 && now > tap_deadline) {
        if (!s_settings) {
            // 任何短按: 打开设置
            s_settings = 1;
            s_sel = 0;
            draw_settings();
        } else {
            if (tap_count == 1) {
                s_sel = (s_sel + 1) % MENU_N;
                draw_settings();
            } else if (tap_count == 2) {
                settings_action();
            } else {
                leave_menu();
            }
        }
        tap_count = 0;
    }
}

static void splash(color_t c, const char* tag_text)
{
    fb_fill(0, 0, LCD_W, LCD_H, c);
    fb_text_cx(0, LCD_W, 100, tag_text, F14, C_WHITE);
    push_screen();
    vTaskDelay(pdMS_TO_TICKS(900));
}

void app_main(void)
{
    gpio_config_t io = { 0 };
    io.pin_bit_mask = (1ULL << PIN_CS) | (1ULL << PIN_DC) | (1ULL << PIN_BL);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_DC, 1);
    gpio_set_level(PIN_BL, 0);

    spi_bus_config_t bus = { 0 };
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = PIN_MISO;
    bus.sclk_io_num = PIN_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = LCD_W * 8 * 2;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = { 0 };
    dev.clock_speed_hz = 40 * 1000 * 1000;
    dev.mode = 0;
    dev.spics_io_num = -1;
    dev.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    st7789_init();
    gpio_set_level(PIN_BL, 1);

    // 自检: 红屏一下, 确认屏幕通路正常
    splash(C_BG, "五子棋监视器");

    gpio_config_t bin = { 0 };
    bin.pin_bit_mask = (1ULL << PIN_BTN);
    bin.mode = GPIO_MODE_INPUT;
    bin.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&bin);

    draw_status();
    batt_init();
    wifi_espnow_init();
    xbox_init();
    // 配对过就按地址直连; 没配对过则开机扫 30 秒, 手柄进配对模式即可连上
    toast_set(xbox_has_saved() ? "正在自动连接 Xbox 手柄..." : "正在搜寻 Xbox 手柄...",
              C_YELLOW, 0);
    xbox_start_pairing();
    uint32_t boot_scan_at = now_ms();
    draw_status();

    ESP_LOGI(TAG, "monitor start, free heap=%d", (int)esp_get_free_heap_size());

    xbox_state_t prev_xbox = xbox_state();
    uint32_t batt_last = 0;

    while (1) {
        poll_button();

        // 诊断心跳: 主循环是否还在跑
        {
            static uint32_t hb = 0;
            if (now_ms() - hb > 2000) {
                hb = now_ms();
                int fkhz = 0;
                spi_device_get_actual_freq(s_spi, &fkhz);
                ESP_LOGI(TAG, "hb heap=%d link=%d pkt=%u xb=%d btn=%d push=%u rnd=%u err=%u spi=%dkHz rep=%u",
                         (int)esp_get_free_heap_size(), s_linked,
                         (unsigned)s_last_pkt_ms, (int)xbox_state(), gpio_get_level(PIN_BTN),
                         (unsigned)g_push_cnt, (unsigned)g_render_cnt,
                         (unsigned)g_push_err, fkhz,
                         (unsigned)xbox_get_input()->reports);
            }
        }

        // 连接状态变化: 刷新状态页
        {
            static int prev_linked = 0;
            if (s_linked != prev_linked) {
                prev_linked = s_linked;
                if (!s_linked) { s_have_state = 0; s_pong_have = 0; }
                if (!s_settings) draw_status();
            }
        }

        // 每秒回执, 让主机用收包超时快速判定掉线
        {
            static uint32_t ack_last = 0;
            if (s_linked && now_ms() - ack_last > 1000) {
                ack_last = now_ms();
                esp_now_send(s_host, (const uint8_t*)"GMKACK", 6);
            }
        }

        // 手柄 → 主机: 按键掩码(变化才发) + 手柄在线位 + 嘲讽快捷键
        {
            static uint16_t prev_mask = 0;
            static uint32_t btn_last = 0;
            static uint8_t prev_f = 0xFF;
            static uint32_t f_last = 0;
            static uint16_t prev_tb = 0;

            uint16_t m = 0;
            if (xbox_is_connected()) {
                const xbox_input_t* in = xbox_get_input();
                if (in->up) m |= 0x0001;
                if (in->down) m |= 0x0002;
                if (in->left) m |= 0x0004;
                if (in->right) m |= 0x0008;
                if (in->a) m |= 0x0010;
                if (in->b) m |= 0x0020;
                if (in->x) m |= 0x0040;
                if (in->y) m |= 0x0080;
                if (in->lb) m |= 0x0100;
                if (in->rb) m |= 0x0200;
                if (in->lt > 0x200) m |= 0x0400;
                if (in->rt > 0x200) m |= 0x0800;
                if (in->menu) m |= 0x1000;
                if (in->view) m |= 0x2000;
            }

            if (s_linked) {
                // 手柄 Y: 本机(手柄方)作弊辅助开关, 和键盘方的 X 各自独立(仅五子棋)
                static uint16_t prev_y = 0;
                uint16_t yb = (uint16_t)(m & 0x0080);
                if (yb && !prev_y && s_game_id == GID_GOMOKU) {
                    s_cheat_mine = !s_cheat_mine;
                    toast_set(s_cheat_mine ? "作弊辅助: 开" : "作弊辅助: 关",
                              s_cheat_mine ? C_YELLOW : C_DGRAY, 1200);
                }
                prev_y = yb;

                // 还没开局就按方向键 → 提示先准备(仅五子棋; 玩家会以为是方向键坏了)
                static uint16_t prev_dir = 0;
                uint16_t dir = (uint16_t)(m & 0x000F);
                if (dir && !(prev_dir & dir) && s_game_id == GID_GOMOKU && s_have_state &&
                    (s_net_flags & 0x01) && !(s_net_flags & 0x08)) {
                    toast_set("当前还未准备, 请按 A 准备后再动", C_YELLOW, 1600);
                }
                prev_dir = dir;

                if (m != prev_mask && now_ms() - btn_last > 30) {
                    uint8_t pkt[8];
                    memcpy(pkt, "GMKBTN", 6);
                    pkt[6] = (uint8_t)(m & 0xFF);
                    pkt[7] = (uint8_t)(m >> 8);
                    esp_now_send(s_host, pkt, sizeof(pkt));
                    prev_mask = m;
                    btn_last = now_ms();
                }
                // bit0=手柄在线  bit1=手柄方作弊辅助开启
                uint8_t f = (uint8_t)((xbox_is_connected() ? 1 : 0) | (s_cheat_mine ? 2 : 0));
                if (f != prev_f || now_ms() - f_last > 1000) {
                    uint8_t pkt[7];
                    memcpy(pkt, "GMKPVP", 6);
                    pkt[6] = f;
                    esp_now_send(s_host, pkt, sizeof(pkt));
                    prev_f = f;
                    f_last = now_ms();
                }
                // 嘲讽: X / LB / RB 按下沿
                uint16_t tb = (uint16_t)(m & 0x0340);
                if (tb && !(prev_tb & tb))
                    esp_now_send(s_host, (const uint8_t*)"GMKTAU", 6);
                prev_tb = tb;
            } else {
                prev_mask = m;
                prev_f = 0xFF;
                prev_tb = 0;
                s_cheat_mine = 0;
            }
        }

        // 电量每 5 秒读一次, 有变化就刷新
        if (now_ms() - batt_last > 5000) {
            batt_last = now_ms();
            int old = s_batt_pct;
            batt_update();
            if (s_batt_pct != old) {
                if (!s_settings) {
                    if (s_linked && view_ready()) render_state();
                    else draw_status();
                } else {
                    draw_settings();
                }
            }
        }

        // Xbox 状态变化: 连贯提示
        {
            xbox_state_t st = xbox_state();
            if (st != prev_xbox) {
                switch (st) {
                case XBOX_ST_SCANNING:
                    if (prev_xbox != XBOX_ST_IDLE) toast_set("正在搜寻 Xbox 手柄...", C_YELLOW, 0);
                    break;
                case XBOX_ST_FOUND:
                    toast_set("已搜索到 Xbox 手柄", C_CYAN, 900);
                    toast_push("正在尝试连接...", C_ORANGE, 0);
                    break;
                case XBOX_ST_CONNECTED:
                    toast_set("已连接上 Xbox 手柄", C_GREEN, 2000);
                    break;
                case XBOX_ST_FAILED:
                    toast_set("连接失败, 请让手柄进入配对模式", C_RED, 2500);
                    break;
                default:
                    break;
                }
                prev_xbox = st;
                if (!s_settings) {
                    if (s_linked && view_ready()) render_state();
                    else draw_status();
                } else {
                    draw_settings();
                }
            }
        }

        // 手柄按键实时反馈: 按下去屏幕立刻弹名字, 证明链路真的通了
        if (xbox_is_connected()) {
            const xbox_input_t* in = xbox_get_input();
            static uint32_t s_prev_mask = 0;
            uint32_t mask = 0;
            if (in->a) mask |= 1u << 0;
            if (in->b) mask |= 1u << 1;
            if (in->x) mask |= 1u << 2;
            if (in->y) mask |= 1u << 3;
            if (in->lb) mask |= 1u << 4;
            if (in->rb) mask |= 1u << 5;
            if (in->up) mask |= 1u << 6;
            if (in->down) mask |= 1u << 7;
            if (in->left) mask |= 1u << 8;
            if (in->right) mask |= 1u << 9;
            if (in->menu) mask |= 1u << 10;
            if (in->view) mask |= 1u << 11;
            uint32_t nu = mask & ~s_prev_mask;
            s_prev_mask = mask;
            // 联机对战时不弹按键提示, 免得盖住事件日志
            if (nu && !(s_net_flags & 0x01)) {
                char tb[24] = "手柄 ";
                const char* nm = "";
                if (nu & (1u << 0)) nm = "A";
                else if (nu & (1u << 1)) nm = "B";
                else if (nu & (1u << 2)) nm = "X";
                else if (nu & (1u << 3)) nm = "Y";
                else if (nu & (1u << 4)) nm = "LB";
                else if (nu & (1u << 5)) nm = "RB";
                else if (nu & (1u << 6)) nm = "上";
                else if (nu & (1u << 7)) nm = "下";
                else if (nu & (1u << 8)) nm = "左";
                else if (nu & (1u << 9)) nm = "右";
                else if (nu & (1u << 10)) nm = "菜单";
                else if (nu & (1u << 11)) nm = "视图";
                strcat(tb, nm);
                toast_set(tb, C_GREEN, 600);
                if (!s_settings) {
                    if (s_linked && view_ready()) render_state();
                    else draw_status();
                } else {
                    draw_settings();
                }
            }
        }

        // 看到手柄但广播不可连接: 提示长按配对键(只提示一次)
        {
            static int pair_btn_tip = 0;
            if (xbox_need_pair_button() && !xbox_is_connected()) {
                if (!pair_btn_tip) {
                    pair_btn_tip = 1;
                    toast_set("检测到手柄, 请长按配对键", C_ORANGE, 4000);
                    if (!s_settings) {
                        if (s_linked && view_ready()) render_state();
                        else draw_status();
                    } else {
                        draw_settings();
                    }
                }
            } else {
                pair_btn_tip = 0;
            }
        }

        // 开机扫描超时就停, 省电(可在设置里重试); 已看到手柄时多给 2 分钟
        if (boot_scan_at) {
            uint32_t limit = xbox_need_pair_button() ? 120000 : 30000;
            if (xbox_is_connected()) {
                boot_scan_at = 0;
            } else if (now_ms() - boot_scan_at > limit) {
                boot_scan_at = 0;
                xbox_disconnect();
                toast_set("未发现 Xbox 手柄, 可在设置中重试", C_DGRAY, 2500);
                if (!s_settings) {
                    if (s_linked && view_ready()) render_state();
                    else draw_status();
                } else {
                    draw_settings();
                }
            }
        }

        // 提示队列推进
        if (toast_tick()) {
            if (!s_settings) {
                if (s_linked && view_ready()) render_state();
                else draw_status();
            } else {
                draw_settings();
            }
        }

        // 新状态包渲染
        if (s_gs_ready) {
            s_gs_ready = 0;
            if (s_linked && !s_settings) render_state();
        }

        // 胜利动画: 本地 130ms 刷新复刻阶段切换(仅五子棋)
        if (s_linked && !s_settings && s_game_id == GID_GOMOKU && s_have_state && (s_gs.flags0 & F0_ANIM)
            && now_ms() - s_last_render > 130) {
            render_state();
        }

        // 作弊横幅滚动: 80ms 重绘一次(仅五子棋)
        if (s_linked && !s_settings && s_game_id == GID_GOMOKU && s_have_state && (s_gs.flags1 & F1_CHEAT)
            && now_ms() - s_last_render > 80) {
            render_state();
        }

        // 掉线检测(3 秒无包)
        if (s_linked && now_ms() - s_last_pkt_ms > 3000) {
            s_linked = 0;
            s_have_state = 0;
            s_pong_have = 0;
            if (!s_settings) draw_status();
        }

        // 搜寻/等待动画: 未连上或还没收到画面数据时持续刷新, 避免看起来像卡死
        if ((!s_linked || !view_ready()) && !s_settings && now_ms() - s_search_anim > 400) {
            s_search_anim = now_ms();
            draw_status();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
