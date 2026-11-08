#include "xbox_ble.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "host/ble_store.h"

static const char* TAG = "xbox";

#define UUID_SVC_HID      0x1812
#define UUID_SVC_BATTERY  0x180F
#define UUID_CHR_REPORT   0x2A4D
#define UUID_CHR_BATTERY  0x2A19
#define XBOX_APPEARANCE   964

static volatile xbox_state_t s_state = XBOX_ST_IDLE;
static volatile bool s_enabled = false;
static volatile bool s_connected = false;
static volatile uint8_t s_battery = 255;

static xbox_input_t s_in;

static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static ble_addr_t s_peer;
static bool s_peer_valid = false;

// 服务/特征发现状态机
static uint16_t s_hid_s = 0, s_hid_e = 0;
static uint16_t s_bat_s = 0, s_bat_e = 0;
static int s_stage = 0;
// HID 里的辅助特征: 部分手柄要先读到报告表/设成 Report 协议才肯上报数据
static uint16_t s_map_val = 0;    // 0x2A4B Report Map
static uint16_t s_hinfo_val = 0;  // 0x2A4A HID Information
static uint16_t s_ctrl_val = 0;   // 0x2A4C HID Control Point
static uint16_t s_proto_val = 0;  // 0x2A4E Protocol Mode

// HID 服务里报告特征可能有好几个(输入/输出/feature), 全部记下来并订阅
#define MAX_REP 8
static uint16_t s_rep_vals[MAX_REP];
static uint8_t s_rep_props[MAX_REP];
static int s_rep_cnt = 0;
static int s_rep_idx = 0;
static uint16_t s_rep_wr = 0;   // 可写的报告特征(输出报告), 用来发握手包
static uint8_t s_hs = 0;        // 加密后握手进度: 0=等订阅 1=读HID信息 2=读报告表 3=写控制点 4=完成
static int s_enc_fail = 0;      // 连续加密/连接失败次数

static bool is_report_handle(uint16_t h)
{
    for (int i = 0; i < s_rep_cnt; i++)
        if (s_rep_vals[i] == h) return true;
    return false;
}

static int s_rep_off = -1;   // 报告负载偏移(首包自适应: 有 Report ID 时为 1)
static volatile int s_need_pair_btn = 0;
static volatile int s_sec_req = 0;   // 已发起过配对, 防止 CCCD 写失败后反复触发
static uint32_t s_conn_at = 0;       // 本次连接时刻(用于配对超时)
static volatile int s_fail_cnt = 0;  // 连续失败次数
static volatile int s_encrypted = 0; // 本次链路已加密(配对过)

static struct ble_npl_event s_cmd_ev;
static volatile int s_cmd = 0; // 1=开始配对 2=断开

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

// ---------------- NVS 记住配对地址 ----------------
static void load_saved(void)
{
    nvs_handle_t h;
    if (nvs_open("gmkmon", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t buf[8] = { 0 };
    size_t len = sizeof(buf);
    if (nvs_get_blob(h, "xbox", buf, &len) == ESP_OK && len == 8 && buf[7] == 1) {
        memcpy(s_peer.val, buf, 6);
        s_peer.type = buf[6];
        s_peer_valid = true;
    }
    nvs_close(h);
}

static void save_saved(void)
{
    nvs_handle_t h;
    if (nvs_open("gmkmon", NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t buf[8];
    memcpy(buf, s_peer.val, 6);
    buf[6] = s_peer.type;
    buf[7] = 1;
    nvs_set_blob(h, "xbox", buf, sizeof(buf));
    nvs_commit(h);
    nvs_close(h);
}

bool xbox_has_saved(void) { return s_peer_valid; }
bool xbox_need_pair_button(void) { return s_need_pair_btn != 0; }

// 失败不要放弃: 手柄经常要试好几次才连上, 累计几次后继续重试
static void note_failure(void)
{
    if (++s_fail_cnt >= 3) {
        s_fail_cnt = 0;
        ESP_LOGW(TAG, "连续 3 次失败, 继续自动重试");
    }
}

xbox_state_t xbox_state(void)
{
    uint32_t now = now_ms();
    if (s_connected) {
        // 连上了但 15 秒还没完成配对 → 主动断开重试, 避免界面卡在"连接中"
        // (要先给服务发现 + CCCD 订阅留出时间, 之前 5 秒太短)
        if (!s_encrypted && s_state == XBOX_ST_CONNECTING && now - s_conn_at > 15000) {
            s_conn_at = now;
            ESP_LOGW(TAG, "15s 未完成配对, 断开重试");
            note_failure();
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        // 加密完成但 12 秒还收不到任何输入报告 → 也是坏的, 断开重连
        else if (s_encrypted && s_state != XBOX_ST_CONNECTED && s_in.reports == 0 &&
                 now - s_conn_at > 12000) {
            s_conn_at = now;
            ESP_LOGW(TAG, "连接后无输入报告(12s), 断开重试");
            note_failure();
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return s_state;
}
bool xbox_is_connected(void) { return s_connected; }
uint8_t xbox_battery(void) { return s_battery; }
const xbox_input_t* xbox_get_input(void) { return &s_in; }

// ---------------- 报告解析(16B, 见 XboxControllerNotificationParser) ----------------
static void parse_report(const uint8_t* d, int len, int off)
{
    if (len < off + 16) return;
    const uint8_t* p = d + off;

    uint8_t dir = p[12];
    uint8_t mainb = p[13];
    uint8_t cen = p[14];
    uint8_t shr = p[15];

    s_in.a = mainb & 0x01;
    s_in.b = mainb & 0x02;
    s_in.x = mainb & 0x08;
    s_in.y = mainb & 0x10;
    s_in.lb = mainb & 0x40;
    s_in.rb = mainb & 0x80;

    s_in.view = cen & 0x04;
    s_in.menu = cen & 0x08;
    s_in.xbox = cen & 0x10;
    s_in.ls = cen & 0x20;
    s_in.rs = cen & 0x40;
    s_in.share = shr & 0x01;

    s_in.up = (dir == 1 || dir == 2 || dir == 8);
    s_in.right = (dir >= 2 && dir <= 4);
    s_in.down = (dir >= 4 && dir <= 6);
    s_in.left = (dir >= 6 && dir <= 8);

    s_in.lx = (uint16_t)(p[0] | (p[1] << 8));
    s_in.ly = (uint16_t)(p[2] | (p[3] << 8));
    s_in.rx = (uint16_t)(p[4] | (p[5] << 8));
    s_in.ry = (uint16_t)(p[6] | (p[7] << 8));
    s_in.lt = (uint16_t)(p[8] | (p[9] << 8));
    s_in.rt = (uint16_t)(p[10] | (p[11] << 8));
    s_in.last_ms = now_ms();
    s_in.reports++;
}

// ---------------- GATT 发现 ----------------
static int dsc_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc* dsc, void* arg);
static void disc_rep_dscs(uint16_t conn);
static void hs_next(uint16_t conn);
static void request_conn_params(uint16_t conn);

// 握手(串行, 一次只发一个 ATT 操作, 避免 ENOMEM / ATT 超时)
// ATT 回 0x05/0x0E/0x0F(需要认证/授权/加密) 时才是发起配对的正确时机
static void maybe_secure(uint16_t conn, int status)
{
    int att = status & 0xFF;
    if ((att == 0x05 || att == 0x0E || att == 0x0F) && !s_sec_req) {
        s_sec_req = 1;
        int rc = ble_gap_security_initiate(conn);
        ESP_LOGI(TAG, "ATT 需要加密(0x%02x), 发起配对 rc=%d", att, rc);
    }
}

static int hs_read_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                      struct ble_gatt_attr* attr, void* arg)
{
    int step = (int)(intptr_t)arg;
    ESP_LOGI(TAG, "hs read step=%d status=%d len=%d", step, error->status,
             (attr && attr->om) ? (int)OS_MBUF_PKTLEN(attr->om) : 0);
    if (error->status) {
        maybe_secure(conn_handle, error->status);
        // 需要加密: 停在这里等配对完成, 配对成功后 ENC_CHANGE 会重新发现并重开握手
        int att = error->status & 0xFF;
        if (att == 0x05 || att == 0x0E || att == 0x0F) return 0;
    }
    s_hs = (uint8_t)(step + 1);
    hs_next(conn_handle);
    return 0;
}

static void hs_next(uint16_t conn)
{
    while (1) {
        if (s_hs == 1) {
            if (s_hinfo_val) {
                int rc = ble_gattc_read(conn, s_hinfo_val, hs_read_cb, (void*)(intptr_t)1);
                if (rc == 0) return;
                ESP_LOGW(TAG, "读 HID 信息 rc=%d", rc);
            }
            s_hs = 2;
        } else if (s_hs == 2) {
            if (s_map_val) {
                int rc = ble_gattc_read(conn, s_map_val, hs_read_cb, (void*)(intptr_t)2);
                if (rc == 0) return;
                ESP_LOGW(TAG, "读报告表 rc=%d", rc);
            }
            s_hs = 3;
        } else if (s_hs == 3) {
            // 不写 HID Control Point(0x2A4C): 实测这个写手柄不应答, ATT 挂满 30 秒
            // 超时(status=13)后手柄直接断链(disconnected reason=534)。
            // 输入报告在读完报告表之后就已经正常上报, 这一步是多余的。
            s_hs = 4;
        } else {
            ESP_LOGI(TAG, "握手完成, 等输入报告");
            s_stage = 0;
            return;
        }
    }
}

// CCCD 写结果: 如果服务端要求加密(0x05/0x0F), 这时才发起配对
static int cccd_wr_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                      struct ble_gatt_attr* attr, void* arg)
{
    (void)attr;
    int is_report = (int)(intptr_t)arg;
    if (error->status == 0) {
        ESP_LOGI(TAG, "cccd subscribed%s", is_report ? " (报告)" : "");
        // 注意: 这里不能算"已连接"。订阅成功不等于收到数据,
        // 必须在真正收到输入报告后才置 CONNECTED(见 NOTIFY_RX)。
        if (is_report && s_hs == 0) {
            s_hs = 1;
            hs_next(conn_handle);
        }
        return 0;
    }
    int att = error->status & 0xFF;
    ESP_LOGW(TAG, "cccd write status=%d", error->status);
    if ((att == 0x05 || att == 0x0F) && !s_sec_req) {
        s_sec_req = 1;
        int rc = ble_gap_security_initiate(conn_handle);
        ESP_LOGI(TAG, "需要加密, 发起配对 rc=%d", rc);
    }
    return 0;
}

static int chr_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                  const struct ble_gatt_chr* chr, void* arg)
{
    if (error->status == 0 && chr) {
        if (s_stage == 2 && chr->uuid.u.type == BLE_UUID_TYPE_16) {
            uint16_t u = chr->uuid.u16.value;
            ESP_LOGI(TAG, "chr h=%d u=%04x p=%02x", chr->val_handle, u, chr->properties);
            if (u == UUID_CHR_REPORT) {
                if (s_rep_cnt < MAX_REP) {
                    s_rep_props[s_rep_cnt] = chr->properties;
                    s_rep_vals[s_rep_cnt++] = chr->val_handle;
                }
                if (!s_rep_wr && (chr->properties & BLE_GATT_CHR_PROP_WRITE))
                    s_rep_wr = chr->val_handle;
            } else if (u == 0x2A4B) {
                s_map_val = chr->val_handle;
            } else if (u == 0x2A4A) {
                s_hinfo_val = chr->val_handle;
            } else if (u == 0x2A4C) {
                s_ctrl_val = chr->val_handle;
            } else if (u == 0x2A4E) {
                s_proto_val = chr->val_handle;
            }
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (s_stage == 2) {
            s_stage = 3;
            if (s_rep_cnt) {
                ESP_LOGI(TAG, "报告特征 %d 个, 首个 val=%d props=%02x", s_rep_cnt,
                         s_rep_vals[0], s_rep_props[0]);
                s_rep_idx = 0;
                disc_rep_dscs(conn_handle);
                return 0;
            }
            s_stage = 0;
            ESP_LOGW(TAG, "未找到报告特征");
        }
    }
    return 0;
}

static int dsc_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc* dsc, void* arg)
{
    (void)arg;
    if (error->status == 0 && dsc) {
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16 && dsc->uuid.u16.value == 0x2902 &&
            is_report_handle(chr_val_handle)) {
            uint8_t v[2] = { 0x01, 0x00 };
            int rc = ble_gattc_write_flat(conn_handle, dsc->handle, v, sizeof(v),
                                          cccd_wr_cb, (void*)(intptr_t)1);
            ESP_LOGI(TAG, "subscribe val=%d cccd=%d rc=%d", chr_val_handle, dsc->handle, rc);
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        s_rep_idx++;
        disc_rep_dscs(conn_handle);
    }
    return 0;
}

static void disc_rep_dscs(uint16_t conn)
{
    if (s_rep_idx >= s_rep_cnt) {
        // 有报告特征但一个 CCCD 都没写出去时, 也要进握手
        if (s_hs == 0) {
            s_hs = 1;
            hs_next(conn);
        }
        return;
    }
    uint16_t val = s_rep_vals[s_rep_idx];
    uint16_t end = (s_rep_idx + 1 < s_rep_cnt) ? (uint16_t)(s_rep_vals[s_rep_idx + 1] - 2) : s_hid_e;
    if (end <= val) end = (uint16_t)(val + 1);
    ble_gattc_disc_all_dscs(conn, val, end, dsc_cb, NULL);
}

static int svc_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                  const struct ble_gatt_svc* svc, void* arg)
{
    if (error->status == 0 && svc) {
        if (svc->uuid.u.type == BLE_UUID_TYPE_16) {
            if (svc->uuid.u16.value == UUID_SVC_HID) { s_hid_s = svc->start_handle; s_hid_e = svc->end_handle; }
            else if (svc->uuid.u16.value == UUID_SVC_BATTERY) { s_bat_s = svc->start_handle; s_bat_e = svc->end_handle; }
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "svcs hid=%d-%d bat=%d-%d", s_hid_s, s_hid_e, s_bat_s, s_bat_e);
        if (s_hid_s) {
            s_stage = 2;
            ble_gattc_disc_all_chrs(conn_handle, s_hid_s, s_hid_e, chr_cb, NULL);
        } else {
            ESP_LOGW(TAG, "未找到 HID 服务");
        }
    }
    return 0;
}

static void discover(uint16_t conn)
{
    s_hid_s = s_hid_e = s_bat_s = s_bat_e = 0;
    s_rep_cnt = 0;
    s_rep_idx = 0;
    s_rep_wr = 0;
    s_hs = 0;
    s_map_val = s_hinfo_val = s_ctrl_val = s_proto_val = 0;
    s_stage = 1;
    ble_gattc_disc_all_svcs(conn, svc_cb, NULL);
}

// ---------------- 连接 ----------------
static void start_scan(void);

static int gap_cb(struct ble_gap_event* event, void* arg);

static void connect_to(const ble_addr_t* addr)
{
    // 默认参数的 supervision timeout 只有 2.56s, 和 WiFi/ESP-NOW 共存时
    // 很容易被挤掉 —— 日志里就是加密中 status=520(HCI 0x08 连接超时)。
    // 这里放宽到 6 秒, 并把发起扫描占空比降到 50%。
    struct ble_gap_conn_params cp;
    memset(&cp, 0, sizeof(cp));
    cp.scan_itvl = 0x20;              // 20ms
    cp.scan_window = 0x10;            // 10ms (50%)
    cp.itvl_min = 12;                 // 15ms
    cp.itvl_max = 24;                 // 30ms
    cp.latency = 0;
    cp.supervision_timeout = 600;     // 6s
    cp.min_ce_len = 0;
    cp.max_ce_len = 0;

    s_state = XBOX_ST_CONNECTING;
    int rc = ble_gap_connect(s_own_addr_type, addr, 8000, &cp, gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "connect rc=%d", rc);
        s_state = XBOX_ST_SCANNING;
    }
}

static bool adv_is_xbox(const struct ble_hs_adv_fields* f)
{
    if (f->appearance_is_present && f->appearance == XBOX_APPEARANCE) return true;
    if (f->name && f->name_len) {
        char n[33];
        int l = f->name_len > 32 ? 32 : f->name_len;
        memcpy(n, f->name, l);
        n[l] = 0;
        if (strstr(n, "Xbox")) return true;
    }
    if (f->mfg_data && f->mfg_data_len >= 3 && f->mfg_data[0] == 0x06 && f->mfg_data[1] == 0x00)
        return true;
    return false;
}

static void start_scan(void)
{
    if (!s_enabled) return;
    struct ble_gap_disc_params p;
    memset(&p, 0, sizeof(p));
    p.itvl = 0x40;     // 40ms 扫描间隔
    p.window = 0x30;   // 30ms 窗口(75% 占空比, 比原来的 50% 快很多)
    p.passive = 0;
    p.filter_duplicates = 0;
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &p, gap_cb, NULL);
    if (rc == 0) {
        s_state = XBOX_ST_SCANNING;
    } else if (rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "disc rc=%d", rc);
    }
}

static int gap_cb(struct ble_gap_event* event, void* arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields f;
        if (ble_hs_adv_parse_fields(&f, event->disc.data, event->disc.length_data) != 0) return 0;

        bool match;
        if (s_peer_valid) {
            match = (memcmp(event->disc.addr.val, s_peer.val, 6) == 0) || adv_is_xbox(&f);
        } else {
            match = adv_is_xbox(&f);
        }
        if (!match) return 0;

        // event_type=3 (ADV_NONCONN_IND): 手柄在广播但不可连接, 多半没进配对模式
        if (event->disc.event_type == 3) {
            if (!s_need_pair_btn) {
                ESP_LOGW(TAG, "手柄广播不可连接, 需要长按配对键");
            }
            s_need_pair_btn = 1;
            // 有些手柄广播类型标错/刚唤醒, 已配对过的地址每 6 秒硬试一次连接
            static uint32_t hard_try_ms = 0;
            if (s_peer_valid && now_ms() - hard_try_ms > 6000) {
                hard_try_ms = now_ms();
                ESP_LOGW(TAG, "硬试连接已配对地址");
                ble_gap_disc_cancel();
                connect_to(&event->disc.addr);
            }
            return 0;
        }
        s_need_pair_btn = 0;

        s_peer = event->disc.addr;
        s_peer_valid = true;
        ble_gap_disc_cancel();
        s_state = XBOX_ST_FOUND;
        {
            char nm[33] = "-";
            if (f.name && f.name_len) {
                int l = f.name_len > 32 ? 32 : f.name_len;
                memcpy(nm, f.name, l);
                nm[l] = 0;
            }
            ESP_LOGI(TAG, "found %02x:%02x:%02x:%02x:%02x:%02x rssi=%d type=%d evt=%d name=%s app=%d mfg_len=%d",
                     s_peer.val[5], s_peer.val[4], s_peer.val[3], s_peer.val[2], s_peer.val[1], s_peer.val[0],
                     event->disc.rssi, s_peer.type, event->disc.event_type, nm,
                     f.appearance_is_present ? f.appearance : -1,
                     f.mfg_data ? f.mfg_data_len : 0);
        }
        connect_to(&s_peer);
        return 0;
    }
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            s_connected = true;
            s_state = XBOX_ST_CONNECTING;  // 真正收到报告才算已连接
            s_need_pair_btn = 0;
            s_sec_req = 0;
            s_fail_cnt = 0;
            s_conn_at = now_ms();
            s_encrypted = 0;
            save_saved();
            ESP_LOGI(TAG, "connected handle=%d", s_conn);
            int mrc = ble_gattc_exchange_mtu(s_conn, NULL, NULL);
            struct ble_gap_conn_desc cd;
            if (ble_gap_conn_find(s_conn, &cd) == 0 && cd.sec_state.encrypted) {
                // 之前配对过, 链路直接就是加密的
                s_encrypted = 1;
                ESP_LOGI(TAG, "mtu rc=%d 链路已加密, 直接发现", mrc);
                discover(s_conn);
            } else {
                // 关键: Xbox 手柄是 HID 外设, 由它自己发 SMP Security Request, 主机被动响应。
                // 主机主动 ble_gap_security_initiate 会让手柄直接不理人 ——
                // 实测日志: 33001ms 发起配对, 手柄从此不再发任何包, 6s 后 supervision timeout(520)。
                // 所以这里只做服务发现, 等订阅 CCCD 报"需要加密"(0x05) 时再发起配对。
                ESP_LOGI(TAG, "mtu rc=%d 直接发现服务(等手柄发起配对)", mrc);
                discover(s_conn);
            }
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
            s_connected = false;
            s_state = XBOX_ST_FAILED;
            s_conn = BLE_HS_CONN_HANDLE_NONE;
            if (s_enabled) start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnected reason=%d", event->disconnect.reason);
        s_connected = false;
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_stage = 0;
        s_hs = 0;
        s_sec_req = 0;
        s_encrypted = 0;
        if (s_enabled) {
            s_state = XBOX_ST_SCANNING;
            start_scan();
        } else {
            s_state = (s_fail_cnt >= 3) ? XBOX_ST_FAILED : XBOX_ST_IDLE;
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // 手柄要求重新配对: 删掉旧密钥再继续
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        ESP_LOGW(TAG, "repeat pairing, 删除旧密钥重试");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "enc change status=%d", event->enc_change.status);
        if (event->enc_change.status == 0) {
            s_encrypted = 1;
            s_sec_req = 0;
            s_enc_fail = 0;
            struct ble_gap_conn_desc d2;
            if (ble_gap_conn_find(s_conn, &d2) == 0)
                ESP_LOGI(TAG, "加密完成 bonded=%d key_size=%d", d2.sec_state.bonded, d2.sec_state.key_size);
            request_conn_params(s_conn);
            discover(s_conn);  // 加密成功, 重新发现并订阅
        } else {
            ESP_LOGW(TAG, "配对失败 status=%d, 断开重试", event->enc_change.status);
            s_sec_req = 0;
            // 旧的配对密钥可能已失效(手柄被别的设备重配过) → 清掉, 重新配对
            if (++s_enc_fail >= 3) {
                s_enc_fail = 0;
                ble_store_clear();
                ESP_LOGW(TAG, "连续 %d 次失败, 已清除配对信息, 请长按手柄配对键重配", 3);
            }
            note_failure();
            ble_gap_terminate(s_conn, BLE_ERR_AUTH_FAIL);
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t ah = event->notify_rx.attr_handle;
        uint8_t buf[32];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) len = sizeof(buf);
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) != 0) return 0;

        // 诊断: 前 3 个通知不管哪个 handle 都打出来
        {
            static int s_any_n = 0;
            if (s_any_n < 3) {
                s_any_n++;
                ESP_LOGI(TAG, "notify#%d handle=%d len=%d", s_any_n, ah, len);
            }
        }

        if (s_rep_cnt && is_report_handle(ah)) {
            if (s_rep_off < 0) {
                s_rep_off = (len >= 17 && buf[0] == 0x01) ? 1 : 0;
                ESP_LOGI(TAG, "report len=%d off=%d", len, s_rep_off);
            }
            parse_report(buf, len, s_rep_off);
            if (s_in.reports <= 3) {
                ESP_LOGI(TAG, "rep%d len=%d raw=%02x %02x %02x %02x %02x %02x", (int)s_in.reports, len,
                         buf[0], buf[1], buf[2], buf[3], buf[4], len > 5 ? buf[5] : 0);
            }
            if (s_state != XBOX_ST_CONNECTED) {
                s_state = XBOX_ST_CONNECTED;
                save_saved();
                ESP_LOGI(TAG, "手柄数据已就绪");
            }
        } else if (s_bat_s && ah == s_bat_s) {
            // 电池服务暂不订阅
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_enabled && s_state == XBOX_ST_SCANNING) start_scan();
        return 0;

    default:
        return 0;
    }
}

// ---------------- 主机任务 ----------------
// 加密成功后把连接参数收紧一点(15ms + 6s 超时), 掉包容忍度更高
static void request_conn_params(uint16_t conn)
{
    struct ble_gap_upd_params p;
    memset(&p, 0, sizeof(p));
    p.itvl_min = 12;              // 15ms
    p.itvl_max = 12;
    p.latency = 0;
    p.supervision_timeout = 600;  // 6s
    int rc = ble_gap_update_params(conn, &p);
    ESP_LOGI(TAG, "请求连接参数 rc=%d", rc);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr rc=%d", rc);
        return;
    }
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    // 手柄天线小, 发射功率拉到最大(+9dBm)才够稳
    esp_err_t txrc = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    ESP_LOGI(TAG, "ble sync, addr_type=%d, saved=%d, txpwr rc=%d",
             s_own_addr_type, (int)s_peer_valid, (int)txrc);
    if (s_enabled) start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "ble host reset reason=%d", reason);
}

static void cmd_ev_fn(struct ble_npl_event* ev)
{
    (void)ev;
    int cmd = s_cmd;
    s_cmd = 0;
    if (cmd == 1) {
        s_enabled = true;
        s_fail_cnt = 0;
        if (s_connected) {
            discover(s_conn);
        } else {
            if (ble_gap_disc_active()) ble_gap_disc_cancel();
            start_scan();
        }
    } else if (cmd == 2) {
        s_enabled = false;
        if (ble_gap_disc_active()) ble_gap_disc_cancel();
        if (s_connected) ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        s_state = XBOX_ST_IDLE;
    }
}

static void host_task(void* param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void xbox_init(void)
{
    // 自己保证 NVS 可用(没有 WiFi 初始化时也要能记住配对)
    esp_err_t er = nvs_flash_init();
    if (er == ESP_ERR_NVS_NO_FREE_PAGES || er == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    load_saved();

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble init fail %d", err);
        return;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Xbox 手柄只支持 LE Legacy 配对(Just Works), 开 SC 会在密钥交换阶段
    // 被手柄晾着直到连接超时(HCI 0x08), 所以这里关掉 SC
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    // 主动把我们的 IRK 分发给手柄(很多 HID 外设靠它完成绑定),
    // 但不强求手柄的 ID 密钥(它不给, 要了会失败)
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_npl_event_init(&s_cmd_ev, cmd_ev_fn, NULL);

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "ble stack started, saved=%d", (int)s_peer_valid);
}

void xbox_start_pairing(void)
{
    s_cmd = 1;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_cmd_ev);
}

void xbox_disconnect(void)
{
    s_cmd = 2;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_cmd_ev);
}
