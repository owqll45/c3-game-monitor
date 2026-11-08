#include "u8g2font.h"
#include <string.h>

// ---- 字体头 ----
// [0]glyph_cnt [1]bbx_mode [2]bits0 [3]bits1
// [4]bits_w [5]bits_h [6]bits_x [7]bits_y [8]bits_delta
// [9]max_w [10]max_h [11]x_offset [12]y_offset
// [13]ascent_A [14]descent_g [15]ascent_para [16]descent_para
// [17:18]start_upper_A [19:20]start_lower_a [21:22]start_unicode

static uint16_t hdr_u16(const uint8_t* f, int i) { return ((uint16_t)f[i] << 8) | f[i + 1]; }

int u8g2_font_height(const uint8_t* font) { return (int8_t)font[10]; }
int u8g2_font_baseline(const uint8_t* font) { return (int8_t)font[10] + (int8_t)font[12]; }

// ---- 位流 ----
typedef struct { const uint8_t* p; uint8_t bit; } bitdec_t;

static uint32_t get_ubits(bitdec_t* d, int cnt)
{
    uint8_t bit_pos = d->bit;
    uint32_t val = d->p[0] >> bit_pos;
    int bp = bit_pos + cnt;
    if (bp >= 8) {
        bp -= 8;
        val |= (uint32_t)(*++d->p) << (8 - bit_pos);
    }
    d->bit = (uint8_t)bp;
    return val & ((1u << cnt) - 1);
}

static int get_sbits(bitdec_t* d, int cnt)
{
    return (int)get_ubits(d, cnt) - (1 << (cnt - 1));
}

// ---- glyph 查找 ----
static const uint8_t* find_glyph(const uint8_t* font, uint32_t enc)
{
    const uint8_t* f = font + 23;
    if (enc <= 255) {
        if (enc >= 'a') f += hdr_u16(font, 19);
        else if (enc >= 'A') f += hdr_u16(font, 17);
        for (; f[1]; f += f[1])
            if (f[0] == enc) return f + 2;
    } else {
        f += hdr_u16(font, 21);
        const uint8_t* lut = f;
        uint16_t e;
        do {
            f += ((uint16_t)lut[0] << 8) + lut[1];
            e = ((uint16_t)lut[2] << 8) + lut[3];
            lut += 4;
        } while (e < enc && e != 0);
        for (; (e = ((uint16_t)f[0] << 8) + f[1]) != 0; f += f[2])
            if (e == enc) return f + 3;
    }
    return NULL;
}

int u8g2_decode(const uint8_t* font, uint32_t code, u8g2_glyph_t* g, uint8_t* buf)
{
    const uint8_t* d = find_glyph(font, code);
    if (!d) {
        // 找不到时用空格度量(宽度 0)
        g->w = g->h = 0; g->xoff = g->yoff = 0; g->xadv = 0;
        return -1;
    }
    bitdec_t bd = { d, 0 };
    int w = (int)get_ubits(&bd, font[4]);
    int h = (int)get_ubits(&bd, font[5]);
    g->xoff = (int8_t)get_sbits(&bd, font[6]);
    g->yoff = (int8_t)get_sbits(&bd, font[7]);
    g->xadv = (int8_t)get_sbits(&bd, font[8]);
    g->w = (uint8_t)w;
    g->h = (uint8_t)h;

    if (!buf || w <= 0 || h <= 0) return 0;
    memset(buf, 0, (size_t)w * h);

    int bits0 = font[2], bits1 = font[3];
    int lx = 0, ly = 0;
    int guard = 0;
    do {
        uint32_t ab[2];
        ab[0] = get_ubits(&bd, bits0);
        ab[1] = get_ubits(&bd, bits1);
        int i = 0;
        for (;;) {
            uint32_t length = ab[i];
            while (length) {
                int len = (length > (uint32_t)(w - lx)) ? (w - lx) : (int)length;
                length -= len;
                if (i) memset(buf + ly * w + lx, 1, len);
                lx += len;
                if (lx >= w) { lx = 0; ly++; if (ly >= h) break; }
            }
            if (ly >= h) break;
            if (i) {
                if (!get_ubits(&bd, 1)) break;
                i = 0;
                if (++guard > 10000) return 0; // 防畸形数据死循环
            } else {
                i = 1;
            }
        }
    } while (ly < h);
    return 0;
}

uint32_t u8g2_utf8_next(const char** s)
{
    const uint8_t* p = (const uint8_t*)*s;
    uint32_t c;
    if (p[0] < 0x80) { c = p[0]; p += 1; }
    else if ((p[0] & 0xE0) == 0xC0) { c = ((uint32_t)(p[0] & 0x1F) << 6) | (p[1] & 0x3F); p += 2; }
    else if ((p[0] & 0xF0) == 0xE0) { c = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F); p += 3; }
    else { c = ((uint32_t)(p[0] & 0x07) << 18) | ((uint32_t)(p[1] & 0x3F) << 12) | ((uint32_t)(p[2] & 0x3F) << 6) | (p[3] & 0x3F); p += 4; }
    *s = (const char*)p;
    return c;
}

int u8g2_str_width(const uint8_t* font, const char* s)
{
    int x = 0;
    u8g2_glyph_t g;
    while (*s) {
        uint32_t cp = u8g2_utf8_next(&s);
        if (u8g2_decode(font, cp, &g, NULL) == 0) x += g.xadv;
    }
    return x;
}
