/*
 * u8g2 RLE 字体解码器(独立移植自 M5GFX/lgfx 的 U8g2font)
 * 支持 u8g2 bdfconv 生成的 C 数组字体(如 efontCN)
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t w, h;    // glyph 位图尺寸
    int8_t xoff;     // 位图左边相对落笔点的偏移
    int8_t yoff;     // 位图底边相对 baseline 的偏移(向上为负)
    int8_t xadv;     // 落笔点步进
} u8g2_glyph_t;

// 解码一个 glyph。buf 为 NULL 时只取度量;否则解码位图(1=前景),
// buf 至少 w*h 字节。返回 0 成功,-1 字体里没有该字符。
int u8g2_decode(const uint8_t* font, uint32_t code, u8g2_glyph_t* g, uint8_t* buf);

// 字体行高相关
int u8g2_font_height(const uint8_t* font);   // max_char_height
int u8g2_font_baseline(const uint8_t* font); // max_char_height + y_offset

// UTF-8 字符串像素宽
int u8g2_str_width(const uint8_t* font, const char* s);

// 取 UTF-8 串下一个 codepoint,推进指针
uint32_t u8g2_utf8_next(const char** s);

#ifdef __cplusplus
}
#endif
