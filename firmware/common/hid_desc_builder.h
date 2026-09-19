// hid_desc_builder.h — 受信機の HID レポートディスクリプタを実行時に組み立てる
//
// 元のパッドに在った軸・ボタンだけを同じ Usage で並べる。
// joydev は ABS_* コードの昇順に軸番号を振るので、軸の集合が一致していれば
// /dev/input/js0 の軸番号・ボタン番号が元のパッドと一致する。
//
// レポートの中身（レポート ID なし）
//   [軸 int8 × 軸数（X,Y,Z,Rx,Ry,Rz のうち在るものだけ）]
//   [ハット u8（0〜7 = 上から時計回り、0x0F = 中立）]   ※ハットが在るときだけ
//   [ボタン ceil(n/8) バイト（bit0 = ボタン 1）]
#pragma once
#include <stdint.h>
#include "packet.h"

// ディスクリプタの作り方を変えたら増やす（受信機の bcdDevice に載り、
// Windows が古いディスクリプタをキャッシュしたままになるのを防ぐ）
#define WG_HID_DESC_FORMAT 1

#define WG_HID_DESC_MAX  96
#define WG_HID_REPORT_MAX 12

struct WgHidLayout {
  uint8_t axisMask;
  uint8_t axisCount;
  uint8_t buttonCount;
  uint8_t hat;
  uint8_t reportLen;
};

static inline uint8_t *wgDescItem(uint8_t *p, uint8_t prefix, uint32_t v, uint8_t n) {
  *p++ = (uint8_t)(prefix | n);
  for (uint8_t i = 0; i < n; i++) *p++ = (uint8_t)(v >> (8 * i));
  return p;
}

// 戻り値: ディスクリプタの長さ（0 = 失敗）
static inline uint16_t wgBuildHidDescriptor(uint8_t *buf, uint16_t cap,
                                            uint8_t axisMask, uint8_t buttonCount,
                                            bool hat, WgHidLayout *layout) {
  if (buttonCount > WG_BUTTON_MAX) buttonCount = WG_BUTTON_MAX;
  axisMask &= 0x3F;
  uint8_t axisCount = 0;
  for (int a = 0; a < WG_AXIS_COUNT; a++) if (axisMask & (1u << a)) axisCount++;
  if (axisCount == 0 && buttonCount == 0 && !hat) return 0;
  if (cap < WG_HID_DESC_MAX) return 0;

  uint8_t *p = buf;
  p = wgDescItem(p, 0x04, 0x01, 1);            // Usage Page (Generic Desktop)
  p = wgDescItem(p, 0x08, 0x05, 1);            // Usage (Game Pad)
  p = wgDescItem(p, 0xA0, 0x01, 1);            // Collection (Application)

  if (axisCount) {
    p = wgDescItem(p, 0x04, 0x01, 1);          // Usage Page (Generic Desktop)
    for (int a = 0; a < WG_AXIS_COUNT; a++)
      if (axisMask & (1u << a)) p = wgDescItem(p, 0x08, (uint32_t)(0x30 + a), 1);
    p = wgDescItem(p, 0x14, 0x81, 1);          // Logical Minimum (-127)
    p = wgDescItem(p, 0x24, 0x7F, 1);          // Logical Maximum (127)
    p = wgDescItem(p, 0x74, 8, 1);             // Report Size (8)
    p = wgDescItem(p, 0x94, axisCount, 1);     // Report Count
    p = wgDescItem(p, 0x80, 0x02, 1);          // Input (Data, Var, Abs)
  }

  if (hat) {
    // 実機のゲームパッド（元の F310 を含む）と同じ「論理値 0〜7 ＋ Null state」の形。
    // 範囲外の値（ここでは 0x0F）が中立を表す。TinyUSB のサンプルの
    // 「1〜8、Null 指定なし」より広く受け入れられる。
    p = wgDescItem(p, 0x04, 0x01, 1);          // Usage Page (Generic Desktop)
    p = wgDescItem(p, 0x08, 0x39, 1);          // Usage (Hat switch)
    p = wgDescItem(p, 0x14, 0, 1);             // Logical Minimum (0)
    p = wgDescItem(p, 0x24, 7, 1);             // Logical Maximum (7)
    p = wgDescItem(p, 0x34, 0, 1);             // Physical Minimum (0)
    p = wgDescItem(p, 0x44, 315, 2);           // Physical Maximum (315)
    p = wgDescItem(p, 0x64, 0x14, 1);          // Unit (degree)
    p = wgDescItem(p, 0x74, 8, 1);             // Report Size (8)
    p = wgDescItem(p, 0x94, 1, 1);             // Report Count (1)
    p = wgDescItem(p, 0x80, 0x42, 1);          // Input (Data, Var, Abs, Null state)
    // ハットで設定した物理値・単位をボタン定義に持ち越さないよう戻す
    p = wgDescItem(p, 0x34, 0, 1);             // Physical Minimum (0)
    p = wgDescItem(p, 0x44, 0, 1);             // Physical Maximum (0)
    p = wgDescItem(p, 0x64, 0, 1);             // Unit (None)
  }

  if (buttonCount) {
    p = wgDescItem(p, 0x04, 0x09, 1);          // Usage Page (Button)
    p = wgDescItem(p, 0x18, 1, 1);             // Usage Minimum (1)
    p = wgDescItem(p, 0x28, buttonCount, 1);   // Usage Maximum (n)
    p = wgDescItem(p, 0x14, 0, 1);             // Logical Minimum (0)
    p = wgDescItem(p, 0x24, 1, 1);             // Logical Maximum (1)
    p = wgDescItem(p, 0x74, 1, 1);             // Report Size (1)
    p = wgDescItem(p, 0x94, buttonCount, 1);   // Report Count (n)
    p = wgDescItem(p, 0x80, 0x02, 1);          // Input (Data, Var, Abs)
    uint8_t pad = (uint8_t)((8 - (buttonCount % 8)) % 8);
    if (pad) {
      p = wgDescItem(p, 0x74, 1, 1);           // Report Size (1)
      p = wgDescItem(p, 0x94, pad, 1);         // Report Count
      p = wgDescItem(p, 0x80, 0x03, 1);        // Input (Const, Var, Abs) = 詰め物
    }
  }

  *p++ = 0xC0;                                  // End Collection

  if (layout) {
    layout->axisMask    = axisMask;
    layout->axisCount   = axisCount;
    layout->buttonCount = buttonCount;
    layout->hat         = hat ? 1 : 0;
    layout->reportLen   = (uint8_t)(axisCount + (hat ? 1 : 0) + (buttonCount + 7) / 8);
  }
  return (uint16_t)(p - buf);
}

// WgState から HID レポートを組み立てる。戻り値はレポート長
static inline uint8_t wgBuildHidReport(const WgHidLayout *L, const WgState *s, uint8_t *out) {
  uint8_t n = 0;
  for (int a = 0; a < WG_AXIS_COUNT; a++) {
    if (!(L->axisMask & (1u << a))) continue;
    int v = (int)s->axis[a] - 128;
    if (v < -127) v = -127;
    if (v >  127) v =  127;
    out[n++] = (uint8_t)(int8_t)v;
  }
  if (L->hat) out[n++] = (s->hat <= 7) ? s->hat : 0x0F;   // 範囲外 = 中立（Null state）
  uint8_t nb = (uint8_t)((L->buttonCount + 7) / 8);
  for (uint8_t i = 0; i < nb; i++) out[n++] = (uint8_t)(s->buttons >> (8 * i));
  return n;
}
