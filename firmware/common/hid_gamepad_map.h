// hid_gamepad_map.h — HID レポートディスクリプタを解析して、パッドの生レポートから
// WgState を取り出すためのマップを作る（送信機で使う）
//
// VID:PID ごとの決め打ちマッピング表ではなく、ディスクリプタを読んで
// Usage（X, Y, Z, Rx, Ry, Rz, Hat switch, Button 1..n）の位置を求める。
// Linux の hid-generic も同じ Usage を見て ABS_* / BTN_* に割り当てるので、
// これで元のパッドと同じ軸番号・ボタン番号になる。
#pragma once
#include <stdint.h>
#include <string.h>
#include "packet.h"

#define WG_HID_MAX_CAND 4      // 同時に見るレポート ID の数

struct WgHidField {
  uint8_t  present;
  uint16_t bitOffset;          // レポート先頭（ID バイトを除く）からのビット位置
  uint8_t  bitSize;
  int32_t  lmin, lmax;
};

struct WgHidMap {
  uint8_t    valid;            // X 軸が見つかっていれば 1
  uint8_t    reportId;         // 0 = レポート ID なし
  uint8_t    gamepadApp;       // Joystick / GamePad コレクション内なら 1
  uint16_t   reportBits;
  WgHidField axis[WG_AXIS_COUNT];
  WgHidField hat;
  uint16_t   buttonOffset;     // ボタン 1 のビット位置
  uint8_t    buttonCount;
  uint16_t   vid, pid;
};

// ---- ディスクリプタ解析 ----

static inline int32_t wgHidItemVal(const uint8_t *p, uint8_t size, bool isSigned) {
  switch (size) {
    case 1: return isSigned ? (int32_t)(int8_t)p[0] : (int32_t)p[0];
    case 2: {
      uint16_t u = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
      return isSigned ? (int32_t)(int16_t)u : (int32_t)u;
    }
    case 4:
      return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
    default: return 0;
  }
}

static inline void wgHidAssign(WgHidMap *m, uint32_t usage, uint16_t bitOff,
                               uint8_t bitSize, int32_t lmin, int32_t lmax) {
  uint16_t page = (uint16_t)(usage >> 16);
  uint16_t id   = (uint16_t)(usage & 0xFFFF);
  if (page == 0x01) {                       // Generic Desktop
    if (id >= 0x30 && id <= 0x35) {         // X, Y, Z, Rx, Ry, Rz
      WgHidField *f = &m->axis[id - 0x30];
      if (f->present) return;               // 最初に出てきたものを使う
      f->present = 1; f->bitOffset = bitOff; f->bitSize = bitSize;
      f->lmin = lmin; f->lmax = lmax;
      if (id == 0x30) m->valid = 1;
    } else if (id == 0x39) {                // Hat switch
      if (m->hat.present) return;
      m->hat.present = 1; m->hat.bitOffset = bitOff; m->hat.bitSize = bitSize;
      m->hat.lmin = lmin; m->hat.lmax = lmax;
    }
  } else if (page == 0x09) {                // Button
    if (id >= 1 && id <= WG_BUTTON_MAX && bitSize == 1) {
      if (m->buttonCount == 0) {
        if (bitOff < (id - 1)) return;                  // 下溢れ（不正な並び）は無視
        m->buttonOffset = (uint16_t)(bitOff - (id - 1));
      }
      if (id > m->buttonCount) m->buttonCount = (uint8_t)id;
    }
  }
}

// 戻り値: 解析できたら true（out に最良のレポートのマップが入る）
static inline bool wgHidParseDescriptor(const uint8_t *d, uint16_t len, WgHidMap *out) {
  WgHidMap cand[WG_HID_MAX_CAND];
  uint16_t cursor[WG_HID_MAX_CAND];
  uint8_t  nCand = 0;
  memset(cand, 0, sizeof(cand));
  memset(cursor, 0, sizeof(cursor));

  uint16_t usagePage = 0;
  int32_t  lmin = 0, lmax = 0;
  uint32_t reportSize = 0, reportCount = 0;
  uint8_t  reportId = 0;
  uint32_t usages[32]; uint8_t nUsages = 0;
  uint32_t usageMin = 0, usageMax = 0; bool haveRange = false;
  uint32_t application = 0;
  uint8_t  depth = 0;

  uint16_t i = 0;
  while (i < len) {
    uint8_t b = d[i++];
    if (b == 0xFE) {                        // long item（使われないが読み飛ばす）
      if (i + 1 >= len) break;
      uint8_t sz = d[i]; i += (uint16_t)(2 + sz); continue;
    }
    uint8_t size = b & 0x03; if (size == 3) size = 4;
    uint8_t type = (uint8_t)((b >> 2) & 0x03);
    uint8_t tag  = (uint8_t)((b >> 4) & 0x0F);
    if (i + size > len) break;
    const uint8_t *data = d + i;
    i += size;

    // bType: 0 = Main, 1 = Global, 2 = Local
    if (type == 1) {                        // Global
      switch (tag) {
        case 0: usagePage  = (uint16_t)wgHidItemVal(data, size, false); break;
        case 1: lmin       = wgHidItemVal(data, size, true);  break;
        case 2: lmax       = wgHidItemVal(data, size, true);
                if (lmax < lmin) lmax = wgHidItemVal(data, size, false);  // 0x25 0xFF 対策
                break;
        case 7: reportSize = (uint32_t)wgHidItemVal(data, size, false);
                if (reportSize > 32) reportSize = 0;      // 不正なディスクリプタ対策
                break;
        case 8: reportId   = (uint8_t)wgHidItemVal(data, size, false);   break;
        case 9: reportCount= (uint32_t)wgHidItemVal(data, size, false);
                if (reportCount > 256) reportCount = 0;   // 4G 回ループして core1 を殺さない
                break;
        default: break;                     // Push/Pop・単位系は未対応（ゲームパッドでは使われない）
      }
    } else if (type == 2) {                 // Local
      uint32_t u = (uint32_t)wgHidItemVal(data, size, false);
      if (size < 4) u |= ((uint32_t)usagePage << 16);
      switch (tag) {
        case 0: if (nUsages < 32) usages[nUsages++] = u; break;
        case 1: usageMin = u; haveRange = true; break;
        case 2: usageMax = u; break;
        default: break;
      }
    } else if (type == 0) {                 // Main
      if (tag == 10) {                      // Collection
        uint8_t ctype = size ? data[0] : 0;
        if (ctype == 0x01 && nUsages) application = usages[0];
        depth++;
      } else if (tag == 12) {               // End Collection
        if (depth) depth--;
        if (!depth) application = 0;
      } else if (tag == 8) {                // Input
        uint8_t flags = size ? data[0] : 0;
        // 対象のレポート ID のスロットを確保
        int slot = -1;
        for (uint8_t k = 0; k < nCand; k++) if (cand[k].reportId == reportId) { slot = k; break; }
        if (slot < 0 && nCand < WG_HID_MAX_CAND) {
          slot = nCand++;
          cand[slot].reportId = reportId;
        }
        if (slot >= 0) {
          WgHidMap *m = &cand[slot];
          if (application == 0x00010004u || application == 0x00010005u) m->gamepadApp = 1;
          uint16_t off = cursor[slot];
          bool isConst = (flags & 0x01) != 0;
          bool isVar   = (flags & 0x02) != 0;
          if (reportSize == 0 || reportCount == 0) {
            // 不正な項目。ビット位置だけ進めずに読み飛ばす
            nUsages = 0; haveRange = false; usageMin = usageMax = 0;
            continue;
          }
          for (uint32_t k = 0; k < reportCount; k++) {
            if (!isConst && isVar) {
              uint32_t u = 0;
              if (nUsages) u = usages[k < nUsages ? k : (uint32_t)(nUsages - 1)];
              else if (haveRange) u = usageMin + k;
              if (u) wgHidAssign(m, u, (uint16_t)(off + k * reportSize),
                                 (uint8_t)reportSize, lmin, lmax);
            }
          }
          uint32_t next = (uint32_t)off + reportCount * reportSize;
          if (next > 0xFFFF) next = 0xFFFF;               // 桁あふれ防止
          cursor[slot] = (uint16_t)next;
          m->reportBits = cursor[slot];
        }
      }
      nUsages = 0; haveRange = false; usageMin = usageMax = 0;   // ローカル項目をクリア
    }
  }

  // 一番それらしいレポートを選ぶ
  int best = -1, bestScore = -1;
  for (uint8_t k = 0; k < nCand; k++) {
    if (!cand[k].valid && cand[k].buttonCount == 0) continue;
    int score = cand[k].buttonCount + (cand[k].valid ? 32 : 0) +
                (cand[k].gamepadApp ? 64 : 0) + (cand[k].hat.present ? 8 : 0);
    if (score > bestScore) { bestScore = score; best = k; }
  }
  if (best < 0) return false;
  *out = cand[best];
  out->valid = 1;
  return true;
}

// ---- レポートの取り出し ----

static inline int32_t wgHidReadBits(const uint8_t *r, uint16_t len,
                                    uint16_t off, uint8_t size, bool isSigned) {
  uint32_t v = 0;
  for (uint8_t k = 0; k < size && k < 32; k++) {
    uint16_t bit = (uint16_t)(off + k);
    if ((uint16_t)(bit >> 3) >= len) break;
    if (r[bit >> 3] & (1u << (bit & 7))) v |= (1ul << k);
  }
  if (isSigned && size < 32 && (v & (1ul << (size - 1)))) v |= ~((1ul << size) - 1);
  return (int32_t)v;
}

// フィールドがレポート内に収まっているか
static inline bool wgHidFieldFits(const WgHidField *f, uint16_t len) {
  return ((uint32_t)f->bitOffset + f->bitSize) <= (uint32_t)len * 8;
}

static inline uint8_t wgHidScaleAxis(const WgHidField *f, const uint8_t *r, uint16_t len) {
  int32_t range = f->lmax - f->lmin;
  if (range <= 0) return WG_AXIS_CENTER;
  if (!wgHidFieldFits(f, len)) return WG_AXIS_CENTER;   // 短いレポートを全開と誤読しない
  int32_t v = wgHidReadBits(r, len, f->bitOffset, f->bitSize, f->lmin < 0);
  if (v < f->lmin) v = f->lmin;
  if (v > f->lmax) v = f->lmax;
  return (uint8_t)(((int64_t)(v - f->lmin) * 255) / range);
}

// report は「レポート ID を除いた」データ部分
// 戻り値 false = レポートが宣言より短く信用できない（呼び出し側は中立＋未接続扱いにする）
//
// ⚠ 範囲外のビットを 0 として読むと、軸は「全開の片側」、ハットは「上」に化ける。
//   転送の途中終了や短縮レポートでロボットが暴走しうるので、必ず長さを検証する。
static inline bool wgHidExtract(const WgHidMap *m, const uint8_t *report, uint16_t len,
                                WgState *s) {
  if ((uint32_t)len * 8 < m->reportBits) { wgStateNeutral(s); return false; }
  for (int a = 0; a < WG_AXIS_COUNT; a++) {
    s->axis[a] = m->axis[a].present ? wgHidScaleAxis(&m->axis[a], report, len)
                                    : WG_AXIS_CENTER;
  }
  s->hat = WG_HAT_NEUTRAL;
  if (m->hat.present && wgHidFieldFits(&m->hat, len)) {
    int32_t v = wgHidReadBits(report, len, m->hat.bitOffset, m->hat.bitSize,
                              m->hat.lmin < 0);
    int32_t dir = v - m->hat.lmin;
    if (dir >= 0 && dir <= 7) s->hat = (uint8_t)dir;
  }
  s->buttons = 0;
  for (uint8_t b = 0; b < m->buttonCount && b < WG_BUTTON_MAX; b++) {
    uint16_t bit = (uint16_t)(m->buttonOffset + b);
    if ((uint32_t)bit >= (uint32_t)len * 8) break;
    if (wgHidReadBits(report, len, bit, 1, false)) s->buttons |= (uint16_t)(1u << b);
  }
  return true;
}

static inline void wgHidToInfo(const WgHidMap *m, WgInfo *n) {
  n->axisMask = 0;
  for (int a = 0; a < WG_AXIS_COUNT; a++)
    if (m->axis[a].present) n->axisMask |= (uint8_t)(1u << a);
  n->buttonCount = m->buttonCount > WG_BUTTON_MAX ? WG_BUTTON_MAX : m->buttonCount;
  n->caps = m->hat.present ? WG_CAP_HAT : 0;
  n->vid = m->vid;
  n->pid = m->pid;
}
