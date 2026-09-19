// packet.h — 送信機・受信機で共有する無線パケット定義
//
// パケットは長さで種別を見分ける（TXDA のペイロード長）。
//   12 バイト: State  … パッドの状態。30〜100ms ごとに送る
//   10 バイト: Info   … パッドの構成（軸の種類・ボタン数）。接続時と 2 秒ごとに送る
//
// 軸の並びについて（重要）
//   Linux の joydev は ABS_* コードの昇順に js の軸番号を振る。
//   ABS_X(0) ABS_Y(1) ABS_Z(2) ABS_RX(3) ABS_RY(4) ABS_RZ(5) ABS_HAT0X(16) ABS_HAT0Y(17)
//   そこでパケットの軸スロットも HID Usage（X,Y,Z,Rx,Ry,Rz）固定の並びとし、
//   受信機は「元のパッドに在った軸だけ」を同じ順で HID ディスクリプタに出す。
//   こうすると /dev/input/js0 の軸番号・ボタン番号が元のパッドと一致する。
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define WG_NODE_RX 0x0001      // 受信機（親機）
#define WG_NODE_TX 0x0002      // 送信機（子機）

// 軸スロット（= HID Usage の並び。js の軸番号と同じ順序）
enum {
  WG_AX_X = 0, WG_AX_Y, WG_AX_Z, WG_AX_RX, WG_AX_RY, WG_AX_RZ,
  WG_AXIS_COUNT
};

#define WG_AXIS_CENTER  0x80   // 軸の中立値
#define WG_HAT_NEUTRAL  8      // ハットの中立（0〜7 が方向、0=上、時計回り）
#define WG_BUTTON_MAX   16     // 無線で送れるボタン数

#define WG_FLAG_PAD     0x01   // bit0: パッド接続中

#define WG_STATE_LEN    12
#define WG_INFO_LEN     10
#define WG_HB_LEN        5     // 受信機 → 送信機のハートビート
#define WG_CHCMD_LEN     3     // 送信機 → 受信機のチャンネル変更通知
#define WG_CHCMD_MAGIC 0x3C
#define WG_HB_MAGIC   0xC3
#define WG_INFO_MAGIC   0xA5
#define WG_INFO_VER     0x01

#define WG_CAP_HAT      0x01   // Info.caps bit0: ハットあり

// ---- State（12 バイト）----
//  0    u8   seq
//  1-2  u16  buttons (LE)
//  3    u8   hat
//  4-9  u8   axis[6]  = X, Y, Z, Rx, Ry, Rz
//  10   u8   flags
//  11   u8   checksum (byte0〜10 の XOR)
struct WgState {
  uint8_t  seq;
  uint16_t buttons;
  uint8_t  hat;
  uint8_t  axis[WG_AXIS_COUNT];
  uint8_t  flags;
};

// ---- Info（10 バイト）----
//  0    u8   0xA5
//  1    u8   version (0x01)
//  2    u8   axisMask   bit n = 軸スロット n が元のパッドに在る
//  3    u8   buttonCount
//  4    u8   caps       bit0 = ハットあり
//  5-6  u16  VID (LE)
//  7-8  u16  PID (LE)
//  9    u8   checksum (byte0〜8 の XOR)
struct WgInfo {
  uint8_t  axisMask;
  uint8_t  buttonCount;
  uint8_t  caps;
  uint16_t vid;
  uint16_t pid;
};

// ---- Heartbeat（4 バイト、受信機 → 送信機）----
// 送信は ACK のないブロードキャストなので、送信機は「届いたか」を知る手段がない。
// 受信機から定期的に投げ返すことで、送信機側でもリンクの生死が分かるようにする。
//  0  u8  0xC3
//  1  u8  seq
//  2  i8  受信機が見ている RSSI（dBm）
//  3  u8  flags（bit0: 受信機がパッドのパケットを受信できている）
//  4  u8  checksum (byte0〜3 の XOR)
//
// flags.bit0 が無いと、送信機の緑は「受信機→送信機の片方向」しか保証しない。
// 前方向（送信機→受信機）が死んでいても受信機の電波は届くので緑のままになってしまう。
#define WG_HB_RXOK  0x01
struct WgHeartbeat {
  uint8_t seq;
  int8_t  rssi;
  uint8_t flags;
};

static inline uint8_t wgXor(const uint8_t *b, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= b[i];
  return x;
}

static inline void wgStateNeutral(WgState *s) {
  s->buttons = 0;
  s->hat     = WG_HAT_NEUTRAL;
  s->flags   = 0;
  for (int i = 0; i < WG_AXIS_COUNT; i++) s->axis[i] = WG_AXIS_CENTER;
}

// seq を除いて比較する（変化検出用）
static inline bool wgStateEqual(const WgState *a, const WgState *b) {
  return a->buttons == b->buttons && a->hat == b->hat && a->flags == b->flags &&
         memcmp(a->axis, b->axis, WG_AXIS_COUNT) == 0;
}

static inline void wgStateEncode(const WgState *s, uint8_t *out) {
  out[0] = s->seq;
  out[1] = (uint8_t)(s->buttons & 0xFF);
  out[2] = (uint8_t)(s->buttons >> 8);
  out[3] = s->hat;
  for (int i = 0; i < WG_AXIS_COUNT; i++) out[4 + i] = s->axis[i];
  out[10] = s->flags;
  out[11] = wgXor(out, 11);
}

static inline bool wgStateDecode(const uint8_t *in, size_t len, WgState *s) {
  if (len != WG_STATE_LEN || in[11] != wgXor(in, 11)) return false;
  s->seq     = in[0];
  s->buttons = (uint16_t)in[1] | ((uint16_t)in[2] << 8);
  s->hat     = in[3];
  for (int i = 0; i < WG_AXIS_COUNT; i++) s->axis[i] = in[4 + i];
  s->flags   = in[10];
  return true;
}

static inline void wgInfoEncode(const WgInfo *n, uint8_t *out) {
  out[0] = WG_INFO_MAGIC;
  out[1] = WG_INFO_VER;
  out[2] = n->axisMask;
  out[3] = n->buttonCount;
  out[4] = n->caps;
  out[5] = (uint8_t)(n->vid & 0xFF);
  out[6] = (uint8_t)(n->vid >> 8);
  out[7] = (uint8_t)(n->pid & 0xFF);
  out[8] = (uint8_t)(n->pid >> 8);
  out[9] = wgXor(out, 9);
}

static inline bool wgInfoDecode(const uint8_t *in, size_t len, WgInfo *n) {
  if (len != WG_INFO_LEN) return false;
  if (in[0] != WG_INFO_MAGIC || in[1] != WG_INFO_VER) return false;
  if (in[9] != wgXor(in, 9)) return false;
  n->axisMask    = in[2] & 0x3F;          // 未定義ビットは落とす（受信機の再起動ループ防止）
  n->buttonCount = in[3] > WG_BUTTON_MAX ? WG_BUTTON_MAX : in[3];
  n->caps        = in[4] & WG_CAP_HAT;
  n->vid         = (uint16_t)in[5] | ((uint16_t)in[6] << 8);
  n->pid         = (uint16_t)in[7] | ((uint16_t)in[8] << 8);
  return true;
}

static inline bool wgInfoEqual(const WgInfo *a, const WgInfo *b) {
  return a->axisMask == b->axisMask && a->buttonCount == b->buttonCount &&
         a->caps == b->caps;
}

static inline void wgHbEncode(const WgHeartbeat *h, uint8_t *out) {
  out[0] = WG_HB_MAGIC;
  out[1] = h->seq;
  out[2] = (uint8_t)h->rssi;
  out[3] = h->flags;
  out[4] = wgXor(out, 4);
}

static inline bool wgHbDecode(const uint8_t *in, size_t len, WgHeartbeat *h) {
  if (len != WG_HB_LEN || in[0] != WG_HB_MAGIC || in[4] != wgXor(in, 4)) return false;
  h->seq   = in[1];
  h->rssi  = (int8_t)in[2];
  h->flags = in[3];
  return true;
}

// ---- チャンネル変更通知（3 バイト、送信機 → 受信機）----
//  0  u8  0x3C
//  1  u8  移動先チャンネル（31〜45）
//  2  u8  checksum (byte0〜1 の XOR)
//
// 複数ペアを同時に運用すると、グループ番号で論理的には分離されても同じチャンネルでは
// 電波を取り合う（キャリアセンスは RSSI -80dBm 以上で NG）。そこで送信機が起動時に
// 空きチャンネルを探し、ここで受信機へ伝える。
// 通知が届かず片方だけ移動した場合に備え、両機とも「一定時間受信が無ければ ch31 へ戻る」
// 動作を持つ（ch31 は待ち合わせ用として常に既定値）。
static inline void wgChCmdEncode(uint8_t ch, uint8_t *out) {
  out[0] = WG_CHCMD_MAGIC;
  out[1] = ch;
  out[2] = wgXor(out, 2);
}

static inline bool wgChCmdDecode(const uint8_t *in, size_t len, uint8_t *ch) {
  if (len != WG_CHCMD_LEN || in[0] != WG_CHCMD_MAGIC || in[2] != wgXor(in, 2)) return false;
  if (in[1] < 31 || in[1] > 45) return false;
  *ch = in[1];
  return true;
}
