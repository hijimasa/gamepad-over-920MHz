// im920sl_config.h
// XIAO RP2040 から IM920sL を設定・制御するヘルパー（Arduino-Pico 用）
// 根拠: IM920sL 取扱説明書 Rev.1.5 (2025.8.22)、IM920sL-ADP 製品資料 Rev.1.0
//
// 機能
//  im920AutoConfig() : 起動時に設定を読み出し、目標値と違う項目だけ ENWR で書き込む
//                      （Flash 書換えは全項目合計 10 万回まで。一致していれば書かない）
//  im920Bridge()     : USB CDC(Serial) <-> IM920sL(Serial1) の透過ブリッジ（IM920-USB2 の代わり）
//  im920Pair()       : STGN によるグループ番号登録（ペアリング）
//  im920Send()       : BUSY を確認して TXDA 送信
//  im920ParseRx()    : 受信行 "aa,bbbb,dd:xx,xx,..." を解析
//
// 配線（回路図どおり / 変換アダプタ IM920sL-ADP の J1/J2）
//  GPIO0 (D6/TX) -> J2-3 RxD   (IM920sL pin6)
//  GPIO1 (D7/RX) <- J1-4 TxD   (pin7)
//  GPIO28 (D2)   <- J1-1 BUSY  (pin1)  L の間だけコマンド受付
//  GPIO29 (D3)   -> J1-10 RESET(pin19) L でリセット、内部 10kΩ プルアップ
//  GPIO2 (D8)    <- SW1（押下で GND、内部プルアップ）
//  J1-9 VCC=3.3V / J2-9 GND / J1-8 STATUS -> LED（任意）
//
// 役割
//  受信機（PC 側）= 親機 ノード 0001（グループ番号に自分の固有 ID が自動設定される）
//  送信機（パッド側）= 子機 ノード 0002（STGN で親機のグループ番号を受け取る）
#pragma once
#include <Arduino.h>

#define IM_SERIAL      Serial1
#define IM_BAUD        19200        // モジュールの初期値（SBRT 4）
// 目標ボーレート。12 バイト送信の UART 時間が 19200bps では約 16ms かかり、
// 電波の送信時間 5.5ms の 3 倍を食っている。115200bps なら約 2.7ms。
// SBRT は取扱説明書に「Flash メモリに記憶します」の記載が無く揮発性とみられるため、
// 毎起動で設定しても Flash 書換え回数（全項目合計 10 万回）を消費しない。
#define IM_BAUD_TARGET 115200       // SBRT 7
#define IM_SBRT_TARGET "7"
#define PIN_IM_BUSY    28
#define PIN_IM_RESET   29
#define PIN_SW_CFG     2

// AutoConfig が目標にするチャンネル（wg_channel.h から書き換えられる）
inline char *im920ChannelBuf() { static char b[4] = "31"; return b; }

// ペアリング済みか（AutoConfig で判定）
inline bool &im920PairedRef() { static bool v = true; return v; }

struct ImParam {
  const char *readCmd;
  const char *setCmd;
  const char *value;    // 読出し応答と同じ書式
};

// 並び順に意味あり: STNN(0001 は ENWR 必須) → STRT（変更するとチャネルが既定値に戻る）→ STCH
static const ImParam IM_PARAMS[] = {
#ifdef IM_ROLE_TX
  {"RDNN", "STNN", "0002"},   // 子機
#else
  {"RDNN", "STNN", "0001"},   // 親機
#endif
  {"RDRT", "STRT", "1"},      // 1=高速モード(100kbps) 2=中距離 3=長距離
  {"RDCH", "STCH", "31"},     // 31〜45 = 4s モード（1時間あたり送信時間の総和制限なし）
                              // 01〜29 = 360s モード（1時間 360 秒まで。高頻度送信だと超過して NG）
  {"RDPO", "STPO", "2"},      // 2=10mW
  {"RDTR", "STTR", "00"},     // キャリアセンス NG 時の自動リトライ回数（遅延を避けるため 0）
};

// ---- 低レベル ----
inline void im920Begin() {
  pinMode(PIN_IM_BUSY, INPUT);
  pinMode(PIN_IM_RESET, INPUT);          // 解放（モジュール内部プルアップで H）
  pinMode(PIN_SW_CFG, INPUT_PULLUP);
  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(IM_BAUD);
}

// 指定ボーレートで開き直す
inline void im920Reopen(uint32_t baud) {
  IM_SERIAL.end();
  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(baud);
  delay(20);
}

inline void im920Flush() { while (IM_SERIAL.available()) IM_SERIAL.read(); }

// ブロッキング中でも LED の点滅などを続けるためのフック
typedef void (*Im920TickFn)();
inline Im920TickFn &im920TickRef() { static Im920TickFn fn = nullptr; return fn; }
inline void im920SetTick(Im920TickFn fn) { im920TickRef() = fn; }
inline void im920Tick() { Im920TickFn f = im920TickRef(); if (f) f(); }

// 受信データ行（"aa,bbbb,dd:..."）を 1 本だけ退避しておく。
// 送信処理が応答を待っている間に届いた受信行を捨ててしまわないようにする。
inline char *im920StashBuf() { static char b[128]; return b; }
inline bool &im920StashHas() { static bool v = false; return v; }
inline uint32_t &im920StashPuts() { static uint32_t n = 0; return n; }
inline uint32_t &im920RawLines() { static uint32_t n = 0; return n; }

inline void im920StashPut(const String &line) {
  if (line.indexOf(':') < 0) return;
  im920StashPuts()++;              // 受信データ行だけ
  strncpy(im920StashBuf(), line.c_str(), 127);
  im920StashBuf()[127] = '\0';
  im920StashHas() = true;
}
inline bool im920StashTake(char *out, size_t cap) {
  if (!im920StashHas()) return false;
  strncpy(out, im920StashBuf(), cap - 1);
  out[cap - 1] = '\0';
  im920StashHas() = false;
  return true;
}

// 1 行読み（CR/LF 除去）。タイムアウト時は空文字
// 受信データ行は応答ではないので、退避したうえで読み飛ばす
inline String im920ReadLine(uint32_t timeoutMs) {
  String line;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (IM_SERIAL.available()) {
      char c = IM_SERIAL.read();
      if (c == '\n') {
        line.trim();
        if (line.length()) {
          im920RawLines()++;
          if (line.indexOf(':') >= 0) { im920StashPut(line); line = ""; continue; }
          return line;
        }
        continue;
      }
      if (c != '\r') line += c;
    }
    im920Tick();
    yield();
  }
  line.trim();
  return line;
}

// BUSY=L（受付可）になるまで待つ
inline bool im920WaitReady(uint32_t timeoutMs = 200) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_IM_BUSY) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;
    im920Tick();
    yield();
  }
  return true;
}

// ハードウェアリセット（STGN 後の通常復帰はこれが必要。SRST では復帰しない）
inline String im920HardReset() {
  digitalWrite(PIN_IM_RESET, LOW);
  pinMode(PIN_IM_RESET, OUTPUT);
  delay(20);
  pinMode(PIN_IM_RESET, INPUT);
  return im920ReadLine(1500);            // 起動メッセージ例: "IM920sL Ver.01.00"
}

// コマンド送信 → 1 行応答
inline String im920Cmd(const String &cmd, uint32_t timeoutMs = 500) {
  if (!im920WaitReady()) return "";
  im920Flush();
  IM_SERIAL.print(cmd);
  IM_SERIAL.print("\r\n");
  return im920ReadLine(timeoutMs);
}

// ---- 設定 ----
// 戻り値: 書き込んだ項目数（-1 = 失敗）
inline int im920AutoConfig(Stream *log = nullptr) {
  int changed = 0;
  bool writeEnabled = false;
  // HEX 入出力モードを明示する。既定だが ENWR で Flash に記憶される設定なので、
  // 過去に ECIO を書かれたモジュールを挿すと TXDA が文字列として解釈され、
  // 無言で通信できなくなる。ENWR の外で出すので Flash は消費しない。
  im920Cmd("DCIO");
  for (const auto &p : IM_PARAMS) {
    // チャンネルだけは EEPROM の設定値を使う（複数ペア運用のため）
    const char *target = strcmp(p.setCmd, "STCH") ? p.value : im920ChannelBuf();
    String cur = im920Cmd(p.readCmd);
    if (log) log->printf("%s -> '%s' (target %s)\n", p.readCmd, cur.c_str(), target);
    if (cur.equalsIgnoreCase(target)) continue;
    if (!writeEnabled) {
      if (im920Cmd("ENWR") != "OK") return -1;
      writeEnabled = true;
    }
    String res = im920Cmd(String(p.setCmd) + " " + target);
    if (log) log->printf("%s %s -> %s\n", p.setCmd, target, res.c_str());
    if (res != "OK") { im920Cmd("DSWR"); return -1; }
    changed++;
  }
  if (writeEnabled) im920Cmd("DSWR");
  // グループ番号が未設定だと TXDA は必ず NG になる（取扱説明書 10-4 TXDA）。
  // キャリアセンス失敗と区別がつかないので、ここで明示する。
  String gn = im920Cmd("RDGN");
  if (log) log->printf("RDGN -> %s\n", gn.c_str());
  if (gn.equalsIgnoreCase("FFFFFFFF")) {
    if (log) log->println(F("!! グループ番号が未設定です。ペアリングするまで送信できません"));
    im920PairedRef() = false;
  } else if (gn.length() >= 8) {
    im920PairedRef() = true;
  }
  return changed;
}

// ペアリング（親機・子機の両方で実行。両者を 50cm 以内に置く）
//  親機: ENWR → STGN で登録パケットを送信
//  子機: ENWR → STGN で待ち受け、成功すると "GRNOREGD" を出力
//  完了後は両方ともハードウェアリセットで通常動作に戻す
inline bool im920Pair(bool isParent, uint32_t waitMs = 30000, Stream *log = nullptr) {
  if (im920Cmd("ENWR") != "OK") return false;
  if (im920Cmd("STGN", 1000) != "OK") return false;
  bool ok = isParent;                       // 親機は OK 応答で送信開始
  if (!isParent) {
    uint32_t t0 = millis();
    while (millis() - t0 < waitMs) {
      String l = im920ReadLine(500);
      if (log && l.length()) log->println(l);
      if (l == "GRNOREGD") { ok = true; break; }
    }
  } else {
    uint32_t t0 = millis();                 // 子機側の完了を待つ
    while (millis() - t0 < waitMs) { im920Tick(); delay(1); }
  }
  im920HardReset();
  if (log) log->printf("RDGN -> %s\n", im920Cmd("RDGN").c_str());
  return ok;
}

// USB CDC <-> IM920sL 透過ブリッジ
// CDC から '+' を 3 回続けて送ると抜ける（リセット不要）
inline void im920Bridge() {
  int plus = 0;
  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '+') {
        if (++plus >= 3) {
          Serial.println();
          Serial.println(F("bridge を抜けました"));
          return;
        }
        continue;                 // '+' は 3 回目の判定が済むまで送らない
      }
      while (plus) { IM_SERIAL.write('+'); plus--; }
      IM_SERIAL.write(c);
    }
    while (IM_SERIAL.available()) Serial.write(IM_SERIAL.read());
    im920Tick();
    yield();
  }
}

// ---- チャンネル操作 ----
// ENWR を使わないので Flash 書換え回数を消費しない（取扱説明書の STCH の項には
// 「ENWR コマンド実行時は Flash メモリに記憶します」の記載が無い＝揮発的）。
// 起動時は AutoConfig が Flash の既定値 ch31 に戻すため、待ち合わせ用として機能する。
inline bool im920SetChannelVolatile(uint8_t ch) {
  if (ch < 31 || ch > 45) return false;
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "STCH %02u", ch);
  return im920Cmd(cmd, 500) == "OK";
}

// 現在のチャンネルの信号強度[dBm]。読めなければ 0
inline int8_t im920ReadRssi() {
  String r = im920Cmd("RDRS", 300);
  if (r.length() < 2) return 0;
  return (int8_t)strtoul(r.c_str(), nullptr, 16);
}

// ---- データ ----
// TXDA（HEX 入出力モード）。len は 1〜32
// 失敗の内訳を分けて返す（NG 率の原因を切り分けるため）
enum {
  IM_SEND_OK = 0,
  IM_SEND_BUSY,      // BUSY が L にならない（前の送信がまだ終わっていない）
  IM_SEND_NG,        // モジュールが NG を返した（キャリアセンス失敗・送信時間制限など）
  IM_SEND_NORESP,    // 応答が返ってこない
  IM_SEND_BADLEN,
};

// 直近に TXDA の応答として読んだ行（想定外の応答の切り分け用）
inline String &im920LastResponse() { static String r; return r; }

inline int im920SendEx(const uint8_t *buf, size_t len) {
  static const char hex[] = "0123456789ABCDEF";
  if (len == 0 || len > 32) return IM_SEND_BADLEN;
  if (!im920WaitReady()) return IM_SEND_BUSY;
  // ここで捨てると届いたハートビートを失うので flush しない
  char cmd[5 + 64 + 1] = "TXDA ";
  for (size_t i = 0; i < len; i++) {
    cmd[5 + 2 * i] = hex[buf[i] >> 4];
    cmd[6 + 2 * i] = hex[buf[i] & 0x0F];
  }
  cmd[5 + 2 * len] = '\0';
  IM_SERIAL.print(cmd);
  IM_SERIAL.print("\r\n");
  String r = im920ReadLine(300);
  im920LastResponse() = r;
  if (r == "OK") return IM_SEND_OK;
  if (r.length() == 0) return IM_SEND_NORESP;
  return IM_SEND_NG;
}

inline bool im920Send(const uint8_t *buf, size_t len) {
  return im920SendEx(buf, len) == IM_SEND_OK;
}

// 受信行 "00,0002,C4:01,02,03" を解析。成功で true
inline bool im920ParseRx(const String &line, uint16_t &node, int8_t &rssi,
                         uint8_t *out, size_t maxLen, size_t &len) {
  int colon = line.indexOf(':');
  if (colon < 10 || line.charAt(2) != ',' || line.charAt(7) != ',') return false;
  node = (uint16_t)strtoul(line.substring(3, 7).c_str(), nullptr, 16);
  rssi = (int8_t)strtoul(line.substring(8, colon).c_str(), nullptr, 16);  // dBm（符号付き）
  len = 0;
  for (int i = colon + 1; i + 2 <= (int)line.length() && len < maxLen; i += 3) {
    out[len++] = (uint8_t)strtoul(line.substring(i, i + 2).c_str(), nullptr, 16);
  }
  return len > 0;
}


// ---- 受信機のメインループ用: ノンブロッキングの行読み ----
// im920Cmd() / im920ReadLine() は待ちに入り RX バッファも捨てるので、
// メインループではこちらを使う。
class Im920LineReader {
 public:
  // 1 行そろったら true。line は '\0' 終端（CR/LF は含まない）
  bool poll() {
    while (IM_SERIAL.available()) {
      char c = (char)IM_SERIAL.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line[_n] = '\0';
        size_t n = _n;
        _n = 0;
        if (n) return true;
        continue;
      }
      if (_n < sizeof(line) - 1) line[_n++] = c;
      else _n = 0;                      // 長すぎる行は捨てる
    }
    return false;
  }
  void reset() { _n = 0; }
  char line[128];   // 32 バイト受信時 "aa,bbbb,dd:" + 32*3-1 = 106 文字
 private:
  size_t _n = 0;
};

// 受信行 "00,0002,C4:01,02,03" を解析（char* 版）
inline bool im920ParseRx(const char *line, uint16_t &node, int8_t &rssi,
                         uint8_t *out, size_t maxLen, size_t &len) {
  size_t n = strlen(line);
  const char *colon = strchr(line, ':');
  if (!colon || (size_t)(colon - line) < 10 || n < 11) return false;
  if (line[2] != ',' || line[7] != ',') return false;
  char tmp[8];
  memcpy(tmp, line + 3, 4); tmp[4] = '\0';
  node = (uint16_t)strtoul(tmp, nullptr, 16);
  size_t rlen = (size_t)(colon - (line + 8));
  if (rlen >= sizeof(tmp)) return false;
  memcpy(tmp, line + 8, rlen); tmp[rlen] = '\0';
  rssi = (int8_t)strtoul(tmp, nullptr, 16);
  len = 0;
  for (const char *p = colon + 1; p[0] && p[1] && len < maxLen; p += 3) {
    char h[3] = { p[0], p[1], '\0' };
    out[len++] = (uint8_t)strtoul(h, nullptr, 16);
    if (p[2] != ',') break;
  }
  return len > 0;
}

/* ---- 使い方 ----
#define IM_ROLE_TX            // 送信機のときだけ定義
#include "im920sl_config.h"

void setup() {
  Serial.begin(115200);
  im920Begin();
  im920HardReset();
  if (digitalRead(PIN_SW_CFG) == LOW) {          // SW1 を押しながら起動
    delay(1500);
    uint32_t t0 = millis();
    while (digitalRead(PIN_SW_CFG) == LOW) {}    // 押し続け時間で分岐
    if (millis() - t0 > 3000) {                  // 長押し: ペアリング
      im920AutoConfig(&Serial);
#ifdef IM_ROLE_TX
      im920Pair(false, 30000, &Serial);
#else
      im920Pair(true, 10000, &Serial);
#endif
    } else {                                     // 短押し: 設定ブリッジ
      Serial.println("IM920sL bridge mode");
      im920Bridge();
    }
  }
  if (im920AutoConfig(&Serial) > 0) im920HardReset();
}

// 送信機: 状態が変わったら即送信＋無変化でも 100ms ごとに送信
// 受信機: 受信行を im920ParseRx() し、node == 0x0002 のものだけ採用。
//         300ms 受信が途絶えたら全ボタン OFF・軸中立を HID に出す
*/

// 応答を待たずに TXDA を投げる（受信処理を止めたくないとき用）
// 応答の OK/NG 行は、呼び出し側のノンブロッキング行読みが拾って無視することになる
inline bool im920SendNoWait(const uint8_t *buf, size_t len) {
  static const char hex[] = "0123456789ABCDEF";
  if (len == 0 || len > 32) return false;
  if (digitalRead(PIN_IM_BUSY) == HIGH) return false;   // 送信中なら今回は見送る
  char cmd[5 + 64 + 1] = "TXDA ";
  for (size_t i = 0; i < len; i++) {
    cmd[5 + 2 * i] = hex[buf[i] >> 4];
    cmd[6 + 2 * i] = hex[buf[i] & 0x0F];
  }
  cmd[5 + 2 * len] = '\0';
  IM_SERIAL.print(cmd);
  IM_SERIAL.print("\r\n");
  return true;
}

// ---- ボーレートの自動判別と高速化 ----
// 戻り値: 実際に使っているボーレート（0 = モジュールが応答しない）
//
// モジュールが今どのボーレートで喋っているか分からない状況（初期値のまま、
// 前回 SBRT 済み、他プロジェクトからの転用）でも確実に繋がるよう、
// RDVR の応答に "IM920" が含まれるかで判定する。
inline uint32_t im920SetupBaud(Stream *log = nullptr) {
  static const uint32_t candidates[] = {IM_BAUD_TARGET, IM_BAUD, 57600, 38400,
                                        9600, 230400, 4800, 2400, 1200, 460800};
  uint32_t found = 0;
  for (uint32_t b : candidates) {
    im920Reopen(b);
    for (int attempt = 0; attempt < 2 && !found; attempt++) {
      String r = im920Cmd("RDVR", 400);
      if (r.indexOf("IM920") >= 0) found = b;
    }
    if (found) break;
  }
  if (!found) {
    if (log) log->println(F("ボーレート判別に失敗（モジュールが応答しない）"));
    im920Reopen(IM_BAUD);
    return 0;
  }
  if (log) log->printf("モジュールのボーレート: %lu bps\n", (unsigned long)found);
  if (found == IM_BAUD_TARGET) return found;

  // 目標まで上げる。SBRT は「レスポンス出力後にボーレートを変更」する
  String r = im920Cmd("SBRT " IM_SBRT_TARGET, 500);
  if (r != "OK") {
    if (log) log->printf("SBRT -> %s（%lu bps のまま使います）\n", r.c_str(),
                         (unsigned long)found);
    return found;
  }
  im920Reopen(IM_BAUD_TARGET);
  String v = im920Cmd("RDVR", 400);
  if (v.indexOf("IM920") >= 0) {
    if (log) log->printf("%lu bps に切り替えました（%s）\n",
                         (unsigned long)IM_BAUD_TARGET, v.c_str());
    return IM_BAUD_TARGET;
  }
  if (log) log->println(F("切替後に応答が無いため元のボーレートに戻します"));
  im920Reopen(found);
  return found;
}
