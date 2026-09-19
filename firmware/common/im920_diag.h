// im920_diag.h — IM920sL が応答しないときの切り分け用
//   diag        BUSY の状態と、ハードリセット後に Serial1 に出てくる生バイトを表示
//   raw <cmd>   BUSY を無視してコマンドを送り、返ってきた生バイトを表示
//   scan        ボーレートを変えながら RDVR を投げ、応答するものを探す
#pragma once
#include <Arduino.h>
#include "im920sl_config.h"

inline void im920DumpRaw(Stream &log, uint32_t ms) {
  uint32_t t0 = millis();
  int n = 0;
  String ascii;
  while (millis() - t0 < ms) {
    while (IM_SERIAL.available()) {
      uint8_t c = (uint8_t)IM_SERIAL.read();
      log.printf("%02X ", c);
      ascii += (c >= 0x20 && c < 0x7F) ? (char)c : '.';
      n++;
      t0 = millis();
    }
    im920Tick();
    yield();
  }
  log.printf("\n  %d bytes", n);
  if (n) log.printf("  \"%s\"", ascii.c_str());
  log.println();
}

// ピンが外部に駆動されているか、浮いているかを見る
//   内部プルアップ/プルダウンを切り替えて、値が追従したら「何も繋がっていない」
inline void im920PinProbe(Stream &log, uint8_t pin, const char *name,
                          const char *expect) {
  pinMode(pin, INPUT_PULLUP);   delay(5); int up = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN); delay(5); int dn = digitalRead(pin);
  pinMode(pin, INPUT);
  const char *verdict;
  if (up == 1 && dn == 0)      verdict = "浮いている（未接続の可能性）";
  else if (up == 1 && dn == 1) verdict = "外部が H に駆動（接続あり）";
  else if (up == 0 && dn == 0) verdict = "外部が L に駆動（接続あり）";
  else                         verdict = "不定";
  log.printf("  %-6s GPIO%-2d pull-up=%d pull-down=%d -> %s  [期待: %s]\n",
             name, pin, up, dn, verdict, expect);
}

// 線の静電容量の目安（断線の切り分け）。L に落として内部プルアップで離し、
// H になるまでのループ回数を数える。相手の回路がぶら下がっていれば数が増える。
inline uint32_t im920RiseCount(uint8_t pin) {
  uint32_t best = 0;
  for (int k = 0; k < 4; k++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(300);
    uint32_t n = 0;
    noInterrupts();
    pinMode(pin, INPUT_PULLUP);
    while (!digitalRead(pin) && n < 200000) n++;
    interrupts();
    pinMode(pin, INPUT);
    if (n > best) best = n;
    delayMicroseconds(300);
  }
  return best;
}

inline void im920PinTest(Stream &log) {
  log.println(F("ピンの接続確認（XIAO の全ピン。何かに繋がっているピンを探す）:"));
  struct { uint8_t gpio; const char *label; const char *expect; } pins[] = {
    { 28, "D2/A2", "IM920sL BUSY（出力＝駆動される）" },
    { 29, "D3/A3", "IM920sL RESET（内部 10k プルアップで H）" },
    {  6, "D4",    "未接続" },
    {  7, "D5",    "未接続" },
    {  0, "D6/TX", "IM920sL RxD（入力。内部プルアップがあれば H）" },
    {  1, "D7/RX", "IM920sL TxD（出力＝駆動される）" },
    {  2, "D8",    "SW1（未実装なら浮く）" },
    {  4, "D9",    "未接続" },
    {  3, "D10",   "未接続" },
  };
  for (auto &p : pins) im920PinProbe(log, p.gpio, p.label, p.expect);
  log.println(F("線の静電容量の目安（未接続ピンと同程度なら、その線は繋がっていない）:"));
  for (auto &p : pins) {
    if (p.gpio == 1) continue;                 // UART RX は後で戻すのでまとめて
    log.printf("  %-6s GPIO%-2d -> %lu\n", p.label, p.gpio,
               (unsigned long)im920RiseCount(p.gpio));
  }
  log.printf("  %-6s GPIO%-2d -> %lu\n", "D7/RX", 1, (unsigned long)im920RiseCount(1));
  IM_SERIAL.end();                 // RX のピン機能を戻す
  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(IM_BAUD);
}

// TX(GPIO0) と RX(GPIO1) が繋がって（短絡して）いないかを見る
// 高インピーダンスの線は 1 回の測定だと誤判定するので、何度か繰り返して一致を見る
inline void im920LoopTest(Stream &log) {
  IM_SERIAL.end();
  pinMode(0, OUTPUT);
  pinMode(1, INPUT);
  int follow = 0;
  const int N = 5;
  for (int i = 0; i < N; i++) {
    digitalWrite(0, HIGH); delay(5); int hi = digitalRead(1);
    digitalWrite(0, LOW);  delay(5); int lo = digitalRead(1);
    if (hi == 1 && lo == 0) follow++;
  }
  digitalWrite(0, HIGH); delay(5);
  pinMode(0, INPUT);
  log.printf("TX/RX 短絡テスト: %d/%d 回で RX が TX に追従 -> %s\n", follow, N,
             (follow == N) ? "TX と RX が繋がっている（または相手が駆動していない）"
                           : (follow == 0)
                               ? "RX は H のまま（相手が H を保持＝正常な配線）"
                               : "不安定（線が浮いている可能性）");
  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(IM_BAUD);
}

// リセット中に BUSY が動くか（RESET が本当に繋がっているか）
inline void im920ResetWatch(Stream &log) {
  int before = digitalRead(PIN_IM_BUSY);
  digitalWrite(PIN_IM_RESET, LOW);
  pinMode(PIN_IM_RESET, OUTPUT);
  delay(5);
  int during = digitalRead(PIN_IM_BUSY);
  pinMode(PIN_IM_RESET, INPUT);
  int changes = 0, last = digitalRead(PIN_IM_BUSY);
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    int v = digitalRead(PIN_IM_BUSY);
    if (v != last) { changes++; last = v; }
  }
  log.printf("BUSY: リセット前=%d リセット中=%d 解除後 1.5 秒の変化回数=%d 最終=%d\n",
             before, during, changes, last);
  if (changes == 0)
    log.println(F("  → BUSY が一度も動かない。RESET か BUSY の配線、"
                  "またはモジュールがデータモードで起動していない可能性"));
}

// ---- TxD/RxD が逆に繋がっていないかを調べる ----
// RP2040 の UART0 は GPIO0=TX / GPIO1=RX に固定なので、逆向きはソフト UART で試す。
inline void swUartSend(uint8_t pin, const char *str, uint32_t bitUs) {
  for (const char *p = str; *p; p++) {
    uint8_t b = (uint8_t)*p;
    noInterrupts();
    uint32_t t = time_us_32();
    digitalWrite(pin, LOW);                             // start bit
    t += bitUs; while ((int32_t)(time_us_32() - t) < 0) {}
    for (int i = 0; i < 8; i++) {
      digitalWrite(pin, (b >> i) & 1 ? HIGH : LOW);
      t += bitUs; while ((int32_t)(time_us_32() - t) < 0) {}
    }
    digitalWrite(pin, HIGH);                            // stop bit
    t += bitUs; while ((int32_t)(time_us_32() - t) < 0) {}
    interrupts();
  }
}

inline int swUartCapture(uint8_t pin, uint32_t ms, uint8_t *buf, int cap, uint32_t bitUs) {
  int n = 0;
  uint32_t end = millis() + ms;
  while (millis() < end && n < cap) {
    if (digitalRead(pin)) continue;                     // start bit待ち
    noInterrupts();
    uint32_t t = time_us_32() + bitUs + bitUs / 2;      // ビット中央
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) {
      while ((int32_t)(time_us_32() - t) < 0) {}
      if (digitalRead(pin)) v |= (uint8_t)(1u << i);
      t += bitUs;
    }
    while ((int32_t)(time_us_32() - t) < 0) {}          // stop bit
    interrupts();
    buf[n++] = v;
  }
  return n;
}

inline void im920SwapTest(Stream &log) {
  const uint32_t bitUs = 1000000UL / IM_BAUD;           // 19200bps -> 52us
  uint8_t buf[96];
  log.println(F("TxD/RxD 逆接続テスト（GPIO1 から送信し、GPIO0 で受信する）"));
  IM_SERIAL.end();
  pinMode(1, OUTPUT); digitalWrite(1, HIGH);            // 逆向きの TX
  pinMode(0, INPUT_PULLUP);                             // 逆向きの RX
  delay(5);

  // 1) RESET をかけて起動メッセージを待つ
  digitalWrite(PIN_IM_RESET, LOW);
  pinMode(PIN_IM_RESET, OUTPUT);
  delay(20);
  pinMode(PIN_IM_RESET, INPUT);
  int n = swUartCapture(0, 1500, buf, sizeof(buf), bitUs);
  log.printf("  リセット後の受信: %d バイト  \"", n);
  for (int i = 0; i < n; i++) log.print((buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.');
  log.println("\"");

  // 2) RDVR を送って応答を待つ
  swUartSend(1, "RDVR\r\n", bitUs);
  n = swUartCapture(0, 700, buf, sizeof(buf), bitUs);
  log.printf("  RDVR の応答:      %d バイト  \"", n);
  for (int i = 0; i < n; i++) log.print((buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.');
  log.println("\"");
  log.println(F("  ここで応答があれば TxD/RxD が逆。D6/D7 の 2 本を入れ替えてください"));

  pinMode(1, INPUT);
  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(IM_BAUD);
}

// モジュールの GND（または VCC）が繋がっていないかを見る決定的なテスト
//   GND が浮いていると、モジュールの内部 ESD/プルアップ経由で全ピンが同じ節点に
//   ぶら下がる形になり、1 本を L に引くと他のピンも一緒に下がる。
//   正常に GND が繋がっていれば、TX を L にしても BUSY や RESET は動かない。
inline void im920GndTest(Stream &log) {
  IM_SERIAL.end();
  pinMode(1, INPUT); pinMode(PIN_IM_BUSY, INPUT); pinMode(PIN_IM_RESET, INPUT);
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH); delay(5);
  int rxH = digitalRead(1), buH = digitalRead(PIN_IM_BUSY), reH = digitalRead(PIN_IM_RESET);
  digitalWrite(0, LOW);  delay(5);
  int rxL = digitalRead(1), buL = digitalRead(PIN_IM_BUSY), reL = digitalRead(PIN_IM_RESET);
  digitalWrite(0, HIGH); delay(5);
  pinMode(0, INPUT);

  log.println(F("GND 接続テスト（GPIO0 を H/L したときの他ピンの動き）:"));
  log.printf("  TX=H: RX=%d BUSY=%d RESET=%d\n", rxH, buH, reH);
  log.printf("  TX=L: RX=%d BUSY=%d RESET=%d\n", rxL, buL, reL);
  int followers = (rxH != rxL) + (buH != buL) + (reH != reL);
  if (followers >= 2) {
    log.println(F("  → GPIO0 を L にすると他のピンも一緒に下がる。"));
    log.println(F("     モジュールの GND（ADP J2-9 / pin18）が繋がっていない疑いが濃厚。"));
    log.println(F("     VCC に 3.3V が来ていても、GND が浮いていると全ピンが H に張り付き、"));
    log.println(F("     BUSY が L に落ちず、起動メッセージも出ない（今の症状と一致）。"));
  } else if (followers == 0) {
    log.println(F("  → 他のピンは動かない。GND は繋がっていそう。"));
  } else {
    log.println(F("  → 一部だけ追従。配線を個別に確認してほしい。"));
  }

  IM_SERIAL.setTX(0);
  IM_SERIAL.setRX(1);
  IM_SERIAL.setFIFOSize(256);
  IM_SERIAL.begin(IM_BAUD);
}

// 自分（XIAO）の送信が本当にピンに出ているかを確認する
//   Serial1 で送信しながら GPIO0 を監視し、変化回数を数える。
//   0 回なら XIAO 側の送信が出ていない（ピン割り当てや外部の強い駆動を疑う）。
inline void im920TxTest(Stream &log) {
  im920Flush();
  const char *msg = "RDVR\r\n";
  uint32_t changes = 0;
  int last = gpio_get(0);
  IM_SERIAL.print(msg);
  uint32_t t0 = micros();
  while (micros() - t0 < 6000) {            // 6 バイト @19200bps ≒ 3.1ms
    int v = gpio_get(0);
    if (v != last) { changes++; last = v; }
  }
  log.printf("送信テスト: \"RDVR\" 送信中の GPIO0 の変化回数 = %lu\n",
             (unsigned long)changes);
  if (changes == 0)
    log.println(F("  → XIAO の送信がピンに出ていない（Serial1 のピン割り当て、"
                  "または外部から強く H に固定されている）"));
  else
    log.println(F("  → XIAO の送信自体は出ている（モジュールに届いているかは別）"));

  // 同じ時間、受信側の線も見る
  changes = 0; last = gpio_get(1);
  t0 = micros();
  while (micros() - t0 < 300000) {
    int v = gpio_get(1);
    if (v != last) { changes++; last = v; }
  }
  log.printf("受信テスト: 送信後 300ms の GPIO1 の変化回数 = %lu\n",
             (unsigned long)changes);
  if (changes == 0)
    log.println(F("  → モジュールからの返信がまったく無い"));
}

// ボーレートを変えながら「ハードリセット → 起動メッセージ」を見る
// 起動メッセージはこちらの送信に依存せず必ず出るので、送信経路が怪しくても
// ボーレートを特定できる（正しいレートなら "IM920sL Ver.xx.xx" が読める）
inline uint32_t im920ScanReset(Stream &log) {
  static const uint32_t bauds[] = {19200, 115200, 57600, 38400, 9600, 4800, 2400, 1200};
  uint32_t found = 0;
  for (uint32_t b : bauds) {
    IM_SERIAL.end();
    IM_SERIAL.begin(b);
    delay(30);
    im920Flush();
    digitalWrite(PIN_IM_RESET, LOW);
    pinMode(PIN_IM_RESET, OUTPUT);
    delay(20);
    pinMode(PIN_IM_RESET, INPUT);

    uint8_t buf[64];
    int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 1200 && n < (int)sizeof(buf)) {
      while (IM_SERIAL.available() && n < (int)sizeof(buf)) buf[n++] = (uint8_t)IM_SERIAL.read();
      im920Tick();
    }
    log.printf("%6lu bps: %2d バイト ", (unsigned long)b, n);
    bool ascii = n >= 4;
    for (int i = 0; i < n; i++) {
      log.printf("%02X ", buf[i]);
      if (!((buf[i] >= 0x20 && buf[i] < 0x7F) || buf[i] == '\r' || buf[i] == '\n')) ascii = false;
    }
    if (n) {
      log.print(" \"");
      for (int i = 0; i < n; i++) log.print((buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.');
      log.print("\"");
    }
    log.println(ascii ? "  <- 読める！" : "");
    if (ascii && !found) found = b;
  }
  IM_SERIAL.end();
  IM_SERIAL.begin(IM_BAUD);
  if (found) log.printf("起動メッセージが読めたボーレート: %lu bps\n", (unsigned long)found);
  else       log.println(F("どのボーレートでも起動メッセージが読めない"));
  return found;
}

// 実験用: 設定値をその場で書き換える（ENWR → 設定 → DSWR）
// ※Flash を消費するので測定用に限る。再起動すると AutoConfig が既定値に戻す
inline bool im920SetParam(Stream &log, const char *cmd, const char *value) {
  if (im920Cmd("ENWR") != "OK") { log.println(F("ENWR NG")); return false; }
  String r = im920Cmd(String(cmd) + " " + value, 1000);
  im920Cmd("DSWR");
  log.printf("%s %s -> %s\n", cmd, value, r.c_str());
  return r == "OK";
}

// ---- パケット暗号化＋送信元認証（AES-GCM 256bit）----
// 取扱説明書 7-6: 「暗号化設定時は、暗号化キーが一致する場合だけ通信可能で、
// 非暗号化パケットを無視し、非暗号化設定時は暗号化パケットを無視します」
// 「暗号化有効時は通信時間が 24 バイト分増加します」（= 高速モードで +1.92ms）
// ※STGN（ペアリング）時は常に非暗号化パケットなので、ペアリングは先に済ませてよい
//
// STKY は ENWR 中のみ実行可能で Flash に記憶される（読出しコマンドは無い）。
// 書換え回数を消費するので、設定は明示的なコマンドのときだけ行う。
inline bool im920SetEncryption(Stream &log, const char *key64) {
  if (key64) {
    size_t n = strlen(key64);
    if (n != 64) { log.println(F("キーは 16 進数 64 桁で指定してください")); return false; }
    for (size_t i = 0; i < n; i++) {
      if (!isxdigit((int)(unsigned char)key64[i])) {
        log.println(F("キーに 16 進数以外の文字があります")); return false;
      }
    }
  }
  if (im920Cmd("ENWR") != "OK") { log.println(F("ENWR NG")); return false; }
  bool ok = true;
  if (key64) {
    String r = im920Cmd(String("STKY ") + key64, 1000);
    log.printf("STKY -> %s\n", r.c_str());
    ok = ok && (r == "OK");
  }
  String r2 = im920Cmd(key64 ? "EENC" : "DENC", 1000);
  log.printf("%s -> %s\n", key64 ? "EENC" : "DENC", r2.c_str());
  ok = ok && (r2 == "OK");
  im920Cmd("DSWR");
  if (ok) {
    log.println(F("設定しました。もう一方の機にも同じキーで実行してください"));
    log.println(F("（両方に同じキーを入れるまで通信できません）"));
  }
  return ok;
}

inline void im920Diag(Stream &log) {
  log.printf("BUSY  (GPIO%d) = %d  (L=コマンド受付可)\n", PIN_IM_BUSY,
             digitalRead(PIN_IM_BUSY));
  log.printf("RESET (GPIO%d) = %d  (解放中はモジュール内部プルアップで H)\n",
             PIN_IM_RESET, digitalRead(PIN_IM_RESET));
  log.printf("baud = %d, TX=GPIO0 RX=GPIO1\n", IM_BAUD);

  log.println(F("ハードリセットして 1.5 秒間の生バイトを見ます:"));
  im920Flush();
  digitalWrite(PIN_IM_RESET, LOW);
  pinMode(PIN_IM_RESET, OUTPUT);
  delay(20);
  pinMode(PIN_IM_RESET, INPUT);
  im920DumpRaw(log, 1500);
  log.printf("リセット後 BUSY = %d\n", digitalRead(PIN_IM_BUSY));

  log.println(F("BUSY を無視して RDVR を送ります:"));
  IM_SERIAL.print("RDVR\r\n");
  im920DumpRaw(log, 700);
  im920PinTest(log);
  im920LoopTest(log);
  im920GndTest(log);
  im920TxTest(log);
  im920ResetWatch(log);
  im920SwapTest(log);
  log.println(F("何も返らない場合: 電源(3V3/GND)・TxD/RxD の向き・"
                "変換アダプタの J1/J2 の配線・IO8〜IO10 がオープンか を確認"));
}

inline void im920Raw(Stream &log, const char *cmd) {
  im920Flush();
  IM_SERIAL.print(cmd);
  IM_SERIAL.print("\r\n");
  log.printf("> %s\n", cmd);
  im920DumpRaw(log, 700);
}

// ボーレート自動判別（SBRT で変更済みのモジュールを拾う）
// 全レートを試して、そのまま生バイトを表示する。ASCII らしい応答が返ったレートを返す
inline uint32_t im920ScanBaud(Stream &log) {
  static const uint32_t bauds[] = {19200, 115200, 57600, 38400, 9600, 4800, 2400};
  uint32_t found = 0;
  for (uint32_t b : bauds) {
    IM_SERIAL.end();
    IM_SERIAL.begin(b);
    delay(30);
    int best = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
      im920Flush();
      IM_SERIAL.print("RDVR\r\n");
      uint8_t buf[64];
      int n = 0;
      uint32_t t0 = millis();
      while (millis() - t0 < 300 && n < (int)sizeof(buf)) {
        while (IM_SERIAL.available() && n < (int)sizeof(buf)) {
          buf[n++] = (uint8_t)IM_SERIAL.read();
          t0 = millis();
        }
        im920Tick();
      }
      log.printf("%6lu bps 試行%d: %2d バイト ", (unsigned long)b, attempt + 1, n);
      bool ascii = n > 0;
      for (int i = 0; i < n; i++) {
        log.printf("%02X ", buf[i]);
        if (!((buf[i] >= 0x20 && buf[i] < 0x7F) || buf[i] == '\r' || buf[i] == '\n'))
          ascii = false;
      }
      if (n) {
        log.print(" \"");
        for (int i = 0; i < n; i++)
          log.print((buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.');
        log.print("\"");
      }
      log.println(ascii ? "  <- ASCII" : "");
      if (ascii && n > best) best = n;
    }
    if (best && !found) found = b;
  }
  IM_SERIAL.end();
  IM_SERIAL.begin(IM_BAUD);
  if (found) log.printf("ASCII らしい応答: %lu bps\n", (unsigned long)found);
  else       log.println(F("どのボーレートでもまともな応答なし"));
  return found;
}
