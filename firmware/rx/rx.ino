// rx.ino — 受信機（PC 側）
//   IM920sL で受けたパケットを、USB HID ゲームパッド（＋設定用 CDC）として PC に渡す。
//
//   HID ディスクリプタは、送信機から届く Info パケット（元パッドの軸の種類・ボタン数）
//   をもとに起動時に組み立てる。構成が変わったときは EEPROM に保存して自動で再起動する。
//   こうすると /dev/input/js0 の軸番号・ボタン番号が元のパッドと一致する。
//
//   ビルド: Seeed XIAO RP2040 / USB Stack = Adafruit TinyUSB
//   fqbn: rp2040:rp2040:seeed_xiao_rp2040:usbstack=tinyusb
#include "Adafruit_TinyUSB.h"
#include <EEPROM.h>
#include <pico/unique_id.h>

#include "packet.h"
#include "im920sl_config.h"
#include "hid_desc_builder.h"
#include "status_led.h"
#include "boot_button.h"
#include "im920_diag.h"

static const uint32_t FAILSAFE_MS  = 300;   // これだけ受信が途絶えたら中立に戻す
static const uint32_t HID_PERIOD_MS = 10;   // 変化がなくても送る HID 周期
// 半二重なので、受信機がハートビートを送っている間はパッドのパケットを受け取れない。
// 短い周期にすると前方向のパケット落ちが増えるため、既定は 2 秒にしてある。
static uint32_t g_hbPeriodMs = 2000;

// ---- EEPROM に持つパッド構成 ----
#define WG_PROFILE_ADDR  0
#define WG_PROFILE_MAGIC 0x57475031UL       // 'WGP1'
struct WgProfile {
  uint32_t magic;
  uint8_t  axisMask;
  uint8_t  buttonCount;
  uint8_t  caps;
  uint8_t  pad;
  uint16_t vid, pid;
};

static WgProfile    g_prof;
static WgHidLayout  g_layout;
static uint8_t      g_desc[WG_HID_DESC_MAX];
static uint16_t     g_descLen = 0;

Adafruit_USBD_HID usb_hid;
static WgStatusLed g_led;
static WgBootButton g_btn;
static Im920LineReader g_reader;

// ---- 受信状態 ----
static WgState  g_state;
static uint32_t g_lastRxMs = 0;     // 何かを受信した時刻（統計・表示用）
// フェイルセーフは「操作量を含む State パケット」だけで判断する。
// 2 秒周期の Info パケットでこれを更新してしまうと、State が途絶していても
// Info が 1 つ届くたびに 300ms だけフェイルセーフが解除され、
// **途絶直前の古い操作量（例: スティック全開）が再出力される**。
static uint32_t g_lastStateMs = 0;
static bool     g_failsafe = true;
static uint32_t g_lostAtMs = 0;
static int8_t   g_rssi = 0;
static uint32_t g_pktCount = 0, g_badCount = 0, g_dropCount = 0, g_lostCount = 0;
// 「電波で消えた」のか「受信機が取りこぼした」のかを切り分けるための計測
static uint32_t g_lineCount = 0;      // 行リーダが組み立てた行数
static uint32_t g_parseFail = 0;      // 受信行として解釈できなかった行
static uint32_t g_otherNode = 0;      // 送信機以外のノードからの行
static uint32_t g_uartOver  = 0;      // Serial1 の FIFO 溢れ（＝取りこぼし）
static uint32_t g_maxLoopMs = 0;      // loop() の最大間隔。22ms 超で FIFO が溢れうる
static uint32_t g_gapHist[7] = {0};   // seq の飛びの分布。均一なロスか瞬断かを見る
static uint32_t g_maxGap = 0;         // 最大の連続欠落数
static uint32_t g_slowLoops = 0;      // 10ms を超えた loop() の回数
static uint8_t  g_lastSeq = 0;
static bool     g_haveSeq = false;
static WgInfo   g_lastInfo;
static bool     g_haveInfo = false;
static uint8_t  g_infoStreak = 0;
static uint32_t g_hidSent = 0, g_hidSkip = 0, g_wakeReq = 0;
static uint32_t g_hbLastMs = 0, g_hbSent = 0, g_hbBusy = 0;
// 応答待ちなし送信なので、OK/NG 行はメインループの行読みが拾う。ここで数える
static uint32_t g_respOk = 0, g_respNg = 0;
// 送信タイミング: 受信直後に送るのが最も通る。
// 受信機は常時パケットを受けており、その最中はキャリアセンスが通らないため、
// 単純な周期送信では TXDA が 8 割以上 NG になる（実測 OK=1/NG=6）。
// 送信機がパケットを送り終えた直後はチャネルが空き、送信機も受信待ちなので確実。
static bool     g_hbDue = false;

// ---- チャンネル追従 ----
// 送信機の通知で移動し、一定時間 State が来なければ待ち合わせ ch31 へ戻る
#define WG_CH_HOME 31
static const uint32_t CH_REVERT_MS = 12000;
static uint8_t  g_curCh = WG_CH_HOME;
static uint8_t  g_hbSeq = 0;
static bool     g_hbEnable = true;
static uint8_t  g_hidLast[WG_HID_REPORT_MAX];
static uint8_t  g_hidLastLen = 0;

static const uint32_t WDT_MS = 4000;

static void ledTick() {
  g_led.task();
  rp2040.wdt_reset();
}

// =====================================================================
// プロファイル
// =====================================================================
static void profileDefault(WgProfile *p) {
  p->magic = WG_PROFILE_MAGIC;
  p->axisMask = 0x3F;                     // X, Y, Z, Rx, Ry, Rz
  p->buttonCount = WG_BUTTON_MAX;
  p->caps = WG_CAP_HAT;
  p->pad = 0;
  p->vid = p->pid = 0;
}

static void profileLoad() {
  EEPROM.get(WG_PROFILE_ADDR, g_prof);
  if (g_prof.magic != WG_PROFILE_MAGIC || (g_prof.axisMask & ~0x3F) ||
      g_prof.buttonCount > WG_BUTTON_MAX ||
      (g_prof.axisMask == 0 && g_prof.buttonCount == 0)) {
    profileDefault(&g_prof);
  }
}

static void profileSave() {
  g_prof.magic = WG_PROFILE_MAGIC;
  EEPROM.put(WG_PROFILE_ADDR, g_prof);
  EEPROM.commit();
}

// =====================================================================
// コンソール
// =====================================================================
static void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  help    このヘルプ"));
  Serial.println(F("  status  IM920sL の設定と受信状況を表示"));
  Serial.println(F("  config  AutoConfig（設定を目標値に合わせる）"));
  Serial.println(F("  pair    ペアリング（受信機＝親機）"));
  Serial.println(F("  bridge  USB CDC <-> IM920sL 透過ブリッジ（'+' を 3 回で抜ける）"));
  Serial.println(F("  profile 現在の HID 構成を表示"));
  Serial.println(F("  hid     HID レポートの送信状況と最後のレポート"));
  Serial.println(F("  clear   受信統計をリセット（測定用）"));
  Serial.println(F("  hb on|off|<ms>  送信機へのハートビートの切替と周期"));
  Serial.println(F("  reset   HID 構成を既定値に戻して再起動"));
  Serial.println(F("  diag    IM920sL の配線・応答を確認"));
  Serial.println(F("  raw X   BUSY を無視してコマンド X を送る（例: raw RDVR）"));
  Serial.println(F("  scan    ボーレートを変えて応答を探す"));
  Serial.println(F("  rscan   リセットしながらボーレートを変え、起動メッセージを探す"));
  Serial.println(F("  swap    TxD/RxD が逆接続でないかを調べる"));
  Serial.println(F("  ch N    チャネル変更 31〜45（EEPROM に保存。両機に同じ値を）"));
  Serial.println(F("  ttr N   キャリアセンス NG 時の自動リトライ回数（00〜10）"));
  Serial.println(F("  enc K   パケット暗号化＋送信元認証（16進64桁のキー / off）"));
  Serial.println(F("  gnd     IM920sL の GND が繋がっているかを調べる"));
  Serial.println(F("  reboot  再起動"));
}

static void printHid();
static void emitHeartbeat();
static void runPair();
static void runBridge();
static void sendHid(bool force);
static void forceNeutralHid();

static void printProfile() {
  static const char *names[WG_AXIS_COUNT] = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
  Serial.printf("profile: axisMask=0x%02X (", g_prof.axisMask);
  for (int a = 0; a < WG_AXIS_COUNT; a++)
    if (g_prof.axisMask & (1u << a)) Serial.printf("%s ", names[a]);
  Serial.printf(") buttons=%u hat=%u  src=%04X:%04X\n", g_prof.buttonCount,
                (g_prof.caps & WG_CAP_HAT) ? 1 : 0, g_prof.vid, g_prof.pid);
  Serial.printf("  HID report = %u bytes, descriptor = %u bytes\n",
                g_layout.reportLen, g_descLen);
}

static void printStatus() {
  Serial.printf("RDNN=%s RDRT=%s RDCH=%s RDPO=%s\n",
                im920Cmd("RDNN").c_str(), im920Cmd("RDRT").c_str(),
                im920Cmd("RDCH").c_str(), im920Cmd("RDPO").c_str());
  Serial.printf("RDGN=%s RDVR=%s%s\n",
                im920Cmd("RDGN").c_str(), im920Cmd("RDVR").c_str(),
                im920PairedRef() ? "" : "  ← 未ペアリング（送信できません）");
  g_reader.reset();                       // im920Cmd が RX を捨てるので作り直す
  Serial.printf("ch%u（待ち合わせ=%u）\n", g_curCh, WG_CH_HOME);
  Serial.printf("link: %s  RSSI=%ddBm  last=%ldms ago\n",
                g_failsafe ? "DOWN" : "UP", g_rssi,
                (long)(millis() - g_lastRxMs));
  Serial.printf("heartbeat: 送信=%lu 見送り(BUSY)=%lu  応答 OK=%lu NG=%lu\n",
                (unsigned long)g_hbSent, (unsigned long)g_hbBusy,
                (unsigned long)g_respOk, (unsigned long)g_respNg);
  Serial.printf("packets=%lu bad=%lu seqDrop=%lu lostEvents=%lu\n",
                (unsigned long)g_pktCount, (unsigned long)g_badCount,
                (unsigned long)g_dropCount, (unsigned long)g_lostCount);
  Serial.printf("UART: 生の行=%lu 組み立て=%lu 解釈不能=%lu 他ノード=%lu "
                "FIFO溢れ=%lu\n",
                (unsigned long)im920RawLines(), (unsigned long)g_lineCount,
                (unsigned long)g_parseFail, (unsigned long)g_otherNode,
                (unsigned long)g_uartOver);
  Serial.printf("連続欠落の分布: 途切れ無し=%lu / 1個=%lu / 2個=%lu / 3個=%lu / "
                "4-5個=%lu / 6-10個=%lu / 11個以上=%lu（最大 %lu 個）\n",
                (unsigned long)g_gapHist[0], (unsigned long)g_gapHist[1],
                (unsigned long)g_gapHist[2], (unsigned long)g_gapHist[3],
                (unsigned long)g_gapHist[4], (unsigned long)g_gapHist[5],
                (unsigned long)g_gapHist[6], (unsigned long)g_maxGap);
  Serial.printf("loop: 最大間隔=%lums 10ms超=%lu 回"
                "（22ms を超えると FIFO が溢れて行ごと落ちる）\n",
                (unsigned long)g_maxLoopMs, (unsigned long)g_slowLoops);
  Serial.printf("state: buttons=0x%04X hat=%u axes=%u,%u,%u,%u,%u,%u flags=0x%02X\n",
                g_state.buttons, g_state.hat, g_state.axis[0], g_state.axis[1],
                g_state.axis[2], g_state.axis[3], g_state.axis[4], g_state.axis[5],
                g_state.flags);
  printProfile();
  printHid();
}

static void printHid() {
  Serial.printf("HID: failsafe=%d 送信=%lu 回 送れず=%lu 回  最後のレポート(%u B):",
                g_failsafe ? 1 : 0, (unsigned long)g_hidSent,
                (unsigned long)g_hidSkip, g_hidLastLen);
  for (uint8_t i = 0; i < g_hidLastLen; i++) Serial.printf(" %02X", g_hidLast[i]);
  Serial.println();
  Serial.printf("  mounted=%d suspended=%d ready=%d 復帰要求=%lu 回\n",
                TinyUSBDevice.mounted() ? 1 : 0, TinyUSBDevice.suspended() ? 1 : 0,
                usb_hid.ready() ? 1 : 0, (unsigned long)g_wakeReq);
}

static void handleLine(char *line) {
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && line[n - 1] == ' ') line[--n] = '\0';
  if (!n) return;
  if (!strcasecmp(line, "help")) printHelp();
  else if (!strcasecmp(line, "status")) printStatus();
  else if (!strcasecmp(line, "config")) {
    int c = im920AutoConfig(&Serial);
    Serial.printf("AutoConfig -> %d\n", c);
    if (c > 0) Serial.println(im920HardReset());
    g_reader.reset();
  } else if (!strcasecmp(line, "pair")) runPair();
  else if (!strcasecmp(line, "bridge")) runBridge();
  else if (!strcasecmp(line, "profile")) printProfile();
  else if (!strcasecmp(line, "hid")) printHid();
  else if (!strcasecmp(line, "clear")) {
    g_pktCount = g_badCount = g_dropCount = g_lostCount = 0;
    g_lineCount = g_parseFail = g_otherNode = g_uartOver = 0;
    g_maxLoopMs = g_slowLoops = g_maxGap = 0;
    for (unsigned i = 0; i < sizeof(g_gapHist)/sizeof(g_gapHist[0]); i++) g_gapHist[i] = 0;
    im920RawLines() = 0;
    g_hidSent = g_hidSkip = g_wakeReq = g_hbSent = g_hbBusy = g_respOk = g_respNg = 0;
    g_haveSeq = false;
    Serial.println(F("統計をリセットしました"));
  }
  else if (!strncasecmp(line, "hb", 2)) {
    const char *arg = line + 2;
    while (*arg == ' ') arg++;
    if (!strcasecmp(arg, "on")) g_hbEnable = true;
    else if (!strcasecmp(arg, "off")) g_hbEnable = false;
    else if (*arg) {
      long v = strtol(arg, nullptr, 10);
      if (v >= 100 && v <= 10000) { g_hbPeriodMs = (uint32_t)v; g_hbEnable = true; }
    }
    Serial.printf("heartbeat = %s, 周期 %lu ms\n", g_hbEnable ? "on" : "off",
                  (unsigned long)g_hbPeriodMs);
  }
  else if (!strcasecmp(line, "reset")) {
    profileDefault(&g_prof);
    profileSave();
    Serial.println(F("HID 構成を既定値に戻しました。再起動します"));
    Serial.flush(); delay(100); rp2040.reboot();
  } else if (!strcasecmp(line, "diag")) im920Diag(Serial);
  else if (!strncasecmp(line, "raw ", 4)) im920Raw(Serial, line + 4);
  else if (!strcasecmp(line, "scan")) { forceNeutralHid(); im920ScanBaud(Serial); g_reader.reset(); }
  else if (!strcasecmp(line, "rscan")) { forceNeutralHid(); im920ScanReset(Serial); g_reader.reset(); }
  else if (!strcasecmp(line, "swap")) { forceNeutralHid(); im920SwapTest(Serial); g_reader.reset(); }
  else if (!strcasecmp(line, "gnd")) { forceNeutralHid(); im920GndTest(Serial); g_reader.reset(); }
  else if (!strncasecmp(line, "ch ", 3)) {
    int ch = atoi(line + 3);
    if (ch < 31 || ch > 45) Serial.println(F("チャンネルは 31〜45 で指定してください"));
    else if (im920SetChannelVolatile((uint8_t)ch)) {
      g_curCh = (uint8_t)ch;
      Serial.printf("ch%d に移動しました（揮発。再起動で %d に戻る）\n", ch, WG_CH_HOME);
    } else Serial.println(F("チャンネル変更に失敗しました"));
  } else if (!strncasecmp(line, "ttr ", 4)) {
    im920SetParam(Serial, "STTR", line + 4);
    Serial.printf("RDTR -> %s\n", im920Cmd("RDTR").c_str());
  }
  else if (!strcasecmp(line, "txtest")) im920TxTest(Serial);
  else if (!strncasecmp(line, "enc", 3)) {
    const char *arg = line + 3;
    while (*arg == ' ') arg++;
    if (!strcasecmp(arg, "off")) im920SetEncryption(Serial, nullptr);
    else if (strlen(arg) == 64) im920SetEncryption(Serial, arg);
    else {
      Serial.println(F("enc <16進64桁>  : 暗号化キーを設定して有効化"));
      Serial.println(F("enc off         : 暗号化を無効化（キーは残る）"));
      Serial.println(F("※両機に同じキーを入れるまで通信できません"));
      Serial.println(F("※1 パケットあたり通信時間が 24 バイト分（約1.9ms）増えます"));
    }
  }
  else if (!strcasecmp(line, "reboot")) { Serial.flush(); delay(50); rp2040.reboot(); }
  else Serial.printf("unknown: %s（help でコマンド一覧）\n", line);
}

static void pollConsole() {
  static char buf[128];      // enc コマンドは "enc " + 16進64桁 = 68 文字
  static size_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = '\0'; n = 0; handleLine(buf); continue; }
    if (n < sizeof(buf) - 1) buf[n++] = c;
    else n = 0;
  }
}

// =====================================================================
// 受信
// =====================================================================
static void applyInfo(const WgInfo *info) {
  if (!g_haveInfo || !wgInfoEqual(info, &g_lastInfo)) {
    g_lastInfo = *info;
    g_haveInfo = true;
    g_infoStreak = 1;
    return;
  }
  if (g_infoStreak < 255) g_infoStreak++;
  bool same = (info->axisMask == g_prof.axisMask) &&
              (info->buttonCount == g_prof.buttonCount) &&
              ((info->caps & WG_CAP_HAT) == (g_prof.caps & WG_CAP_HAT));
  if (same || g_infoStreak < 3) return;    // 同じ内容を 3 回受けてから反映する

  Serial.printf("パッド構成が変わりました: axisMask 0x%02X->0x%02X buttons %u->%u hat %u->%u\n",
                g_prof.axisMask, info->axisMask, g_prof.buttonCount, info->buttonCount,
                (g_prof.caps & WG_CAP_HAT) ? 1 : 0, (info->caps & WG_CAP_HAT) ? 1 : 0);
  g_prof.axisMask    = info->axisMask;
  g_prof.buttonCount = info->buttonCount;
  g_prof.caps        = info->caps;
  g_prof.vid         = info->vid;
  g_prof.pid         = info->pid;
  profileSave();
  Serial.println(F("HID ディスクリプタを作り直すため再起動します"));
  Serial.flush();
  delay(200);
  rp2040.reboot();
}

static void pollRadio() {
  while (g_reader.poll()) {
    uint16_t node = 0;
    int8_t   rssi = 0;
    uint8_t  data[40];
    size_t   len = 0;
    g_lineCount++;
    if (!strcmp(g_reader.line, "OK")) { g_respOk++; continue; }
    if (!strcmp(g_reader.line, "NG")) { g_respNg++; continue; }
    if (!im920ParseRx(g_reader.line, node, rssi, data, sizeof(data), len)) {
      g_parseFail++; continue;
    }
    if (node != WG_NODE_TX) { g_otherNode++; continue; }

    if (len == WG_STATE_LEN) {
      WgState s;
      if (!wgStateDecode(data, len, &s)) { g_badCount++; continue; }
      if (g_haveSeq) {
        uint8_t gap = (uint8_t)(s.seq - g_lastSeq);
        if (gap > 1) g_dropCount += (uint32_t)(gap - 1);
        uint8_t miss = (uint8_t)(gap - 1);          // 連続で落ちた数
        if (miss > g_maxGap) g_maxGap = miss;
        int b = miss == 0 ? 0 : miss == 1 ? 1 : miss == 2 ? 2 : miss == 3 ? 3
              : miss <= 5 ? 4 : miss <= 10 ? 5 : 6;
        g_gapHist[b]++;
      }
      g_lastSeq = s.seq;
      g_haveSeq = true;
      g_state = s;
      g_rssi = rssi;
      g_lastRxMs = g_lastStateMs = millis();
      g_pktCount++;
      // 送信機が送り終えた直後＝チャネルが空いている。ここで返すと確実に通る
      if (g_hbEnable && g_hbDue) emitHeartbeat();
    } else if (len == WG_CHCMD_LEN) {
      uint8_t ch;
      if (!wgChCmdDecode(data, len, &ch)) { g_badCount++; continue; }
      g_lastRxMs = millis();
      g_pktCount++;
      if (ch != g_curCh) {
        Serial.printf("[%lu] 送信機の指示で ch%u へ移動します\n",
                      (unsigned long)millis(), ch);
        if (im920SetChannelVolatile(ch)) {
          g_curCh = ch;
          g_reader.reset();
        } else {
          Serial.println(F("チャンネル移動に失敗しました"));
        }
      }
    } else if (len == WG_INFO_LEN) {
      WgInfo info;
      if (!wgInfoDecode(data, len, &info)) { g_badCount++; continue; }
      g_rssi = rssi;
      g_lastRxMs = millis();              // ← g_lastStateMs は更新しない（上記の理由）
      g_pktCount++;
      applyInfo(&info);
    } else {
      g_badCount++;
    }
  }
}

// 送信機がリンクの生死を判断できるように、定期的に投げ返す。
// 応答を待たない（HID の処理を止めないため）。OK/NG 行は行読みが拾って無視する。
// 実際に送る（受信直後、またはフォールバックで呼ばれる）
static void emitHeartbeat() {
  g_hbLastMs = millis();
  g_hbDue = false;
  WgHeartbeat h;
  h.seq   = g_hbSeq++;
  h.rssi  = g_rssi;
  h.flags = g_failsafe ? 0 : WG_HB_RXOK;   // 前方向が生きているかを送信機に伝える
  uint8_t buf[WG_HB_LEN];
  wgHbEncode(&h, buf);
  // 応答待ちなし送信（im920SendNoWait）では TXDA が常に NG になった。
  // BUSY が L でも、直前の受信処理が終わっていないと受け付けられないとみられる。
  // ブロッキング送信は 115200bps なら通常数 ms で終わり、2 秒に 1 回なので影響は小さい。
  int r = im920SendEx(buf, sizeof(buf));
  if (r == IM_SEND_OK) { g_hbSent++; g_respOk++; }
  else if (r == IM_SEND_NG) { g_respNg++; }
  else { g_hbBusy++; }
}

static void sendHeartbeat() {
  if (!g_hbEnable) return;
  uint32_t now = millis();
  if (now - g_hbLastMs >= g_hbPeriodMs) g_hbDue = true;
  // 受信が途絶えているときは受信直後を待てないので、周期の 1.5 倍でフォールバック送信
  if (g_hbDue && (now - g_hbLastMs) >= g_hbPeriodMs + g_hbPeriodMs / 2) emitHeartbeat();
}

// =====================================================================
// HID 出力
// =====================================================================
static void sendHid(bool force) {
  static uint8_t last[WG_HID_REPORT_MAX];
  static bool    haveLast = false;
  static uint32_t lastMs = 0;

  if (!TinyUSBDevice.mounted()) { g_hidSkip++; return; }

  WgState out = g_state;
  if (g_failsafe || !(g_state.flags & WG_FLAG_PAD)) wgStateNeutral(&out);

  uint8_t buf[WG_HID_REPORT_MAX];
  uint8_t n = wgBuildHidReport(&g_layout, &out, buf);
  uint32_t now = millis();
  bool changed = !haveLast || memcmp(buf, last, n) != 0;

  // ホストがセレクティブサスペンドで休止させているとき、操作があれば復帰を要求する。
  // これを出さないと、しばらく放置したあと操作しても HID レポートが流れないままになる
  // （Windows は放置した HID デバイスを休止させる。こちらは bmAttributes で
  //   リモートウェイクアップ対応を宣言しているので、要求を出す義務がある）。
  if (TinyUSBDevice.suspended()) {
    if (changed) { TinyUSBDevice.remoteWakeup(); g_wakeReq++; }
    g_hidSkip++;
    return;
  }

  if (!usb_hid.ready()) { g_hidSkip++; return; }
  if (!changed && !force && (now - lastMs) < HID_PERIOD_MS) return;

  usb_hid.sendReport(0, buf, n);
  g_hidSent++;
  memcpy(g_hidLast, buf, n);
  g_hidLastLen = n;
  memcpy(last, buf, n);
  haveLast = true;
  lastMs = now;
}

// BOOTSEL ボタンの扱い（詳細は boot_button.h）
//  起動直後の受付ウィンドウ（LED = 水色の速い点滅）と、
//  実行中はリンクが切れているとき（LED = 赤点滅）だけ受け付ける
static const uint32_t BTN_WINDOW_MS = 5000;
static const uint32_t BTN_LONG_MS   = 3000;

// ブロッキング処理（bridge/pair/診断）の間は loop() が回らず sendHid() も呼ばれない。
// その瞬間の非中立レポートが PC 側に残り続けるので、入る前に必ず中立を出す。
static void forceNeutralHid() {
  g_failsafe = true;
  wgStateNeutral(&g_state);
  sendHid(true);
}

static void runPair() {
  forceNeutralHid();
  WgLedMode prev = g_led.mode();
  g_led.set(WG_LED_PAIRING);
  Serial.println(F("pairing (parent): 子機と 50cm 以内に置いてください"));
  bool ok = im920Pair(true, 15000, &Serial);
  Serial.println(ok ? F("pairing done") : F("pairing NG"));
  g_reader.reset();
  g_led.set(prev);
}

static void runBridge() {
  forceNeutralHid();
  WgLedMode prev = g_led.mode();
  Serial.println(F("bridge mode（'+' を 3 回続けて送ると抜ける）"));
  g_led.set(WG_LED_BRIDGE);
  g_led.task();
  im920Bridge();
  g_reader.reset();
  g_led.set(prev);
}

static void doButtonAction(WgButtonAction a) {
  if (a == WG_BTN_LONG) {
    im920AutoConfig(&Serial);
    g_reader.reset();
    runPair();
  } else if (a == WG_BTN_SHORT) {
    runBridge();
  }
}

static void bootButtonWindow() {
  g_led.set(WG_LED_BUTTON);
  uint32_t t0 = millis();
  bool pressed = false;
  while (millis() - t0 < BTN_WINDOW_MS) {
    g_led.task();
    if (BOOTSEL) { pressed = true; break; }
    delay(10);
  }
  if (!pressed) return;
  g_led.set(WG_LED_PAIRING);
  uint32_t held = wgBootselHold(ledTick);
  doButtonAction(held >= BTN_LONG_MS ? WG_BTN_LONG : WG_BTN_SHORT);
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  wgStateNeutral(&g_state);

  EEPROM.begin(256);
  profileLoad();

  g_descLen = wgBuildHidDescriptor(g_desc, sizeof(g_desc), g_prof.axisMask,
                                   g_prof.buttonCount,
                                   (g_prof.caps & WG_CAP_HAT) != 0, &g_layout);
  if (!g_descLen) {                       // 念のため既定値で作り直す
    profileDefault(&g_prof);
    g_descLen = wgBuildHidDescriptor(g_desc, sizeof(g_desc), g_prof.axisMask,
                                     g_prof.buttonCount, true, &g_layout);
  }

  // VID:PID を既定の 239a:cafe（TinyUSB のサンプル用 ID）から変える。
  // あの ID は世の中に大量に出回っており、Windows は VID/PID 単位で
  // ドライバの割り当てをキャッシュするため、過去に別の 239a:cafe 機器
  // （CDC のみのものなど）を見た PC では複合デバイスとして展開されず、
  // COM ポートだけが出て HID が作られないことがある。
  // 1209:0001 は pid.codes が試作・試験用に開放している ID（配布用ではない）。
  // 元に戻すには次の行をコメントアウトする。
  TinyUSBDevice.setID(0x1209, 0x0001);

  TinyUSBDevice.setManufacturerDescriptor("irlab");
  TinyUSBDevice.setProductDescriptor("Wireless Gamepad");

  // 送信機と受信機で同じシリアル番号が返る個体があり、Windows は
  // VID/PID/シリアルでインスタンスを識別するため、役割の接頭辞を付けて区別する
  static char serial[8 + 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = "WGRX-";
  pico_get_unique_board_id_string(serial + 5, sizeof(serial) - 5);
  TinyUSBDevice.setSerialDescriptor(serial);

  // Windows は HID レポートディスクリプタを VID/PID/REV 単位でキャッシュする。
  // プロファイル学習でディスクリプタが変わったときや、ディスクリプタの作り方を
  // 変えたときに古い定義が使われないよう、構成と形式版から bcdDevice を作る。
  //   bit15-8: axisMask / bit7-6: 形式版 / bit5-1: ボタン数 / bit0: ハット有無
  TinyUSBDevice.setDeviceVersion((uint16_t)((g_prof.axisMask << 8) |
                                            (WG_HID_DESC_FORMAT << 6) |
                                            ((g_prof.buttonCount & 0x1F) << 1) |
                                            ((g_prof.caps & WG_CAP_HAT) ? 1 : 0)));
#ifdef WG_HID_ONLY
  // 切り分け用: CDC を外して HID 単独のデバイスにする。
  // これで Windows がゲームパッドとして認識するなら、複合デバイスの扱いが原因。
  // ※CDC が無くなるので 1200bps タッチでの書込みができない。
  //   書き直すときは BOOT を押しながら RESET でブートローダに入れること。
  TinyUSBDevice.clearConfiguration();
#endif
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(g_desc, g_descLen);
  // usb_hid.setStringDescriptor() は使わない。
  // 設定しても文字列テーブルに載らず、iInterface が「存在しない文字列」を
  // 指す状態になってしまう（lsusb で空欄）。Windows は列挙中に文字列を
  // 取得しに行くため、これが失敗するとインタフェースが起動しないことがある。
  // iInterface = 0（文字列なし）が常に安全。
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {          // 既に列挙済みなら付け直して反映させる
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  g_led.begin();
  g_led.set(WG_LED_BOOT);
  im920SetTick(ledTick);

  im920Begin();
  String boot = im920HardReset();
  im920SetupBaud(&Serial);          // 現在のボーレートを判別し、可能なら 115200 に上げる

  bootButtonWindow();

  if (boot.length()) Serial.println(boot);
  int changed = im920AutoConfig(&Serial);
  if (changed > 0) Serial.println(im920HardReset());
  else if (changed < 0) g_led.set(WG_LED_ERROR);
  g_reader.reset();

  rp2040.wdt_begin(WDT_MS);
  Serial.println(F("wireless gamepad RX ready（help でコマンド一覧）"));
  printProfile();
  g_lastRxMs = g_lastStateMs = millis() - FAILSAFE_MS;
}

void loop() {
  rp2040.wdt_reset();
  {   // loop() が長く止まると UART の FIFO（256B ＝ 115200bps で約 22ms）が溢れる
    static uint32_t lastLoopMs = 0;
    uint32_t t = millis(), d = t - lastLoopMs;
    if (lastLoopMs && d > g_maxLoopMs) g_maxLoopMs = d;
    if (lastLoopMs && d > 10) g_slowLoops++;
    lastLoopMs = t;
    if (IM_SERIAL.overflow()) g_uartOver++;   // 読むとフラグはクリアされる
  }
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  pollConsole();
  pollRadio();

  uint32_t now = millis();
  bool lost = (now - g_lastStateMs) > FAILSAFE_MS || !(g_state.flags & WG_FLAG_PAD);
  if (lost && !g_failsafe) {               // 途絶の開始
    g_failsafe = true;
    g_lostAtMs = now;
    g_lostCount++;
    wgStateNeutral(&g_state);              // 古い操作量を残さない
    Serial.printf("[%lu] link lost（中立を出力します）\n", (unsigned long)now);
  } else if (!lost && g_failsafe) {        // 復帰
    g_failsafe = false;
    Serial.printf("[%lu] link up（途絶 %lu ms, RSSI=%ddBm）\n", (unsigned long)now,
                  (unsigned long)(now - g_lostAtMs), g_rssi);
  }

  // 移動先で受信が途絶えたら待ち合わせチャンネルへ戻る
  if (g_curCh != WG_CH_HOME && (now - g_lastStateMs) > CH_REVERT_MS) {
    Serial.printf("[%lu] ch%u で受信が無いため ch%u に戻ります\n",
                  (unsigned long)now, g_curCh, WG_CH_HOME);
    if (im920SetChannelVolatile(WG_CH_HOME)) g_curCh = WG_CH_HOME;
    g_reader.reset();
    g_lastStateMs = now;                    // すぐに再判定しない
  }

  if (g_led.mode() != WG_LED_BRIDGE && g_led.mode() != WG_LED_PAIRING)
    g_led.set(g_failsafe ? WG_LED_DISCONNECTED : WG_LED_CONNECTED);
  g_led.task();
  sendHeartbeat();

  // 赤点滅（リンクなし）のときだけ BOOTSEL を読む
  doButtonAction(g_btn.poll(g_led.mode() == WG_LED_DISCONNECTED, BTN_LONG_MS, ledTick));

  sendHid(false);
}
