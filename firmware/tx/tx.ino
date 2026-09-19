// tx.ino — 送信機（ゲームパッド側）
//   XIAO RP2040 が PIO-USB で USB ゲームパッドのホストになり、
//   状態を IM920sL（920MHz）で受信機へ送る。
//
//   コア1: PIO-USB ホスト（D+ = GPIO27, D- = GPIO26。実配線に合わせて DMDP）
//   コア0: IM920sL への送信と、USB-C の CDC コンソール
//
//   ビルド: Seeed XIAO RP2040 / USB Stack = Adafruit TinyUSB / CPU 120MHz
//           （build.sh を使うか、下記 FQBN で arduino-cli compile）
//   fqbn: rp2040:rp2040:seeed_xiao_rp2040:usbstack=tinyusb,freq=120
#define IM_ROLE_TX

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include <pico/multicore.h>
#include <EEPROM.h>
#include <pico/unique_id.h>
#include <pico/platform.h>

#include "packet.h"
#include "im920sl_config.h"
#include "hid_gamepad_map.h"
#include "status_led.h"
#include "boot_button.h"
#include "wg_dbg.h"
#include "im920_diag.h"

// ---- USB-A の D+/D- の向き ----
// 回路図 Rev.2 は「GPIO27 = D+, GPIO26 = D-」だが、実配線が逆の個体もあるため
// 起動時に自動判別する（機器のプルアップで H になっている側が D+）。
// 手で決めるときは CDC コンソールの `usbpin 26` / `usbpin 27` / `usbpin auto`。
#define PIN_USB_HOST_DP_DEFAULT  27
#define TX_CFG_ADDR   0
#define TX_CFG_MAGIC  0x57475431UL   // 'WGT1'
// noNeo: NeoPixel を使わない（三色 LED だけで状態表示する）。
// NeoPixel と PIO-USB は PIO を共有するため、core1 停止の切り分け用に切れるようにした。
struct TxCfg { uint32_t magic; uint8_t dpPin; uint8_t noNeo; uint8_t rsv[2]; };
// ウォッチドッグ／ソフトリセットでは RAM の内容が残るので、
// 「異常再起動だった」ことを次の起動に伝えてボタン受付ウィンドウを飛ばす。
#define WG_FASTBOOT_MAGIC 0x57474642UL   // 'WGFB'
static __uninitialized_ram(uint32_t) g_fastBootFlag;

// ---- ブラックボックス ----
// 赤点滅や自動再起動が起きても、再起動でカウンタが消えて原因が追えなかった。
// __uninitialized_ram はソフトリセットでは保持されるので、ここに記録を残す。
// （電源を抜くと magic が壊れてクリアされる＝手動の電源断と自動再起動を区別できる）
#define WG_BB_MAGIC 0x57474242UL         // 'WGBB'
enum {                                   // 再起動・赤点滅の理由
  WG_R_NONE = 0, WG_R_CORE1, WG_R_DESC, WG_R_MANUAL,
  WG_R_PAD, WG_R_HOST, WG_R_HB, WG_R_RXDOWN,
};
typedef struct {
  uint32_t magic;
  uint32_t boots;           // ソフト再起動の回数
  uint32_t rebootReason;    // 最後の自動再起動の理由
  uint32_t upMs;            // そのときの稼働時間
  uint32_t reports, ok, ng; // そのときの主要カウンタ
  uint32_t hbAgeMs;         // 最後にハートビートを受けてからの経過
  uint32_t redCount;        // 赤点滅に落ちた回数（累積）
  uint32_t redPad, redHost, redHb, redRx;   // その原因別の内訳
  uint32_t lastRedMs;       // 直近に赤へ落ちた時刻
  uint32_t savedCh;         // 自動再起動の直前に使っていたチャンネル
  // core1 が固まった瞬間の PIO-USB の状態（根本原因を追うための証拠）
  uint32_t stallInts, stallEpErr, stallEpStall;
  uint8_t  stallInit, stallConn, stallFs, stallSusp;
  uint8_t  stallStage, stallNeedArm, stallDevAddr, stallValid;
} WgBlackBox;   // __uninitialized_ram はマクロ引数をセクション名に使うので
                // 空白を含まない 1 語の型名が必要
static __uninitialized_ram(WgBlackBox) g_bb;

static TxCfg   g_cfg;
static uint8_t g_usbDp = PIN_USB_HOST_DP_DEFAULT;
static const char *g_usbDpSrc = "default";

// ---- 送信のタイミング ----
// 4s モード（ch31〜45）は「送信休止時間 52ms 以上」が電波法上の規定で、
// 満たさないと 3.9 秒ごとに強制休止が入り TXDA が NG を返す（取説 Rev.1.5 §7-3）。
// 実測した NG 率（12 バイト・各 80〜150 回、キャリアセンス約 6ms ＋ 送信 6ms）:
//   55ms=20%  65ms=17.5%  75ms=12.5%  90ms=8%  120ms=4%  150ms=0%
// NG は一度も連続しなかった（90/120ms で全て単発）ので、欠落は最悪 2 周期分。
// 90ms なら 180ms で、受信側フェイルセーフ 300ms に余裕がある。
// ※ 暗号化を有効にすると 1 パケットが 24 バイト増えるので、その場合は要再測定。
// NG は単発でしか起きないことが実測で分かったので（次の周期で回復し、受信側
// フェイルセーフ 300ms には届かない）、間隔を延ばすより詰めたほうが実効レートが高い。
// 90ms に延ばした実測では NG 率 9.2%→7.5% と引き換えに
// 実効更新レートが 8.8/s→6.5/s まで落ちたため、30/60ms に戻した。
static uint32_t TX_MIN_INTERVAL = 30;    // 最短送信間隔 [ms]（rate コマンドで変更可）
static uint32_t TX_KEEPALIVE    = 60;    // 変化がなくても送る間隔 [ms]
static const uint32_t TX_INFO_PERIOD  = 2000;  // Info パケットの間隔 [ms]
// 送信機のモジュールは半二重で、自分が送信している間は受信できない。
// 実測（2026-09-19）では送信間隔によってハートビートの取りこぼしが大きく変わる:
//   送信 60ms（パッド静止）… 欠落 6.6%
//   送信 30ms（操作中）  … 欠落 29.5%
// ハートビートは 2 秒周期なので、5 秒では 2 回連続の欠落（操作中で 8.7%）で
// リンク断と誤判定していた。平均 20〜30 秒に 1 回、動作中に突然赤点滅する原因。
// 15 秒あれば 7 回連続の欠落が必要になり（0.02%）、実用上起きない。
// ※ ロボットの保護は受信側の 300ms フェイルセーフが行うので、
//    送信側のリンク判定を鈍くしても安全性は落ちない。
static const uint32_t HB_TIMEOUT_MS   = 15000;

extern "C" void wgPioState(int *initialized, int *connected, int *fullspeed,
                           int *suspended, unsigned long *ints,
                           unsigned long *epError, unsigned long *epStalled);
extern "C" void wgPioForceReconnect(void);

Adafruit_USBH_Host USBHost;
static WgStatusLed g_led;
static WgBootButton g_btn;

// ---- コア間で共有する状態（mutex で保護）----
static mutex_t   g_mtx;
static WgState   g_state;            // 最新のパッド状態
static WgInfo    g_info;             // パッドの構成
static bool      g_infoValid = false;
static bool      g_padConnected = false;
static uint8_t   g_raw[64];          // dump 用の生レポート
static uint8_t   g_rawLen = 0;
static uint32_t  g_rawSeq = 0;
static uint32_t  g_reportCount = 0;

// ---- USB ホストの記録（コア1 で積み、コア0 で表示する）----
struct UsbEvent {
  uint8_t  kind;          // 1=device mount 2=device umount 3=HID mount 4=HID umount 5=HID 非対応
  uint8_t  addr, inst;
  uint16_t vid, pid;
  uint16_t descLen;
  uint8_t  axisMask, buttons, hat;
};
static UsbEvent g_ev[12];
static uint8_t  g_evHead = 0, g_evTail = 0;
static uint8_t  g_descBuf[256];
static uint16_t g_descBufLen = 0;
static uint32_t g_mountCount = 0;

static void pushEvent(const UsbEvent &e) {
  mutex_enter_blocking(&g_mtx);
  uint8_t next = (uint8_t)((g_evHead + 1) % (uint8_t)(sizeof(g_ev) / sizeof(g_ev[0])));
  if (next != g_evTail) { g_ev[g_evHead] = e; g_evHead = next; }
  mutex_exit(&g_mtx);
}

// ---- コア1 だけが触る ----
static WgHidMap  g_map;
static uint8_t   g_devAddr = 0, g_devInst = 0;

// ---- コア0 だけが触る ----
static volatile bool g_core1Go = false;
static bool     g_dump = false;
static uint32_t g_okCount = 0, g_ngCount = 0, g_shortReports = 0;
static uint32_t g_ngBusy = 0, g_ngResp = 0, g_ngNo = 0;
static uint32_t g_txMinInterval = TX_MIN_INTERVAL;   // rate コマンドで変更
static uint32_t g_lastOkMs = 0;
static uint32_t g_rawSeqShown = 0;
static uint32_t g_lastHbMs = 0, g_hbCount = 0;
// コア1（USB ホスト）の生存監視。
// コア1 が死んでもコア0 は動き続けるため、ウォッチドッグは餌をもらって発火しない。
// その間パッドは「接続中」のまま中立値が送られ、LED も緑のままになってしまう
// （実機で発生）。遠隔操縦では最悪の故障モードなので、明示的に見張る。
static uint32_t g_hostTickSeen = 0, g_hostTickMs = 0;
static bool     g_hostAlive = true;
static uint32_t g_hostStallCount = 0;
static uint32_t g_lockFail = 0;

// ---- チャンネル選定（起動直後に 1 回だけ）----
// ch31 は待ち合わせ用。起動時は AutoConfig が必ず 31 に戻す（Flash の既定値）。
// リンクが確立したら 31〜45 を走査し、最も静かなチャンネルへ受信機ごと移動する。
// 通知が届かず片方だけ移動しても、双方「一定時間受信が無ければ 31 へ戻る」ので復帰できる。
#define WG_CH_HOME      31
#define WG_CH_LAST      45
// 待ち合わせ（ch31）に戻る判定もハートビート頼りなので、同じ理由で延ばす。
// 12 秒＝6 回連続の欠落（操作中で 0.07%／2 秒ごと）で誤って再走査に入り、
// 受信機を取り残して本当の通信断を作っていた。30 秒なら 15 回連続が必要。
static const uint32_t CH_REVERT_MS = 30000;  // これだけ応答が無ければ待ち合わせに戻る
static const int      CH_MAX_TRIES = 3;      // 移動を試みる回数の上限

static uint8_t  g_curCh = WG_CH_HOME;
static bool     g_chScanned = false;
static bool     g_fastBootRecover = false;   // 異常再起動からの復帰で起動したか
static int      g_chTries = 0;
static int8_t   g_chRssi[WG_CH_LAST - WG_CH_HOME + 1];
static const uint32_t HOST_STALL_MS  = 1000;   // これだけ進まなければ異常とみなす
// core1 は常時回っているので、数秒止まっていれば確実に死んでいる。
// 10 秒待つと、再起動と再走査を合わせて実測 21 秒の通信断になっていた（2026-09-19）。
static const uint32_t HOST_REBOOT_MS = 3000;   // これだけ続いたら再起動して復帰を試みる
static int8_t   g_hbRssiHere = 0;    // 送信機が見たハートビートの RSSI
static int8_t   g_hbRssiThere = 0;   // 受信機が見ている RSSI（ハートビートの中身）
static bool     g_rxReceiving = false;  // 受信機が「前方向を受信できている」と言っているか
// ハートビートがどこで落ちているかの切り分け用
static uint32_t g_rxLines = 0, g_rxLinesParsed = 0, g_rxFromRx = 0, g_hbBad = 0;
static uint8_t  g_seq = 0;

// ---- 列挙の経過を RAM のリングバッファに溜める ----
// USB ホストのコールバックはコア1 で呼ばれるので Serial を直接触らず、
// ここに溜めて CDC コンソールの `log` コマンドで読む。
// （TinyUSB 本体の CFG_TUSB_DEBUG は log_printf が Serial1＝IM920sL に出るので使わない）
#define WG_LOG_SIZE 12288
static char     g_logBuf[WG_LOG_SIZE];
static volatile uint16_t g_logW = 0;
static volatile bool     g_logWrapped = false;
// ログを CDC に出すと、その通信自体が TinyUSB デバイス側のログを生んで
// バッファを流してしまう。コンソール処理の間はデバイススタックのログを止める。
static volatile bool     g_logMute = false;

static void wgLogPut(const char *s, int n) {
  for (int i = 0; i < n; i++) {
    g_logBuf[g_logW] = s[i];
    g_logW = (uint16_t)((g_logW + 1) % WG_LOG_SIZE);
    if (g_logW == 0) g_logWrapped = true;
  }
}

// TinyUSB のデバッグ出力の受け皿（wg_dbg.h 参照）
size_t WgDbgStream::write(uint8_t c) {
  if (!g_logMute) wgLogPut((const char *)&c, 1);
  return 1;
}
size_t WgDbgStream::write(const uint8_t *buf, size_t n) {
  if (!g_logMute) wgLogPut((const char *)buf, (int)n);
  return n;
}
WgDbgStream wgDbgStream;

static int wgLog(const char *fmt, ...) {
  char tmp[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  if (n > 0) wgLogPut(tmp, n > (int)sizeof(tmp) - 1 ? (int)sizeof(tmp) - 1 : n);
  return n;
}

static void printLog() {
  g_logMute = true;
  Serial.println(F("---- log ----"));
  if (g_logWrapped)
    for (uint16_t i = g_logW; i < WG_LOG_SIZE; i++) Serial.write(g_logBuf[i]);
  for (uint16_t i = 0; i < g_logW; i++) Serial.write(g_logBuf[i]);
  Serial.println(F("\n---- end ----"));
  Serial.flush();
  g_logMute = false;
}

static const uint32_t WDT_MS = 4000;      // これだけループが止まったら再起動

static void ledTick() {
  g_led.task();
  rp2040.wdt_reset();                      // ブロッキング処理中も餌やりする
}

// 送信処理の合間に退避された受信行を拾う
static void pollHeartbeat() {
  char line[96];
  while (im920StashTake(line, sizeof(line))) {
    g_rxLines++;
    uint16_t node = 0;
    int8_t   rssi = 0;
    uint8_t  data[40];
    size_t   len = 0;
    if (!im920ParseRx(line, node, rssi, data, sizeof(data), len)) continue;
    g_rxLinesParsed++;
    if (node != WG_NODE_RX) continue;
    g_rxFromRx++;
    WgHeartbeat h;
    if (!wgHbDecode(data, len, &h)) { g_hbBad++; continue; }
    g_lastHbMs = millis();
    g_hbCount++;
    g_hbRssiHere = rssi;
    g_hbRssiThere = h.rssi;
    g_rxReceiving = (h.flags & WG_HB_RXOK) != 0;
  }
}

static char g_lastBadResp[40] = "";

// 各チャンネルの雑音を測る。値が大きい（0 に近い）ほど混んでいる
static uint8_t scanChannels() {
  Serial.println(F("空きチャンネルを探しています..."));
  uint8_t best = WG_CH_HOME;
  int8_t  bestRssi = 0;
  for (uint8_t ch = WG_CH_HOME; ch <= WG_CH_LAST; ch++) {
    if (!im920SetChannelVolatile(ch)) continue;
    int8_t worst = -128;
    for (int i = 0; i < 6; i++) {          // 一瞬の空きに騙されないよう最悪値を採る
      int8_t v = im920ReadRssi();
      if (v > worst) worst = v;
      delay(8);
      ledTick();
    }
    g_chRssi[ch - WG_CH_HOME] = worst;
    if (ch == WG_CH_HOME || worst < bestRssi) { bestRssi = worst; best = ch; }
  }
  im920SetChannelVolatile(WG_CH_HOME);      // 通知のため待ち合わせに戻る
  Serial.print(F("  RSSI: "));
  for (uint8_t ch = WG_CH_HOME; ch <= WG_CH_LAST; ch++)
    Serial.printf("%u:%d ", ch, g_chRssi[ch - WG_CH_HOME]);
  Serial.printf("\n  選択: ch%u (%ddBm)\n", best, bestRssi);
  return best;
}

// 移動先を数回通知してから自分も移動する
static void announceAndMove(uint8_t ch) {
  uint8_t buf[WG_CHCMD_LEN];
  wgChCmdEncode(ch, buf);
  for (int i = 0; i < 6; i++) {
    im920SendEx(buf, sizeof(buf));
    delay(40);
    ledTick();
  }
  if (im920SetChannelVolatile(ch)) {
    g_curCh = ch;
    Serial.printf("ch%u へ移動しました\n", ch);
  } else {
    Serial.println(F("チャンネル移動に失敗しました"));
  }
}

static void countSend(int r) {
  if (r == IM_SEND_NG) {
    strncpy(g_lastBadResp, im920LastResponse().c_str(), sizeof(g_lastBadResp) - 1);
    g_lastBadResp[sizeof(g_lastBadResp) - 1] = '\0';
  }
  switch (r) {
    case IM_SEND_OK:     g_okCount++; g_lastOkMs = millis(); break;
    case IM_SEND_BUSY:   g_ngBusy++;  g_ngCount++; break;
    case IM_SEND_NG:     g_ngResp++;  g_ngCount++; break;
    default:             g_ngNo++;    g_ngCount++; break;
  }
}

// 線の静電容量をおおまかに測る（断線の切り分け用）
//   一度 L に落としてから内部プルアップで離し、H になるまでのループ回数を数える。
//   USB ケーブル＋機器がぶら下がっていれば 50〜100pF ついて回数が大きくなる。
//   基板パターンだけ（断線）なら数 pF で、ほぼ即座に H になる。
static uint32_t wgRiseCount(uint8_t pin) {
  uint32_t sum = 0;
  const int N = 32;                        // 分解能を上げるため合計で返す
  for (int k = 0; k < N; k++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(300);
    uint32_t n = 0;
    noInterrupts();
    pinMode(pin, INPUT_PULLUP);
    while (!digitalRead(pin) && n < 200000) n++;
    interrupts();
    pinMode(pin, INPUT);
    sum += n;
    delayMicroseconds(300);
  }
  return sum;
}

static uint32_t g_rise26 = 0, g_rise27 = 0, g_riseOpen = 0;
static void measureUsbLines() {
  g_rise26   = wgRiseCount(26);
  g_rise27   = wgRiseCount(27);
  g_riseOpen = wgRiseCount(6);     // 回路図で未接続のピン（基準）
}

// 機器のプルアップで H になっている側が D+。判別できなければ 0
// PIO-USB を起動する前（＝誰も線を駆動していない状態）に 1 回だけ実行する
static int g_p26pd = -1, g_p27pd = -1, g_p26pu = -1, g_p27pu = -1;
static uint8_t detectDpPin() {
  pinMode(26, INPUT_PULLDOWN);
  pinMode(27, INPUT_PULLDOWN);
  delay(20);
  g_p26pd = digitalRead(26); g_p27pd = digitalRead(27);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  delay(20);
  g_p26pu = digitalRead(26); g_p27pu = digitalRead(27);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  if (g_p26pd && !g_p27pd) return 26;      // 26 側だけが強く H = そこに機器のプルアップ
  if (g_p27pd && !g_p26pd) return 27;
  return 0;
}

// バスが動いているか（PIO-USB がリセットや列挙を試みていれば変化する）
static void printLines(uint8_t a, uint8_t b, bool inv) {
  uint32_t t0 = micros();
  uint32_t nA = 0, nB = 0, hiA = 0, hiB = 0, n = 0;
  int la = inv ? !gpio_get(a) : gpio_get(a), lb = inv ? !gpio_get(b) : gpio_get(b);
  while (micros() - t0 < 200000) {
    int va = inv ? !gpio_get(a) : gpio_get(a), vb = inv ? !gpio_get(b) : gpio_get(b);
    if (va != la) { nA++; la = va; }
    if (vb != lb) { nB++; lb = vb; }
    hiA += (uint32_t)va; hiB += (uint32_t)vb; n++;
  }
  Serial.printf("  200ms の観測: GPIO%u 変化%lu回 H率%lu%%  GPIO%u 変化%lu回 H率%lu%%\n",
                a, (unsigned long)nA, (unsigned long)(hiA * 100 / n),
                b, (unsigned long)nB, (unsigned long)(hiB * 100 / n));
}

static void cfgLoad() {
  EEPROM.get(TX_CFG_ADDR, g_cfg);
  if (g_cfg.magic != TX_CFG_MAGIC ||
      (g_cfg.dpPin != 0 && g_cfg.dpPin != 26 && g_cfg.dpPin != 27)) {
    g_cfg.magic = TX_CFG_MAGIC;
    g_cfg.dpPin = 0;                       // 0 = 自動判別
    g_cfg.noNeo = 0;
    memset(g_cfg.rsv, 0, sizeof(g_cfg.rsv));
  }
}

static void cfgSave() {
  g_cfg.magic = TX_CFG_MAGIC;
  EEPROM.put(TX_CFG_ADDR, g_cfg);
  EEPROM.commit();
}

// =====================================================================
// コア1: PIO-USB ホスト
// =====================================================================
static void __not_in_flash_func(core1WaitForGo)() {
  while (!g_core1Go) { tight_loop_contents(); }
}

static volatile uint32_t g_hostTicks = 0;
static volatile uint8_t  g_hostStage = 0;   // 1=待機解除 2=configure 済 3=begin 済

void setup1() {
  core1WaitForGo();                       // 起動直後の BOOTSEL 判定が終わるまで待つ
  g_hostStage = 1;
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = g_usbDp;
  // D+ = GPIO27 なら D- = 26（DMDP）、D+ = GPIO26 なら D- = 27（DPDM）
  pio_cfg.pinout = (g_usbDp == 27) ? PIO_USB_PINOUT_DMDP : PIO_USB_PINOUT_DPDM;
  USBHost.configure_pio_usb(1, &pio_cfg);
  g_hostStage = 2;
  USBHost.begin(1);
  g_hostStage = 3;
}

// tuh_hid_receive_report() が失敗したまま放置すると、以後レポートが一切
// 届かなくなる。失敗したら次のループで再試行する。
static volatile bool g_needArm = false;
static volatile uint32_t g_needArmSinceMs = 0;
static uint32_t g_armFail = 0;

static void armReport(uint8_t addr, uint8_t inst) {
  if (!tuh_hid_receive_report(addr, inst)) {
    g_armFail++;
    if (!g_needArm) g_needArmSinceMs = millis();
    g_needArm = true;
  } else {
    g_needArm = false;
  }
}

void loop1() {
  g_hostTicks++;
  USBHost.task();
  if (g_needArm && g_devAddr) {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last >= 10) { last = now; armReport(g_devAddr, g_devInst); }
  }
}

extern "C" {

// 列挙の途中で呼ばれる（コア1）。
// ⚠ ここで printf 系（vsnprintf）を呼ばないこと。newlib はコア間で再入可能ではなく、
//   コア0 側の printf と競合するとハングしうる。イベントキュー経由でコア0 に渡す。
void tuh_enum_descriptor_device_cb(uint8_t daddr, const tusb_desc_device_t *d) {
  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 6; e.addr = daddr;
  e.vid = d->idVendor; e.pid = d->idProduct;
  e.axisMask = d->bDeviceClass; e.buttons = d->bMaxPacketSize0;
  pushEvent(e);
}

bool tuh_enum_descriptor_configuration_cb(uint8_t daddr, uint8_t cfg_index,
                                          const tusb_desc_configuration_t *c) {
  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 7; e.addr = daddr; e.inst = cfg_index;
  e.descLen = c->wTotalLength; e.buttons = c->bNumInterfaces;
  pushEvent(e);
  return true;
}

void tuh_mount_cb(uint8_t dev_addr) {
  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 1; e.addr = dev_addr;
  tuh_vid_pid_get(dev_addr, &e.vid, &e.pid);
  pushEvent(e);
  mutex_enter_blocking(&g_mtx);
  g_mountCount++;
  mutex_exit(&g_mtx);
}

void tuh_umount_cb(uint8_t dev_addr) {
  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 2; e.addr = dev_addr;
  pushEvent(e);
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len) {
  {   // ディスクリプタを丸ごと控えておく（desc コマンドで表示）
    mutex_enter_blocking(&g_mtx);
    uint16_t n = desc_len > sizeof(g_descBuf) ? (uint16_t)sizeof(g_descBuf) : desc_len;
    if (desc_report && n) memcpy(g_descBuf, desc_report, n);
    g_descBufLen = desc_report ? n : 0;
    mutex_exit(&g_mtx);
  }
  if (g_devAddr) {                        // 既に 1 台つかんでいる
    armReport(dev_addr, instance);
    return;
  }
  WgHidMap m;
  memset(&m, 0, sizeof(m));
  bool ok = desc_report && desc_len && wgHidParseDescriptor(desc_report, desc_len, &m);
  if (!ok || (!m.valid && m.buttonCount == 0)) {
    UsbEvent e; memset(&e, 0, sizeof(e));
    e.kind = 5; e.addr = dev_addr; e.inst = instance; e.descLen = desc_len;
    tuh_vid_pid_get(dev_addr, &e.vid, &e.pid);
    pushEvent(e);
    armReport(dev_addr, instance);   // ゲームパッドではない
    return;
  }
  tuh_vid_pid_get(dev_addr, &m.vid, &m.pid);
  g_map = m;
  g_devAddr = dev_addr;
  g_devInst = instance;

  WgInfo info;
  wgHidToInfo(&g_map, &info);
  mutex_enter_blocking(&g_mtx);
  g_info = info;
  g_infoValid = true;
  g_padConnected = true;
  wgStateNeutral(&g_state);
  g_state.flags = WG_FLAG_PAD;
  mutex_exit(&g_mtx);

  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 3; e.addr = dev_addr; e.inst = instance; e.descLen = desc_len;
  e.vid = m.vid; e.pid = m.pid;
  e.axisMask = info.axisMask; e.buttons = info.buttonCount;
  e.hat = (uint8_t)((info.caps & WG_CAP_HAT) ? 1 : 0);
  pushEvent(e);

  armReport(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  if (dev_addr != g_devAddr || instance != g_devInst) return;
  g_devAddr = 0;
  memset(&g_map, 0, sizeof(g_map));
  mutex_enter_blocking(&g_mtx);
  wgStateNeutral(&g_state);              // 外れたら中立＋flags=0
  g_padConnected = false;
  g_infoValid = false;
  mutex_exit(&g_mtx);
  UsbEvent e; memset(&e, 0, sizeof(e));
  e.kind = 4; e.addr = dev_addr; e.inst = instance;
  pushEvent(e);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len) {
  if (dev_addr == g_devAddr && instance == g_devInst && g_map.valid) {
    const uint8_t *data = report;
    uint16_t dlen = len;
    bool okId = true;
    if (g_map.reportId) {                 // レポート ID 付きなら先頭 1 バイトが ID
      if (len < 1 || report[0] != g_map.reportId) okId = false;
      else { data = report + 1; dlen = (uint16_t)(len - 1); }
    }
    if (okId) {
      WgState s;
      wgStateNeutral(&s);
      // 宣言より短いレポートは信用しない（中立のまま、パッド未接続として扱う）
      bool ok = wgHidExtract(&g_map, data, dlen, &s);
      s.flags = ok ? WG_FLAG_PAD : 0;
      if (!ok) g_shortReports++;
      mutex_enter_blocking(&g_mtx);
      uint8_t n = (uint8_t)(len > sizeof(g_raw) ? sizeof(g_raw) : len);
      memcpy(g_raw, report, n);
      g_rawLen = n;
      g_rawSeq++;
      g_reportCount++;
      s.seq = g_state.seq;
      g_state = s;
      mutex_exit(&g_mtx);
    }
  }
  armReport(dev_addr, instance);
}

} // extern "C"

// =====================================================================
// コア0: IM920sL 送信 + CDC コンソール
// =====================================================================
static const char *wgReasonName(uint32_t r) {
  switch (r) {
    case WG_R_CORE1:  return "コア1(PIO-USB)の停止";
    case WG_R_DESC:   return "ディスクリプタ変更による再起動";
    case WG_R_MANUAL: return "コンソールからの reboot";
    case WG_R_PAD:    return "パッド切断";
    case WG_R_HOST:   return "USB ホスト停止";
    case WG_R_HB:     return "ハートビート途絶";
    case WG_R_RXDOWN: return "受信機が前方向を受信できていない";
    default:          return "なし";
  }
}
static void wgBbPrint(Stream &o) {
  o.printf("ブラックボックス: ソフト再起動 %lu 回 / 赤点滅 %lu 回\n",
           (unsigned long)g_bb.boots, (unsigned long)g_bb.redCount);
  if (g_bb.redCount)
    o.printf("  赤点滅の内訳: パッド切断=%lu USBホスト停止=%lu "
             "ハートビート途絶=%lu 受信機側=%lu（最後は %lums 時点）\n",
             (unsigned long)g_bb.redPad, (unsigned long)g_bb.redHost,
             (unsigned long)g_bb.redHb, (unsigned long)g_bb.redRx,
             (unsigned long)g_bb.lastRedMs);
  if (g_bb.stallValid)
    o.printf("  停止時の PIO-USB: initialized=%u connected=%u fullspeed=%u suspended=%u "
             "ints=0x%08lX epErr=0x%08lX epStall=0x%08lX / stage=%u needArm=%u addr=%u\n",
             g_bb.stallInit, g_bb.stallConn, g_bb.stallFs, g_bb.stallSusp,
             (unsigned long)g_bb.stallInts, (unsigned long)g_bb.stallEpErr,
             (unsigned long)g_bb.stallEpStall, g_bb.stallStage,
             g_bb.stallNeedArm, g_bb.stallDevAddr);
  if (g_bb.rebootReason)
    o.printf("  直近の自動再起動: %s（稼働 %lums, reports=%lu ok=%lu ng=%lu, "
             "ハートビート %lums 前）\n",
             wgReasonName(g_bb.rebootReason), (unsigned long)g_bb.upMs,
             (unsigned long)g_bb.reports, (unsigned long)g_bb.ok,
             (unsigned long)g_bb.ng, (unsigned long)g_bb.hbAgeMs);
}

static void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  help    このヘルプ"));
  Serial.println(F("  status  IM920sL の設定とパッドの状態を表示"));
  Serial.println(F("  config  AutoConfig（設定を目標値に合わせる）"));
  Serial.println(F("  pair    ペアリング（送信機＝子機）"));
  Serial.println(F("  bridge  USB CDC <-> IM920sL 透過ブリッジ（'+' を 3 回で抜ける）"));
  Serial.println(F("  dump    HID 生レポートの表示を切り替え"));
  Serial.println(F("  map     解析した HID マップを表示"));
  Serial.println(F("  rate N  送信間隔を N ms にして統計をリセット（既定 30ms。64ms 未満は規定違反）"));
  Serial.println(F("  neo on|off  NeoPixel の使用を切り替えて再起動（core1 停止の切り分け用）"));
  Serial.println(F("  usb     USB ホストの状態"));
  Serial.println(F("  usbpin  USB-A の D+ ピンを指定（auto|26|27、保存して再起動）"));
  Serial.println(F("  desc    取得した HID レポートディスクリプタを表示"));
  Serial.println(F("  log     USB スタックのログを表示"));
  Serial.println(F("  usbretry 抜き差しせずに USB の接続検出をやり直す"));
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

static void printStatus() {
  Serial.printf("RDNN=%s RDRT=%s RDCH=%s RDPO=%s\n",
                im920Cmd("RDNN").c_str(), im920Cmd("RDRT").c_str(),
                im920Cmd("RDCH").c_str(), im920Cmd("RDPO").c_str());
  Serial.printf("RDGN=%s RDVR=%s%s\n",
                im920Cmd("RDGN").c_str(), im920Cmd("RDVR").c_str(),
                im920PairedRef() ? "" : "  ← 未ペアリング（送信できません）");
  mutex_enter_blocking(&g_mtx);
  bool conn = g_padConnected;
  WgInfo info = g_info;
  bool iv = g_infoValid;
  uint32_t rc = g_reportCount;
  WgState st = g_state;
  mutex_exit(&g_mtx);
  Serial.printf("pad: %s  reports=%lu\n", conn ? "connected" : "disconnected",
                (unsigned long)rc);
  if (iv) {
    Serial.printf("  VID:PID=%04X:%04X axisMask=0x%02X buttons=%u hat=%u\n",
                  info.vid, info.pid, info.axisMask, info.buttonCount,
                  (info.caps & WG_CAP_HAT) ? 1 : 0);
  }
  Serial.printf("  buttons=0x%04X hat=%u axes=%u,%u,%u,%u,%u,%u\n",
                st.buttons, st.hat, st.axis[0], st.axis[1], st.axis[2],
                st.axis[3], st.axis[4], st.axis[5]);
  uint32_t total = g_okCount + g_ngCount;
  Serial.printf("tx: ok=%lu ng=%lu (%lu%%) lastOk=%ldms ago  送信間隔=%lums\n",
                (unsigned long)g_okCount, (unsigned long)g_ngCount,
                (unsigned long)(total ? g_ngCount * 100 / total : 0),
                (long)(millis() - g_lastOkMs), (unsigned long)g_txMinInterval);
  Serial.printf("  NG の内訳: BUSY が L にならない=%lu / モジュールが NG=%lu / 無応答=%lu\n",
                (unsigned long)g_ngBusy, (unsigned long)g_ngResp, (unsigned long)g_ngNo);
  if (g_lastBadResp[0]) Serial.printf("  直近の想定外応答: \"%s\"\n", g_lastBadResp);
  bool hbOk = g_hbCount && (millis() - g_lastHbMs) < HB_TIMEOUT_MS;
  Serial.printf("ch%u（待ち合わせ=%u 走査=%d 回）\n", g_curCh, WG_CH_HOME, g_chTries);
  Serial.printf("link: %s（受信機が前方向を受信中=%d）応答=%lu 回 最終=%ldms 前\n",
                (hbOk && g_rxReceiving) ? "UP" : "DOWN", g_rxReceiving ? 1 : 0,
                (unsigned long)g_hbCount,
                g_hbCount ? (long)(millis() - g_lastHbMs) : -1L);
  Serial.printf("  UART 行=%lu 退避=%lu / 取出し=%lu 解析成功=%lu 受信機から=%lu HB 復号失敗=%lu\n",
                (unsigned long)im920RawLines(), (unsigned long)im920StashPuts(),
                (unsigned long)g_rxLines, (unsigned long)g_rxLinesParsed,
                (unsigned long)g_rxFromRx, (unsigned long)g_hbBad);
  if (g_hbCount)
    wgBbPrint(Serial);
  Serial.printf("  RSSI: こちらで受信=%ddBm / 受信機側で受信=%ddBm\n",
                  g_hbRssiHere, g_hbRssiThere);
}

static void printUsb();
static void printDesc();
static void printLog();

static void printMap() {
  if (!g_map.valid) { Serial.println(F("HID map: none")); return; }
  Serial.printf("HID map: reportId=%u bits=%u gamepadApp=%u buttons=%u@%u\n",
                g_map.reportId, g_map.reportBits, g_map.gamepadApp,
                g_map.buttonCount, g_map.buttonOffset);
  static const char *names[WG_AXIS_COUNT] = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
  for (int a = 0; a < WG_AXIS_COUNT; a++) {
    if (!g_map.axis[a].present) continue;
    Serial.printf("  %-2s off=%3u size=%u range=%ld..%ld\n", names[a],
                  g_map.axis[a].bitOffset, g_map.axis[a].bitSize,
                  (long)g_map.axis[a].lmin, (long)g_map.axis[a].lmax);
  }
  if (g_map.hat.present)
    Serial.printf("  hat off=%3u size=%u range=%ld..%ld\n", g_map.hat.bitOffset,
                  g_map.hat.bitSize, (long)g_map.hat.lmin, (long)g_map.hat.lmax);
}

static void runPair() {
  WgLedMode prev = g_led.mode();
  g_led.set(WG_LED_PAIRING);
  Serial.println(F("pairing (child): 親機と 50cm 以内に置いてください"));
  bool ok = im920Pair(false, 30000, &Serial);
  Serial.println(ok ? F("pairing OK") : F("pairing NG"));
  g_led.set(ok ? prev : WG_LED_ERROR);
}

static void handleLine(char *line) {
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ')) line[--n] = '\0';
  if (!n) return;
  if (!strcasecmp(line, "help")) printHelp();
  else if (!strcasecmp(line, "status")) printStatus();
  else if (!strcasecmp(line, "config")) {
    int c = im920AutoConfig(&Serial);
    Serial.printf("AutoConfig -> %d\n", c);
    if (c > 0) Serial.println(im920HardReset());
  } else if (!strcasecmp(line, "pair")) runPair();
  else if (!strcasecmp(line, "bridge")) {
    Serial.println(F("bridge mode（'+' を 3 回続けて送ると抜ける）"));
    WgLedMode prevMode = g_led.mode();
    g_led.set(WG_LED_BRIDGE);
    im920Bridge();
    g_led.set(prevMode);
  } else if (!strcasecmp(line, "dump")) {
    g_dump = !g_dump;
    Serial.printf("dump %s\n", g_dump ? "on" : "off");
    if (g_dump) printMap();
  } else if (!strncasecmp(line, "usbpin", 6)) {
    const char *arg = line + 6;
    while (*arg == ' ') arg++;
    if (!strcasecmp(arg, "auto")) g_cfg.dpPin = 0;
    else if (!strcmp(arg, "26"))  g_cfg.dpPin = 26;
    else if (!strcmp(arg, "27"))  g_cfg.dpPin = 27;
    else { Serial.printf("usbpin auto|26|27（今は %d, %s）\n", g_usbDp, g_usbDpSrc); return; }
    cfgSave();
    Serial.println(F("保存しました。再起動します"));
    Serial.flush(); delay(100); rp2040.reboot();
  } else if (!strncasecmp(line, "rate", 4)) {
    const char *arg = line + 4;
    while (*arg == ' ') arg++;
    if (*arg) {
      long v = strtol(arg, nullptr, 10);
      if (v >= 10 && v <= 1000) {
        g_txMinInterval = TX_KEEPALIVE = (uint32_t)v;   // 下限とキープアライブを揃える
        g_okCount = g_ngCount = g_ngBusy = g_ngResp = g_ngNo = 0;   // 統計をリセット
        if (v < 64)
          Serial.println(F("※52ms の送信休止規定を満たさないので NG が増えます"));
      } else Serial.println(F("rate は 10〜1000 ms"));
    }
    Serial.printf("送信間隔 = %lu ms（統計はリセット済み）\n", (unsigned long)g_txMinInterval);
  } else if (!strcasecmp(line, "map")) printMap();
  else if (!strcasecmp(line, "usb")) printUsb();
  else if (!strcasecmp(line, "desc")) printDesc();
  else if (!strcasecmp(line, "log")) printLog();
  else if (!strcasecmp(line, "usbretry")) {
    wgLog("[%lu] usbretry\n", (unsigned long)millis());
    wgPioForceReconnect();
    Serial.println(F("接続検出をやり直しました。数秒後に log / usb を見てください"));
  }
  else if (!strcasecmp(line, "diag")) im920Diag(Serial);
  else if (!strncasecmp(line, "raw ", 4)) im920Raw(Serial, line + 4);
  else if (!strcasecmp(line, "scan")) im920ScanBaud(Serial);
  else if (!strcasecmp(line, "rscan")) im920ScanReset(Serial);
  else if (!strcasecmp(line, "swap")) im920SwapTest(Serial);
  else if (!strcasecmp(line, "gnd")) im920GndTest(Serial);
  else if (!strncasecmp(line, "ch ", 3)) {
    int ch = atoi(line + 3);
    if (ch < WG_CH_HOME || ch > WG_CH_LAST) Serial.println(F("チャンネルは 31〜45"));
    else { announceAndMove((uint8_t)ch); g_chScanned = true; }   // 受信機にも通知する
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
  else if (!strncasecmp(line, "neo", 3)) {
    const char *arg = line + 3;
    while (*arg == ' ') arg++;
    if (!strcasecmp(arg, "on") || !strcasecmp(arg, "off")) {
      g_cfg.noNeo = !strcasecmp(arg, "off") ? 1 : 0;
      cfgSave();
      Serial.printf("NeoPixel を %s にしました。再起動します\n", arg);
      g_bb.rebootReason = WG_R_MANUAL; g_bb.upMs = millis();
      Serial.flush(); delay(100); rp2040.reboot();
    }
    Serial.printf("NeoPixel = %s（PIO を 1 つ消費する。core1 停止の切り分け用）\n",
                  g_cfg.noNeo ? "off" : "on");
  }
  else if (!strcasecmp(line, "reboot")) {
    g_bb.rebootReason = WG_R_MANUAL; g_bb.upMs = millis();
    Serial.flush(); rp2040.reboot();
  }
  else Serial.printf("unknown: %s（help でコマンド一覧）\n", line);
}

static void pollConsole() {
  static char buf[128];      // enc コマンドは "enc " + 16進64桁 = 68 文字
  static size_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = '\0'; n = 0;
      g_logMute = true;                  // コマンドの出力でバッファを汚さない
      handleLine(buf);
      Serial.flush();
      g_logMute = false;
      continue;
    }
    if (n < sizeof(buf) - 1) buf[n++] = c;
    else n = 0;
  }
}

// PIO-USB のルートポートの状態が変わったらログに残す（抜き差し時の経過を見る）
static void pollPioState() {
  static uint32_t lastMs = 0;
  static int pc = -1, pf = -1, ps = -1;
  uint32_t now = millis();
  if (now - lastMs < 5) return;
  lastMs = now;
  int init, conn, fs, susp;
  unsigned long ints, epErr, epStall;
  wgPioState(&init, &conn, &fs, &susp, &ints, &epErr, &epStall);
  if (conn != pc || fs != pf || susp != ps) {
    wgLog("[%lu] port connected=%d fullspeed=%d suspended=%d\n",
          (unsigned long)now, conn, fs, susp);
    pc = conn; pf = fs; ps = susp;
  }
}

static void pollEvents() {
  for (;;) {
    UsbEvent e;
    mutex_enter_blocking(&g_mtx);
    bool has = (g_evTail != g_evHead);
    if (has) {
      e = g_ev[g_evTail];
      g_evTail = (uint8_t)((g_evTail + 1) % (uint8_t)(sizeof(g_ev) / sizeof(g_ev[0])));
    }
    mutex_exit(&g_mtx);
    if (!has) return;
    switch (e.kind) {
      case 1: Serial.printf("[USB] device mount addr=%u VID:PID=%04X:%04X\n",
                            e.addr, e.vid, e.pid);
              wgLog("[%lu] mount addr=%u %04X:%04X\n", (unsigned long)millis(),
                    e.addr, e.vid, e.pid); break;
      case 2: Serial.printf("[USB] device umount addr=%u\n", e.addr); break;
      case 3: Serial.printf("[USB] HID mount addr=%u inst=%u %04X:%04X desc=%uB "
                            "axisMask=0x%02X buttons=%u hat=%u\n",
                            e.addr, e.inst, e.vid, e.pid, e.descLen,
                            e.axisMask, e.buttons, e.hat); break;
      case 4: Serial.printf("[USB] HID umount addr=%u inst=%u\n", e.addr, e.inst); break;
      case 5: Serial.printf("[USB] HID mount addr=%u inst=%u %04X:%04X desc=%uB "
                            "-> ゲームパッドとして解釈できず\n",
                            e.addr, e.inst, e.vid, e.pid, e.descLen); break;
      case 6: Serial.printf("[enum] daddr=%u devDesc %04X:%04X class=%u ep0=%u\n",
                            e.addr, e.vid, e.pid, e.axisMask, e.buttons);
              wgLog("[enum] daddr=%u devDesc %04X:%04X class=%u ep0=%u\n",
                    e.addr, e.vid, e.pid, e.axisMask, e.buttons); break;
      case 7: Serial.printf("[enum] daddr=%u cfg[%u] len=%u itf=%u\n",
                            e.addr, e.inst, e.descLen, e.buttons);
              wgLog("[enum] daddr=%u cfg[%u] len=%u itf=%u\n",
                    e.addr, e.inst, e.descLen, e.buttons); break;
    }
  }
}

static void printUsb() {
  mutex_enter_blocking(&g_mtx);
  uint32_t mc = g_mountCount;
  uint32_t rc = g_reportCount;
  uint16_t dl = g_descBufLen;
  bool conn = g_padConnected;
  mutex_exit(&g_mtx);
  Serial.printf("  レポート再アーム失敗=%lu 回 再試行待ち=%d  短いレポート=%lu 回\n",
                (unsigned long)g_armFail, g_needArm ? 1 : 0,
                (unsigned long)g_shortReports);
  Serial.printf("USB host: mounts=%lu  HID reports=%lu  pad=%s\n",
                (unsigned long)mc, (unsigned long)rc, conn ? "connected" : "none");
  uint8_t dm = (g_usbDp == 27) ? 26 : 27;
  Serial.printf("  PIO-USB: D+=GPIO%d, D-=GPIO%d (%s, %s), CPU=%lu Hz\n",
                g_usbDp, dm, (g_usbDp == 27) ? "DMDP" : "DPDM", g_usbDpSrc,
                (unsigned long)clock_get_hz(clk_sys));
  Serial.printf("  保持しているレポートディスクリプタ: %u バイト（desc で表示）\n", dl);
  uint32_t t1 = g_hostTicks;
  delay(50);
  // ベアメタル（osal_none）では osal_queue_receive が即 return するので、
  // USBHost.task() はブロックしない。直近 50ms で 0 回なら core1 の停止を疑う
  Serial.printf("  コア1: 生存=%d 停止検出=%lu 回 最終更新=%ldms 前 ロック取得失敗=%lu 回\n",
                g_hostAlive ? 1 : 0, (unsigned long)g_hostStallCount,
                (long)(millis() - g_hostTickMs), (unsigned long)g_lockFail);
  Serial.printf("  コア1: stage=%u（3=begin 済）loop1 通算 %lu 回, 直近 50ms で %lu 回"
                "（0 回なら core1 停止を疑う）\n",
                g_hostStage, (unsigned long)g_hostTicks,
                (unsigned long)(g_hostTicks - t1));
  // 注意: PIO-USB は入力に GPIO_OVERRIDE_INVERT を掛けるので、
  // ホスト開始後の gpio_get() は反転している
  bool inv = (g_hostStage >= 3);
  int lvDp = inv ? !gpio_get(g_usbDp) : gpio_get(g_usbDp);
  int lvDm = inv ? !gpio_get(dm)      : gpio_get(dm);
  Serial.printf("  D+ (GPIO%d)=%d  D- (GPIO%d)=%d   ", g_usbDp, lvDp, dm, lvDm);
  if (lvDp && !lvDm)       Serial.println(F("-> フルスピード機器が接続されている"));
  else if (!lvDp && lvDm)  Serial.println(F("-> ロースピード機器、または D+/D- が逆"));
  else if (!lvDp && !lvDm) Serial.println(F("-> 機器なし（両方 L）"));
  else                     Serial.println(F("-> 両方 H（配線か VBUS を確認）"));
  Serial.printf("  起動時（PIO-USB 開始前）の測定: プルダウン時 GPIO26=%d GPIO27=%d / "
                "プルアップ時 GPIO26=%d GPIO27=%d\n", g_p26pd, g_p27pd, g_p26pu, g_p27pu);
  Serial.println(F("   （プルダウンでも H になる側に機器のプルアップがある = そこが D+）"));
  Serial.printf("  線の静電容量の目安（32 回の合計。大きいほどケーブル・機器が"
                "ぶら下がっている）: GPIO26=%lu GPIO27=%lu 未接続の GPIO6=%lu\n",
                (unsigned long)g_rise26, (unsigned long)g_rise27,
                (unsigned long)g_riseOpen);
  if (g_rise26 && g_rise27) {
    uint32_t lo = g_rise26 < g_rise27 ? g_rise26 : g_rise27;
    uint32_t hi = g_rise26 > g_rise27 ? g_rise26 : g_rise27;
    if (lo <= g_riseOpen + 8 && hi > (g_riseOpen + 8) * 3)
      Serial.printf("   → GPIO%d 側が未接続のピンと同程度。D%s 線の断線（R%d 22Ω の"
                    "はんだ不良など）を疑う\n",
                    (g_rise26 < g_rise27) ? 26 : 27,
                    (g_rise26 < g_rise27) ? "-" : "+",
                    (g_rise26 < g_rise27) ? 1 : 2);
  }
  Serial.println(F("   ※ パッドを抜いて reboot → usb を実行し、GPIO26 の値が変わらなければ"));
  Serial.println(F("      その線はパッドまで届いていない（直列抵抗が大きい/断線）"));
  printLines(26, 27, inv);
  int init = 0, pioConn = 0, fs = 0, susp = 0;
  unsigned long ints = 0, epErr = 0, epStall = 0;
  wgPioState(&init, &pioConn, &fs, &susp, &ints, &epErr, &epStall);
  Serial.printf("  PIO-USB ルートポート: initialized=%d connected=%d fullspeed=%d "
                "suspended=%d ints=0x%08lX epErr=0x%08lX epStall=0x%08lX\n",
                init, pioConn, fs, susp, ints, epErr, epStall);
  if (pioConn && mc == 0)
    Serial.println(F("  → PIO-USB は接続を検出しているが列挙が完了していない。"
                     "USB-A の VBUS(5V)/GND が本当に繋がっているか、"
                     "R1/R2 22Ω と C1 47µF まわり、ケーブル長を確認"));
  Serial.println(F("  D+/D- が逆のときは usbpin 26 / usbpin 27 で切り替え（保存して再起動）"));
  if (mc == 0)
    Serial.println(F("  デバイスが 1 台も列挙されていない。USB-A の VBUS(5V)・D+/D- の配線・"
                     "CPU 120MHz を確認"));
}

static void printDesc() {
  mutex_enter_blocking(&g_mtx);
  uint16_t n = g_descBufLen;
  static uint8_t tmp[256];
  memcpy(tmp, g_descBuf, n);
  mutex_exit(&g_mtx);
  if (!n) { Serial.println(F("レポートディスクリプタ未取得")); return; }
  Serial.printf("HID report descriptor (%u bytes):\n", n);
  for (uint16_t i = 0; i < n; i++) {
    Serial.printf("%02X ", tmp[i]);
    if ((i % 16) == 15) Serial.println();
  }
  Serial.println();
}

static void pollDump() {
  if (!g_dump) return;
  mutex_enter_blocking(&g_mtx);
  uint32_t seq = g_rawSeq;
  uint8_t len = g_rawLen;
  uint8_t raw[64];
  memcpy(raw, g_raw, len);
  mutex_exit(&g_mtx);
  if (seq == g_rawSeqShown) return;
  g_rawSeqShown = seq;
  Serial.printf("HID[%2u]:", len);
  for (uint8_t i = 0; i < len; i++) Serial.printf(" %02X", raw[i]);
  Serial.println();
}

// 起動直後の BOOTSEL 受付ウィンドウ（LED = 水色の速い点滅）
// ※電源投入時に押しっぱなしだと書込みモードに入るので「起動後」に押してもらう
//   ここではまだ PIO-USB を開始していないので、BOOTSEL の読み取りは完全に安全
static const uint32_t BTN_WINDOW_MS = 5000;   // 受付ウィンドウ
static const uint32_t BTN_LONG_MS   = 3000;   // これ以上の長押しでペアリング

static void runBridge() {
  WgLedMode prev = g_led.mode();
  Serial.println(F("bridge mode（'+' を 3 回続けて送ると抜ける）"));
  g_led.set(WG_LED_BRIDGE);
  g_led.task();
  im920Bridge();
  g_led.set(prev);
}

static void doButtonAction(WgButtonAction a) {
  if (a == WG_BTN_LONG) {
    im920AutoConfig(&Serial);
    runPair();
  } else if (a == WG_BTN_SHORT) {
    runBridge();
  }
}

static void bootButtonWindow() {
  if (g_fastBootFlag == WG_FASTBOOT_MAGIC) {   // 異常再起動からの復帰は待たない
    g_fastBootFlag = 0;
    g_fastBootRecover = true;
    Serial.println(F("異常再起動からの復帰のため、ボタン受付をとばします"));
    return;
  }
  g_fastBootFlag = 0;
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

void setup() {
  Serial.begin(115200);
  if (g_bb.magic != WG_BB_MAGIC) {      // 電源投入（RAM の内容が壊れている）
    memset(&g_bb, 0, sizeof(g_bb));
    g_bb.magic = WG_BB_MAGIC;
  } else {
    g_bb.boots++;                       // ソフト再起動で戻ってきた
  }
  mutex_init(&g_mtx);
  EEPROM.begin(256);
  cfgLoad();
  wgStateNeutral(&g_state);
  memset(&g_map, 0, sizeof(g_map));

  // NeoPixel は PIO を使うので、ここでは三色 LED だけで起動する。
  // PIO-USB が PIO0 のオフセット 0 を確保した後に NeoPixel を有効にする。
  TinyUSBDevice.setManufacturerDescriptor("irlab");
  TinyUSBDevice.setProductDescriptor("Wireless Gamepad TX");
  {   // 受信機と区別できるシリアル（同じ値を返す個体があるため）
    static char serial[8 + 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = "WGTX-";
    pico_get_unique_board_id_string(serial + 5, sizeof(serial) - 5);
    TinyUSBDevice.setSerialDescriptor(serial);
  }

  g_led.begin(40, false);
  g_led.set(WG_LED_BOOT);
  im920SetTick(ledTick);

  im920Begin();
  String boot = im920HardReset();
  im920SetupBaud(&Serial);          // 現在のボーレートを判別し、可能なら 115200 に上げる

  bootButtonWindow();                     // ここまではコア1 を止めておく

  if (boot.length()) Serial.println(boot);
  int changed = im920AutoConfig(&Serial);
  if (changed > 0) {
    Serial.println(F("設定を書き込んだので再起動します"));
    Serial.println(im920HardReset());
  } else if (changed < 0) {
    Serial.println(F("AutoConfig 失敗（配線・BUSY を確認）"));
    g_led.set(WG_LED_ERROR);
  }
  // 異常再起動からの復帰は、受信機が残っているチャンネルへ直接戻る。
  // ch31 から始めると受信機が待ち合わせに戻るまで待つことになり、実測で 21 秒かかった。
  if (g_fastBootRecover && g_bb.savedCh >= WG_CH_HOME && g_bb.savedCh <= WG_CH_LAST &&
      g_bb.savedCh != WG_CH_HOME) {
    if (im920SetChannelVolatile((uint8_t)g_bb.savedCh)) {
      g_curCh = (uint8_t)g_bb.savedCh;
      g_chScanned = true;             // 再走査はしない（受信機はそこに居る）
      Serial.printf("異常再起動からの復帰: ch%u に直接戻ります\n", g_curCh);
    }
  }
  g_bb.savedCh = 0;

  Serial.println(F("wireless gamepad TX ready（help でコマンド一覧）"));

  measureUsbLines();
  uint8_t det = detectDpPin();
  if (g_cfg.dpPin) { g_usbDp = g_cfg.dpPin; g_usbDpSrc = "usbpin で指定"; }
  else if (det)    { g_usbDp = det;         g_usbDpSrc = "自動判別"; }
  else             { g_usbDp = PIN_USB_HOST_DP_DEFAULT; g_usbDpSrc = "既定値（機器未接続で判別不可）"; }
  Serial.printf("USB-A: D+=GPIO%d D-=GPIO%d（%s／判別結果 %d）\n",
                g_usbDp, (g_usbDp == 27) ? 26 : 27, g_usbDpSrc, det);

  // ハングしても緑点灯のまま放置されないよう、ウォッチドッグを入れる。
  // 受信機側は 300ms のフェイルセーフが効くので、再起動中も暴走にはならない。
  rp2040.wdt_begin(WDT_MS);

  g_core1Go = true;                       // PIO-USB ホストを開始

  uint32_t tw = millis();                 // ホストが PIO を確保し終わるのを待つ
  while (g_hostStage < 3 && millis() - tw < 2000) delay(1);
  delay(50);
#ifndef WG_NO_NEOPIXEL
  if (!g_cfg.noNeo) g_led.enableNeoPixel();   // ここで初めて PIO を 1 つ使う
  else Serial.println(F("NeoPixel は無効（三色 LED のみ。neo on で戻す）"));
  Serial.printf("USB host stage=%u, NeoPixel を有効化\n", g_hostStage);
#else
  Serial.printf("USB host stage=%u, NeoPixel は無効（三色 LED のみ）\n", g_hostStage);
#endif
}

void loop() {
  static WgState lastSent;
  static bool     haveLast = false;
  static uint32_t lastSendMs = 0, lastInfoMs = 0;

  rp2040.wdt_reset();
  pollConsole();
  pollHeartbeat();
  pollPioState();
  pollEvents();
  pollDump();
  g_led.task();

  uint32_t now = millis();

  // LED: パッドが繋がっていて、直近に TXDA が通っていれば緑
  //
  // ⚠ ここは mutex_enter_blocking にしない。
  //   core1 がロックを保持したまま死ぬとコア0 が永久に待ち、ウォッチドッグ（コア0 が
  //   餌をやる）も止まって 4 秒後に再起動になる。受信機は 300ms で中立に落ちるので
  //   安全側ではあるが、復帰に 10 秒以上かかり、下の生存監視にも到達しない。
  //   タイムアウト付きで取り、取れなければ core1 異常として degrade 経路へ進む。
  bool padConn = false;
  bool lockOk = mutex_enter_timeout_ms(&g_mtx, 20);
  if (lockOk) {
    padConn = g_padConnected;
    mutex_exit(&g_mtx);
  } else {
    g_lockFail++;
    g_hostTickMs = (g_hostTickMs > HOST_STALL_MS) ? (millis() - HOST_STALL_MS - 1) : 0;
  }
  if (g_hbCount && (now - g_lastHbMs) >= HB_TIMEOUT_MS) g_rxReceiving = false;

  // ---- チャンネル選定と復帰 ----
  // リンクが確立してから走査する（受信機が待ち合わせ ch にいることを確かめてから移動する）
  if (!g_chScanned && g_hbCount > 0 && g_chTries < CH_MAX_TRIES) {
    g_chScanned = true;
    g_chTries++;
    uint8_t best = scanChannels();
    if (best != g_curCh) announceAndMove(best);
  }
  // 移動先で応答が無ければ待ち合わせチャンネルへ戻る（片方だけ移動した場合の復帰）
  if (g_curCh != WG_CH_HOME && g_hbCount && (now - g_lastHbMs) > CH_REVERT_MS) {
    Serial.printf("[%lu] ch%u で応答が無いため ch%u に戻ります\n",
                  (unsigned long)now, g_curCh, WG_CH_HOME);
    if (im920SetChannelVolatile(WG_CH_HOME)) g_curCh = WG_CH_HOME;
    g_lastHbMs = now;                       // すぐに再判定しない
    g_chScanned = false;                    // 再度やり直す（CH_MAX_TRIES まで）
  }

  // ---- コア1 の生存確認 ----
  if (g_hostStage >= 3) {
    uint32_t t = g_hostTicks;
    if (t != g_hostTickSeen) { g_hostTickSeen = t; g_hostTickMs = now; }
    else if (g_hostTickMs == 0) { g_hostTickMs = now; }
    bool alive = (now - g_hostTickMs) < HOST_STALL_MS;
    if (!alive && g_hostAlive) {
      g_hostAlive = false;
      g_hostStallCount++;
      {   // 固まった瞬間のレジスタを残す。core1 は止まっているが core0 からは読める
        int i2, c2, f2, s2; unsigned long in2, ee2, es2;
        wgPioState(&i2, &c2, &f2, &s2, &in2, &ee2, &es2);
        g_bb.stallInts = in2; g_bb.stallEpErr = ee2; g_bb.stallEpStall = es2;
        g_bb.stallInit = i2; g_bb.stallConn = c2; g_bb.stallFs = f2; g_bb.stallSusp = s2;
        g_bb.stallStage = g_hostStage; g_bb.stallNeedArm = g_needArm ? 1 : 0;
        g_bb.stallDevAddr = g_devAddr; g_bb.stallValid = 1;
      }
      Serial.printf("[%lu] USB ホスト（コア1）が停止。パッドを切断扱いにします\n",
                    (unsigned long)now);
    } else if (alive && !g_hostAlive) {
      g_hostAlive = true;
      Serial.printf("[%lu] USB ホストが復帰\n", (unsigned long)now);
    }
    // レポートの再アームが戻らない＝パッドからの入力が止まっている。
    // core1 は回っているので上の生存監視では捕まらない（レビュー指摘 H-4）。
    if (g_needArm && (now - g_needArmSinceMs) > 500 && padConn) {
      Serial.printf("[%lu] パッドのレポート受信が復帰しません。切断扱いにします\n",
                    (unsigned long)now);
      if (mutex_enter_timeout_ms(&g_mtx, 20)) {
        wgStateNeutral(&g_state);
        g_padConnected = false;
        g_infoValid = false;
        mutex_exit(&g_mtx);
      }
      padConn = false;
    }

    if (!g_hostAlive) {
      // 受信機をフェイルセーフに入れる（中立＋flags.bit0=0）
      if (mutex_enter_timeout_ms(&g_mtx, 20)) {
        wgStateNeutral(&g_state);
        g_padConnected = false;
        g_infoValid = false;
        mutex_exit(&g_mtx);
      }
      padConn = false;
      if (now - g_hostTickMs > HOST_REBOOT_MS) {
        Serial.println(F("復帰しないため再起動します"));
        g_bb.rebootReason = WG_R_CORE1;         // 次の起動で読めるように記録
        g_bb.upMs = now;
        g_bb.reports = g_reportCount;
        g_bb.ok = g_okCount;
        g_bb.ng = g_ngCount;
        g_bb.hbAgeMs = g_hbCount ? (now - g_lastHbMs) : 0xFFFFFFFFUL;
        g_bb.savedCh = g_curCh;       // 受信機はこの ch に残るので、戻って再会合を省く
        Serial.flush();
        g_fastBootFlag = WG_FASTBOOT_MAGIC;
        delay(50);
        rp2040.reboot();
      }
    }
  }

  // 緑は「受信機からハートビートが届いている」＝本当にリンクが生きているときだけ。
  // TXDA の OK はモジュールが受理しただけで、届いたことを意味しない。
  if (g_led.mode() != WG_LED_BRIDGE && g_led.mode() != WG_LED_PAIRING) {
    // 緑の条件: パッドが生きている ＋ USB ホストが生きている ＋
  //           受信機からハートビートが届いている ＋ 受信機が前方向を受信できている
  bool hbFresh = g_hbCount && (now - g_lastHbMs) < HB_TIMEOUT_MS;
  bool linkOk = padConn && g_hostAlive && hbFresh && g_rxReceiving;
    // 緑→赤に落ちた瞬間だけ、原因を記録する（再起動しても残る）
    if (!linkOk && g_led.mode() == WG_LED_CONNECTED) {
      g_bb.redCount++;
      g_bb.lastRedMs = now;
      uint32_t why = WG_R_NONE;
      if (!padConn)            { g_bb.redPad++;  why = WG_R_PAD; }
      else if (!g_hostAlive)   { g_bb.redHost++; why = WG_R_HOST; }
      else if (!hbFresh)       { g_bb.redHb++;   why = WG_R_HB; }
      else                     { g_bb.redRx++;   why = WG_R_RXDOWN; }
      Serial.printf("[%lu] 赤点滅へ: %s\n", (unsigned long)now, wgReasonName(why));
    }
    g_led.set(linkOk ? WG_LED_CONNECTED : WG_LED_DISCONNECTED);
  }

  // ⚠ 送信機では実行中に BOOTSEL を読まない。
  //   BOOTSEL の読み取りは rp2040.idleOtherCore() でコア1 を止め、XIP を一時停止する。
  //   コア1 では PIO-USB が動いているため、これが噛み合わないとコア1 が復帰せず、
  //   コア0 が待ち続けてデッドロックする（実機で発生。LED が緑のまま固まった）。
  //   ボタンを使えるのは、コア1 を起動する前の「起動直後の受付ウィンドウ」だけ。
  //   受信機（PIO-USB を使わない）では実行中も読んで問題ない。
  (void)g_btn;

  if (now - lastSendMs < g_txMinInterval) return;

  // Info パケット（受信機が HID ディスクリプタを合わせるために使う）
  WgState cur;
  WgInfo  info;
  bool    infoValid = false;
  if (!mutex_enter_timeout_ms(&g_mtx, 20)) { g_lockFail++; return; }
  cur = g_state;
  info = g_info;
  infoValid = g_infoValid;
  mutex_exit(&g_mtx);

  if (infoValid && (now - lastInfoMs >= TX_INFO_PERIOD)) {
    uint8_t buf[WG_INFO_LEN];
    wgInfoEncode(&info, buf);
    countSend(im920SendEx(buf, sizeof(buf)));
    lastInfoMs = now;
    lastSendMs = millis();
    return;
  }

  bool changed = !haveLast || !wgStateEqual(&cur, &lastSent);
  if (!changed && (now - lastSendMs < TX_KEEPALIVE)) return;

  cur.seq = g_seq++;
  uint8_t buf[WG_STATE_LEN];
  wgStateEncode(&cur, buf);
  countSend(im920SendEx(buf, sizeof(buf)));   // NG は再送せず次の周期に任せる
  lastSent = cur;
  haveLast = true;
  lastSendMs = millis();
}
