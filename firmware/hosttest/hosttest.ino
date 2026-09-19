// ネイティブ USB ホストの動作確認用スケッチ
//
// 目的: XIAO RP2040 の USB-C に OTG 変換アダプタでゲームパッドを繋ぎ、
//       PIO-USB を使わずに列挙・レポート受信ができるかを確かめる。
//       これが動けば、core1（PIO-USB）の停止問題は構造的に消える。
//
// 配線: 基板の J1（USB-A）に 5V を入れて給電する。
//       5V ピン経由で USB-C の VBUS に回り、OTG 側のパッドにも給電される。
//
// 判定は LED だけで行う（ホストモードでは USB の CDC コンソールが使えないため）。
// どこで止まったかが分かるように色を分けてある:
//   白の点滅     … USBHost.begin() から戻ってきていない（初期化で固まっている）
//   赤の点滅     … USB 機器が何も検出できていない
//                   → まずパッド自身の LED が光っているか見ること。
//                     光っていなければ VBUS が OTG 側に回っていない（給電の問題）
//   橙の点滅     … USB 機器は検出したが HID として認識できていない（列挙の問題）
//   紫の点滅     … HID は認識したがレポート要求が失敗した
//   緑の点灯     … HID を列挙できた（レポートはまだ来ていない）
//                   → このパッドは「変化時のみ」レポートを返すので、
//                     ここまで来たら **スティックを動かすこと**
//   水色の速い点滅 … レポートを受信した（完全に動作）。一度受信したら以後ずっと水色
//
// ビルド: ./build.sh hosttest upload
#include "Adafruit_TinyUSB.h"

#ifndef USE_TINYUSB_HOST
#error usbstack=tinyusb_host でビルドすること（./build.sh hosttest）
#endif

#include "status_led.h"

Adafruit_USBH_Host USBHost;
static WgStatusLed g_led;

static volatile bool     g_anyDev    = false;   // 何らかの USB 機器を検出した
static volatile bool     g_mounted   = false;   // HID として認識できた
static volatile bool     g_armFail   = false;   // レポート要求が失敗した
static volatile uint32_t g_reports   = 0;
static volatile uint32_t g_lastRepMs = 0;
static bool g_begun = false;
// レポート要求が通っていても転送が始まらないことがあるので、
// 動作している PIO-USB 版と同じく、届くまで定期的に要求し直す
static volatile uint8_t  g_addr = 0, g_inst = 0;
static volatile bool     g_haveDev = false;
static uint32_t g_reArm = 0;

void setup() {
  // PIO-USB を使わないので、NeoPixel と PIO を取り合う心配がない
  g_led.begin(40, true);
  g_led.set(WG_LED_BOOT);
  USBHost.begin(0);            // ネイティブ USB コントローラをホストとして使う
  g_begun = true;              // ここに来なければ begin() で固まっている（白のまま）
}

void loop() {
  USBHost.task();              // core1 は使わない。ここで回すだけ
  (void)g_lastRepMs;
  // まだ 1 度もレポートが来ていなければ 100ms ごとに要求し直す
  if (g_haveDev && g_reports == 0 && millis() - g_reArm > 100) {
    g_reArm = millis();
    if (!tuh_hid_receive_report(g_addr, g_inst)) g_armFail = true;
  }
  if (!g_begun)                     g_led.set(WG_LED_BOOT);          // 白：初期化で停止
  else if (g_armFail)               g_led.set(WG_LED_ERROR);         // 紫：要求が失敗
  else if (!g_anyDev)               g_led.set(WG_LED_DISCONNECTED);  // 赤：機器なし
  else if (!g_mounted)              g_led.set(WG_LED_PAIRING);       // 橙：HID でない
  // 一度でもレポートが来たらラッチする。変化時のみ報告するパッドだと
  // 「受信中だけ水色」では一瞬で戻ってしまい、見逃す
  else if (g_reports > 0)           g_led.set(WG_LED_BUTTON);        // 水色：受信済み
  else                              g_led.set(WG_LED_CONNECTED);     // 緑：列挙済み
  g_led.task();
}

// ---- TinyUSB ホストのコールバック ----
// HID かどうかに関わらず、USB 機器を検出した時点で呼ばれる。
// 「機器が見えていない」のか「見えているが HID として扱えない」のかを分ける
void tuh_mount_cb(uint8_t addr)   { (void)addr; g_anyDev = true; }
void tuh_umount_cb(uint8_t addr)  { (void)addr; g_anyDev = false; g_mounted = false; }

void tuh_hid_mount_cb(uint8_t addr, uint8_t inst,
                      uint8_t const *desc, uint16_t len) {
  (void)desc; (void)len;
  g_anyDev  = true;
  g_mounted = true;
  g_addr = addr; g_inst = inst; g_haveDev = true;
  // ブートプロトコルのままだとゲームパッドのレポートが出ない機器があるため明示する
  tuh_hid_set_protocol(addr, inst, HID_PROTOCOL_REPORT);
  if (!tuh_hid_receive_report(addr, inst)) g_armFail = true;   // 最初のレポートを要求
}

void tuh_hid_umount_cb(uint8_t addr, uint8_t inst) {
  (void)addr; (void)inst;
  g_mounted = false;
  g_haveDev = false;
}

void tuh_hid_report_received_cb(uint8_t addr, uint8_t inst,
                                uint8_t const *report, uint16_t len) {
  (void)report; (void)len;
  g_reports++;
  g_lastRepMs = millis();
  if (!tuh_hid_receive_report(addr, inst)) g_armFail = true;   // 次を要求しないと止まる
}
