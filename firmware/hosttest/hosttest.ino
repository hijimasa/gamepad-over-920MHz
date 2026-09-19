// ネイティブ USB ホストの動作確認用スケッチ
//
// 目的: XIAO RP2040 の USB-C に OTG 変換アダプタでゲームパッドを繋ぎ、
//       PIO-USB を使わずに列挙・レポート受信ができるかを確かめる。
//       これが動けば、core1（PIO-USB）の停止問題は構造的に消える。
//
// 配線: 基板の J1（USB-A）に 5V を入れて給電する。
//       5V ピン経由で USB-C の VBUS に回り、OTG 側のパッドにも給電される。
//
// 判定は LED だけで行う（ホストモードでは USB の CDC コンソールが使えないため）:
//   白の点滅 … 起動中
//   赤の点滅 … HID デバイスが見つかっていない
//   緑の点灯 … HID デバイスを列挙できた（ここまで来れば列挙は成功）
//   水色の速い点滅 … レポートが届いている（ここまで来れば完全に動作）
//
// ビルド: ./build.sh hosttest upload
#include "Adafruit_TinyUSB.h"

#ifndef USE_TINYUSB_HOST
#error usbstack=tinyusb_host でビルドすること（./build.sh hosttest）
#endif

#include "status_led.h"

Adafruit_USBH_Host USBHost;
static WgStatusLed g_led;

static volatile bool     g_mounted   = false;
static volatile uint32_t g_reports   = 0;
static volatile uint32_t g_lastRepMs = 0;

void setup() {
  // PIO-USB を使わないので、NeoPixel と PIO を取り合う心配がない
  g_led.begin(40, true);
  g_led.set(WG_LED_BOOT);
  USBHost.begin(0);            // ネイティブ USB コントローラをホストとして使う
}

void loop() {
  USBHost.task();              // core1 は使わない。ここで回すだけ
  uint32_t now = millis();
  if (!g_mounted)                        g_led.set(WG_LED_DISCONNECTED);
  else if (now - g_lastRepMs < 300)      g_led.set(WG_LED_BUTTON);   // レポート受信中
  else                                   g_led.set(WG_LED_CONNECTED);
  g_led.task();
}

// ---- TinyUSB ホストのコールバック ----
void tuh_hid_mount_cb(uint8_t addr, uint8_t inst,
                      uint8_t const *desc, uint16_t len) {
  (void)desc; (void)len;
  g_mounted = true;
  tuh_hid_receive_report(addr, inst);    // 最初のレポートを要求する
}

void tuh_hid_umount_cb(uint8_t addr, uint8_t inst) {
  (void)addr; (void)inst;
  g_mounted = false;
}

void tuh_hid_report_received_cb(uint8_t addr, uint8_t inst,
                                uint8_t const *report, uint16_t len) {
  (void)report; (void)len;
  g_reports++;
  g_lastRepMs = millis();
  tuh_hid_receive_report(addr, inst);    // 次を要求しないと以後届かない
}
