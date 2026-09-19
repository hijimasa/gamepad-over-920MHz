// wg_dbg.h — TinyUSB のデバッグ出力を RAM のリングバッファへ流すための宣言
//
// Adafruit_TinyUSB_Arduino の log_printf() は SERIAL_TUSB_DEBUG（既定 Serial1）へ書く。
// Serial1 は IM920sL に繋がっていて使えず、Serial（USB CDC）にすると
// ホストスタック（コア1）から CDC を叩くことになり競合してハングする。
// そこで -DSERIAL_TUSB_DEBUG=wgDbgStream と -include このファイル を与えて、
// 書き込み先を RAM のリングバッファに差し替える。
#ifndef WG_DBG_H            // sketch のコピーと -include で経路が違うため pragma once は使えない
#define WG_DBG_H
#ifdef __cplusplus
#include <Arduino.h>

class WgDbgStream : public Print {
 public:
  void begin(unsigned long) {}
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t n) override;
  using Print::write;
};

extern WgDbgStream wgDbgStream;
#endif
#endif  // WG_DBG_H
