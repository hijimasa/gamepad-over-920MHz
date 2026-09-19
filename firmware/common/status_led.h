// status_led.h — XIAO RP2040 のオンボード LED で状態を表示する
//   ボタン受付中: 水色の速い点滅
//   接続なし   : 赤の点滅
//   ペアリング中: 黄の点滅
//   接続中     : 緑の点灯
//   設定ブリッジ: 青の点灯 / 起動中: 白の点滅 / 異常: 赤紫の速い点滅
//
// NeoPixel（GPIO12、電源 GPIO11）と、三色 LED（R=17, G=16, B=25／Low で点灯）の
// 両方を同時に駆動する。
//
// ⚠ NeoPixel は RP2040 では PIO を使う（pio_claim_free_sm_and_add_program）。
//   PIO-USB は TX プログラムを PIO0 のオフセット 0 に置く必要があるため、
//   送信機では **PIO-USB を起動してから** enableNeoPixel() を呼ぶこと。
//   begin(brightness, false) なら三色 LED だけで動くので、起動直後の表示に使える。
#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL   12
#endif
#ifndef NEOPIXEL_POWER
#define NEOPIXEL_POWER 11
#endif

enum WgLedMode {
  WG_LED_OFF = 0,
  WG_LED_BOOT,
  WG_LED_DISCONNECTED,
  WG_LED_PAIRING,
  WG_LED_CONNECTED,
  WG_LED_BRIDGE,
  WG_LED_BUTTON,      // 起動直後のボタン受付中（水色の速い点滅）
  WG_LED_ERROR,
};

class WgStatusLed {
 public:
  void begin(uint8_t brightness = 40, bool useNeoPixel = true) {
    _bright = brightness;
#ifdef PIN_LED_R
    pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, HIGH);
    pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, HIGH);
    pinMode(PIN_LED_B, OUTPUT); digitalWrite(PIN_LED_B, HIGH);
#endif
    _lastShown = 0xFFFFFFFF;
    if (useNeoPixel) enableNeoPixel();
  }

  // NeoPixel を使い始める（PIO を 1 つ消費する）
  void enableNeoPixel() {
    if (_neo) return;
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    _px.begin();
    _px.setBrightness(_bright);
    _px.clear();
    _px.show();
    _neo = true;
    _lastShown = 0xFFFFFFFF;
  }

  void set(WgLedMode m) {
    if (m != _mode) { _mode = m; _phase = true; _t0 = millis(); }
  }

  WgLedMode mode() const { return _mode; }

  // 10ms 程度ごとに呼ぶ
  void task() {
    uint32_t period = 0, r = 0, g = 0, b = 0;
    switch (_mode) {
      case WG_LED_OFF:          period = 0;   r = 0;   g = 0;   b = 0;   break;
      case WG_LED_BOOT:         period = 500; r = 120; g = 120; b = 120; break;
      case WG_LED_DISCONNECTED: period = 500; r = 255; g = 0;   b = 0;   break;
      case WG_LED_PAIRING:      period = 200; r = 255; g = 160; b = 0;   break;
      case WG_LED_CONNECTED:    period = 0;   r = 0;   g = 255; b = 0;   break;
      case WG_LED_BRIDGE:       period = 0;   r = 0;   g = 0;   b = 255; break;
      case WG_LED_BUTTON:       period = 150; r = 0;   g = 200; b = 255; break;
      case WG_LED_ERROR:        period = 120; r = 255; g = 0;   b = 255; break;
    }
    if (period) {
      uint32_t now = millis();
      if (now - _t0 >= period / 2) { _t0 = now; _phase = !_phase; }
      if (!_phase) { r = g = b = 0; }
    } else {
      _phase = true;
    }
    show(r, g, b);
  }


 private:
  void show(uint32_t r, uint32_t g, uint32_t b) {
    uint32_t c = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
    if (c == _lastShown) return;
    _lastShown = c;
    if (_neo) {
      _px.setPixelColor(0, _px.Color((uint8_t)r, (uint8_t)g, (uint8_t)b));
      _px.show();
    }
#ifdef PIN_LED_R
    digitalWrite(PIN_LED_R, r > 64 ? LOW : HIGH);   // 三色 LED は Low で点灯
    digitalWrite(PIN_LED_G, g > 64 ? LOW : HIGH);
    digitalWrite(PIN_LED_B, b > 64 ? LOW : HIGH);
#endif
  }

  Adafruit_NeoPixel _px{1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800};
  WgLedMode _mode = WG_LED_OFF;
  uint32_t  _t0 = 0;
  uint32_t  _lastShown = 0xFFFFFFFF;
  uint8_t   _bright = 40;
  bool      _neo = false;
  bool      _phase = true;
};
