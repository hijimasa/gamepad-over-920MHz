// boot_button.h — BOOTSEL ボタンを設定・ペアリングボタンとして使う
//
// 制約
//  ・電源投入時に押しっぱなしだと書込みモード（UF2 ブートローダ）に入ってしまうので、
//    「電源を入れてから押す」운用にする必要がある。
//  ・BOOTSEL の読み取りは XIP（フラッシュ実行）を一瞬止め、もう一方のコアを idle に
//    するため、PIO-USB がホストとして動いている最中は避けたい。
//
// そこで 2 つの受付タイミングを用意する。
//  1. 起動直後のウィンドウ（PIO-USB 開始前なので完全に安全）
//  2. 実行中、**リンクが確立していないとき（LED が赤点滅）だけ**ポーリングする。
//     緑点灯（正常動作中）のときは一切読まないので、運用中の通信を乱さない。
#pragma once
#include <Arduino.h>

enum WgButtonAction { WG_BTN_NONE = 0, WG_BTN_SHORT, WG_BTN_LONG };

// 押されている間、押下時間を測る（tick で LED の点滅などを続ける）
inline uint32_t wgBootselHold(void (*tick)(), uint32_t maxMs = 8000) {
  uint32_t t0 = millis();
  while (BOOTSEL && millis() - t0 < maxMs) {
    if (tick) tick();
    delay(20);
  }
  return millis() - t0;
}

class WgBootButton {
 public:
  // enable が false の間はまったく読まない
  WgButtonAction poll(bool enable, uint32_t longMs, void (*tick)()) {
    if (!enable) return WG_BTN_NONE;
    uint32_t now = millis();
    if (now - _last < _interval) return WG_BTN_NONE;
    _last = now;
    if (!BOOTSEL) return WG_BTN_NONE;
    uint32_t held = wgBootselHold(tick);
    _last = millis();
    if (held >= longMs) return WG_BTN_LONG;
    if (held >= 50)     return WG_BTN_SHORT;
    return WG_BTN_NONE;
  }
  void setInterval(uint32_t ms) { _interval = ms; }

 private:
  uint32_t _last = 0;
  uint32_t _interval = 300;      // 読みに行く間隔
};
