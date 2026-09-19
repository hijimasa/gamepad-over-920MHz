// pio_state.c — Pico-PIO-USB の内部状態を覗くための小さなラッパ
// pio_usb_ll.h は C++ ではそのまま通らないので、C のまま分離する
#include <stdint.h>
#include <stdbool.h>
#include "pio_usb.h"
#include "pio_usb_ll.h"

void wgPioState(int *initialized, int *connected, int *fullspeed, int *suspended,
                unsigned long *ints, unsigned long *epError, unsigned long *epStalled) {
  root_port_t *rp = PIO_USB_ROOT_PORT(0);
  *initialized = rp->initialized ? 1 : 0;
  *connected   = rp->connected ? 1 : 0;
  *fullspeed   = rp->is_fullspeed ? 1 : 0;
  *suspended   = rp->suspended ? 1 : 0;
  *ints        = (unsigned long)rp->ints;
  *epError     = (unsigned long)rp->ep_error;
  *epStalled   = (unsigned long)rp->ep_stalled;
}

// 物理的に抜き差しせずに接続検出をやり直させる
// （connected を落とすと、次のフレームで PIO-USB が再度 connect 割り込みを上げる）
void wgPioForceReconnect(void) {
  root_port_t *rp = PIO_USB_ROOT_PORT(0);
  rp->connected = false;
  rp->suspended = true;
}
