#!/usr/bin/env bash
# ビルド／書込みスクリプト（arduino-cli）
#   ./build.sh tx            送信機をビルド
#   ./build.sh rx            受信機をビルド
#   ./build.sh tx upload     ビルドして書込み（XIAO を BOOTSEL 押しながら接続 → RPI-RP2 にコピー）
#   ./build.sh tx upload /dev/ttyACM0
set -euo pipefail
cd "$(dirname "$0")"

ROLE="${1:-}"
ACTION="${2:-build}"
PORT="${3:-}"

case "$ROLE" in
  # PIO-USB は 12MHz の倍数が必要。120MHz だとタイミング余裕が足りず列挙に
  # 失敗する機器があるため、既定は 240MHz（WG_FREQ で変更可）
  tx) FQBN="rp2040:rp2040:seeed_xiao_rp2040:usbstack=tinyusb,freq=${WG_FREQ:-240}" ;;
  rx) FQBN="rp2040:rp2040:seeed_xiao_rp2040:usbstack=tinyusb" ;;
  # ネイティブ USB ホストの動作確認用。PIO-USB を使わないので 240MHz も不要
  hosttest) FQBN="rp2040:rp2040:seeed_xiao_rp2040:usbstack=tinyusb_host" ;;
  *)  echo "usage: $0 {tx|rx|hosttest} [build|upload] [port]" >&2; exit 1 ;;
esac

SKETCH="firmware/$ROLE"
BUILD="build/$ROLE"
mkdir -p "$BUILD"

# WG_EXTRA_FLAGS で追加の -D を渡せる（例: TinyUSB のデバッグ出力）
#   WG_EXTRA_FLAGS=-DCFG_TUSB_DEBUG=1 ./build.sh tx upload /dev/ttyACM0
# WG_EXTRA_FLAGS_CPP は C++ のみに渡す（-include など C に入れたくないもの）
EXTRA=()
if [ -n "${WG_EXTRA_FLAGS:-}" ] || [ -n "${WG_EXTRA_FLAGS_CPP:-}" ]; then
  EXTRA+=(--build-property "compiler.c.extra_flags=${WG_EXTRA_FLAGS:-}"
          --build-property "compiler.cpp.extra_flags=${WG_EXTRA_FLAGS:-} ${WG_EXTRA_FLAGS_CPP:-}")
fi

arduino-cli compile -b "$FQBN" --library firmware/common \
  "${EXTRA[@]+"${EXTRA[@]}"}" \
  --build-path "$(pwd)/$BUILD" "$SKETCH"

if [ "$ACTION" = "upload" ]; then
  if [ -n "$PORT" ]; then
    arduino-cli upload -b "$FQBN" -p "$PORT" --input-dir "$(pwd)/$BUILD" "$SKETCH"
  else
    arduino-cli upload -b "$FQBN" --input-dir "$(pwd)/$BUILD" "$SKETCH"
  fi
fi
