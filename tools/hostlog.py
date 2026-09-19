#!/usr/bin/env python3
"""ネイティブホストのテストファーム（firmware/hosttest）のログを読む。

ホストモードでは送信機の USB コンソールが使えないため、テストファームは
状態を IM920sL でテキストとして飛ばす。受信機を bridge モードにして、
受信した HEX を ASCII に戻して表示する。

  tools/hostlog.py                 … /dev/ttyACM1（受信機）を使う
  tools/hostlog.py -p /dev/ttyACM1
"""
import argparse, re, sys, time
import serial

LINE = re.compile(r"^\s*([0-9A-Fa-f]{2}),([0-9A-Fa-f]{4}),([0-9A-Fa-f]{2}):(.+)$")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyACM1")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    a = ap.parse_args()

    with serial.Serial(a.port, a.baud, timeout=0.2) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"bridge\n")          # 受信機を透過モードにする
        s.flush()
        time.sleep(0.5)
        s.reset_input_buffer()
        print("受信待ち（Ctrl-C で終了）", file=sys.stderr)
        buf = ""
        try:
            while True:
                d = s.read(4096)
                if not d:
                    continue
                buf += d.decode("utf-8", "replace")
                while "\n" in buf:
                    ln, buf = buf.split("\n", 1)
                    m = LINE.match(ln.strip())
                    if not m:
                        continue
                    node, rssi_hex, payload = m.group(2), m.group(3), m.group(4)
                    rssi = int(rssi_hex, 16)
                    if rssi > 127:
                        rssi -= 256
                    try:
                        data = bytes(int(x, 16) for x in payload.split(",") if x.strip())
                    except ValueError:
                        continue
                    text = "".join(chr(c) if 32 <= c < 127 else "." for c in data)
                    print(f"[{node} {rssi:>4}dBm] {text}")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            s.write(b"+++")           # bridge を抜ける
            s.flush()
            time.sleep(0.3)
    return 0

if __name__ == "__main__":
    sys.exit(main())
