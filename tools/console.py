#!/usr/bin/env python3
"""送信機・受信機の CDC コンソールに繋いでコマンドを実行する簡易ツール。

  tools/console.py                 … status を実行
  tools/console.py status map      … 複数コマンドを順に実行
  tools/console.py -m 5            … コマンドを送らず 5 秒間ログを眺める
  tools/console.py -p /dev/ttyACM1 status
"""
import argparse, sys, time
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmds", nargs="*", default=[])
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-m", "--monitor", type=float, default=0.0,
                    help="コマンド後にログを読む秒数")
    a = ap.parse_args()

    with serial.Serial(a.port, a.baud, timeout=0.1) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for c in a.cmds:
            s.write((c + "\n").encode())
            s.flush()
            t0 = time.time()
            while time.time() - t0 < 1.5:
                d = s.read(4096)
                if d:
                    sys.stdout.write(d.decode("utf-8", "replace"))
                    sys.stdout.flush()
                    t0 = time.time()
        t0 = time.time()
        while time.time() - t0 < a.monitor:
            d = s.read(4096)
            if d:
                sys.stdout.write(d.decode("utf-8", "replace"))
                sys.stdout.flush()
    return 0

if __name__ == "__main__":
    sys.exit(main())
