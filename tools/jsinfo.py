#!/usr/bin/env python3
"""/dev/input/js0 の軸数・ボタン数と、実際のイベントを表示する。

  tools/jsinfo.py            … 情報だけ表示
  tools/jsinfo.py 10         … 10 秒間イベントを表示（パッドを動かして確認）
"""
import fcntl, struct, sys, time, select

DEV = "/dev/input/js0"
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12


def jsname(fd, n=128):
    buf = bytearray(n)
    fcntl.ioctl(fd, 0x80006A13 + (n << 16), buf)
    return buf.split(b"\0")[0].decode("utf-8", "replace")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    dev = sys.argv[2] if len(sys.argv) > 2 else DEV
    f = open(dev, "rb")
    fd = f.fileno()
    axes = bytearray(1); fcntl.ioctl(fd, JSIOCGAXES, axes)
    btns = bytearray(1); fcntl.ioctl(fd, JSIOCGBUTTONS, btns)
    print(f"{dev}: \"{jsname(fd)}\"  軸 {axes[0]} 個 / ボタン {btns[0]} 個")

    axv = [0] * axes[0]
    btv = [0] * btns[0]
    t0 = time.time()
    while time.time() - t0 < secs:
        r, _, _ = select.select([f], [], [], 0.2)
        if not r:
            continue
        ev = f.read(8)
        if len(ev) < 8:
            break
        _, value, etype, number = struct.unpack("IhBB", ev)
        if etype & 0x02 and number < len(axv):      # JS_EVENT_AXIS
            axv[number] = value
        elif etype & 0x01 and number < len(btv):    # JS_EVENT_BUTTON
            btv[number] = value
        if not (etype & 0x80):                      # 初期同期イベントは表示しない
            print("軸 " + " ".join(f"{v:6d}" for v in axv) +
                  "  | ボタン " + "".join(str(v) for v in btv), end="\r", flush=True)
    if secs:
        print()


if __name__ == "__main__":
    main()
