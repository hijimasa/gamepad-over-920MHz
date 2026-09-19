# USBゲームパッド無線化（IM920sL × XIAO RP2040）引き継ぎ資料

Claude Code でファームウェア実装を引き継ぐための資料。ハードウェアの仕様は確定済みで、ソフトウェアはまだ書かれていない。
最終更新: 2026-09 / 回路図 Rev.3

> **この資料は実装を始める前に書かれたものです。** 実装後に判明した事実（PIO-USB の
> 240MHz 要件、D+/D− の実配線、SW1 と STATUS LED の廃止など）は反映済みですが、
> 現在の仕様と測定値は [README](../README.md) が正です。

---

## 1. 目的と全体構成

既存の **USB ゲームパッド** を、Bluetooth より遠くまで届く **920MHz 帯（IM920sL）** で無線化する。PC 側からは普通のゲームパッドに見えるようにする。

```
[USBゲームパッド] ──USB(PIO-USB)──> [送信機 XIAO RP2040] ──UART──> [IM920sL]
                                                                        │ 920MHz
[PC] <──USB HID + CDC── [受信機 XIAO RP2040] <──UART── [IM920sL] <──────┘
```

- **PC 側の利用先:** navigators リポジトリの `gamepad_related` が `/dev/input/js0` を直接読んでいる。受信機が標準 HID ゲームパッドとして認識されれば、既存コードは変更しなくてよい。そのため、**ボタン番号と軸の並びは元のゲームパッドと同じにすること**（`jstest /dev/input/js0` で元パッドの割り当てを確認する）。
- **用途:** ロボットの遠隔操縦。通信が途絶えたときのフェイルセーフは必須。

## 2. ハードウェア（確定）

添付ファイル: `schematic/gamepad_wireless_schematic.svg`（Rev.3）、`gamepad_wireless_BOM.xlsx`、`netlist.md`

### 2.1 ピン割り当て（送信機・受信機で共通。USB-A まわりは送信機のみ）

| XIAO ピン | GPIO | 接続先 | 備考 |
|---|---|---|---|
| D0 | GPIO26 | USB-A **D+**（R2 22Ω 経由） | 送信機のみ。実機で機器のプルアップ位置から確認済み |
| D1 | GPIO27 | USB-A **D−**（R1 22Ω 経由） | 送信機のみ |
| D2 | GPIO28 | IM920sL BUSY（ADP J1-1 / 本体 pin1） | 入力。**L の間だけコマンド受付** |
| D3 | GPIO29 | IM920sL RESET（ADP J1-10 / pin19） | L でリセット。モジュール内部に 10kΩ プルアップ。出力は「LOW を出す／INPUT で解放」の2状態で扱う |
| D6/TX | GPIO0 | IM920sL RxD（ADP J2-3 / pin6） | Serial1 TX |
| D7/RX | GPIO1 | IM920sL TxD（ADP J1-4 / pin7） | Serial1 RX |
| D8 | GPIO2 | 未接続 | Rev.3 で SW1 を廃止（BOOTSEL で代用） |
| 5V | — | USB-A VBUS | 送信機のみ。モバイルバッテリー → XIAO USB-C から供給 |
| 3V3 | — | IM920sL VCC（ADP J1-9 / pin17） | C2 10µF＋C3 0.1µF |
| GND | — | ADP J2-9 / USB-A GND | |
| — | — | ADP J1-8 STATUS | 未接続（Rev.3 で STATUS LED を廃止。状態表示は XIAO のカラー LED） |

未使用ピンは D4, D5, D9, D10。IM920sL の IO8〜IO10 はオープンのままにする（電源投入時にオープンならデータモードで起動する）。

### 2.2 PIO-USB の設定

```cpp
pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
pio_cfg.pin_dp = 26;                    // 実配線の D+
pio_cfg.pinout = PIO_USB_PINOUT_DPDM;   // D- = D+ + 1 = GPIO27（既定値）
USBHost.configure_pio_usb(1, &pio_cfg);
```

- Pico-PIO-USB の `PIO_USB_PINOUT` は、`DPDM`（D− = D+ + 1、既定）と `DMDP`（D− = D+ − 1）の2種類。`pinout` が未定義というエラーが出たら、ライブラリを更新する。
- **CPU クロックは 240MHz**（12MHz の倍数である必要がある。120MHz では列挙に失敗した）。

## 3. IM920sL 仕様メモ（取扱説明書 Rev.1.5 より）

- **UART:** 19200bps 8N1（初期値。`SBRT` で変更できる）。コマンドの終端は CR LF。
- **応答:** `OK` / `NG` / 読出し値。起動時に `IM920sL Ver.xx.xx` を出力する。
- **BUSY:** L のときだけコマンドを受け付ける。H の間に送ったコマンドやデータは無視される。
- **Flash への保存:** `ENWR` の後に行った設定が Flash に保存される（書換えは全項目合計で 10 万回まで）。終わったら `DSWR` で書込み禁止に戻す。
- **主なコマンドと設定値:**

  | 設定・読出し | 内容 | 本プロジェクトの設定値 |
  |---|---|---|
  | `STNN` / `RDNN` | ノード番号（16進4桁） | 受信機 = 親機 **0001**、送信機 = **0002** |
  | `STRT` / `RDRT` | 通信モード（1=高速 100kbps、2=中距離、3=長距離） | **1** |
  | `STCH` / `RDCH` | チャネル（01〜29、31〜45） | **31** |
  | `STPO` / `RDPO` | 送信出力（1=1mW、2=10mW） | **2** |
  | `STTR` / `RDTR` | キャリアセンス NG 時の自動リトライ回数（00〜10） | **00** |
  | `STGN` / `RDGN` | グループ番号の登録（ペアリング） | §5.3 |
  | `TXDA <hex>` | ブロードキャスト送信（1〜32 バイト、HEX 入出力モード） | |
  | `SRST` | ソフトウェアリセット | |

  - **設定の順序:** `STNN 0001` は ENWR 状態でないと NG になる。`STRT` を変えるとチャネルが既定値に戻るので、**STRT → STCH の順**で設定する。
- **受信データの出力形式:** `aa,bbbb,dd:xx,xx,...`（`bbbb` = 送信元ノード番号、`dd` = RSSI を符号付き16進で表したもの（dBm））。
- **チャネルと送信時間制限（重要）:**
  - **ch01〜29（360s モード）:** 1時間あたりの送信時間の合計が 360 秒まで。20Hz で送り続けると約1時間で上限に達し、`TXDA` が NG を返すようになる。**使わない**。
  - **ch31〜45（4s モード）:** 時間総和の制限なし。キャリアセンス約6ms。3.9 秒送信するごとに 52ms の休止が入る（モジュールが自動で処理し、その間の送信は NG になる）。**こちらを使う**。
  - ⚠ 高速モードと ch31 の組み合わせが受け付けられるかは実機で未確認。最初に `STCH 31` の応答を確認すること。
- **1パケットの電波の送信時間（高速モード）:** 4.56ms ＋ 80µs × バイト数。
- **ペアリング（STGN）:** 親機・子機の両方で `ENWR` → `STGN` を実行し、両者を **50cm 以内**に置く。成功すると子機が `GRNOREGD` を出力する。**完了後は RESET 端子で再起動する（SRST では通常動作に戻らない）**。
- **遅延:** マルチホップ処理のため、コマンドを入れてから実際に送信されるまでの遅延にばらつきがある（取扱説明書に注意書きあり）。

## 4. 既存コード

`im920sl_config.h`（添付。Arduino-Pico 用。ダミーの Arduino.h を使った構文チェックのみ済み、**実機では未検証**）

| 関数 | 内容 |
|---|---|
| `im920Begin()` | ピンと Serial1 の初期化（FIFO 256） |
| `im920HardReset()` | RESET 端子でリセットし、起動メッセージを返す |
| `im920Cmd(cmd, timeout)` | BUSY を確認してコマンドを送り、1行の応答を返す。**ブロッキング。送信前に RX バッファを捨てる** |
| `im920AutoConfig(log)` | 各設定を読み出し、目標値と違う項目だけ ENWR で書き込む。役割は `#define IM_ROLE_TX` で切り替える |
| `im920Pair(isParent, waitMs, log)` | STGN でペアリングし、最後に RESET 端子で再起動する |
| `im920Bridge()` | USB CDC と IM920sL の透過ブリッジ（戻らない） |
| `im920Send(buf, len)` | `TXDA` を送信し、応答が OK なら true |
| `im920ParseRx(line, node, rssi, out, maxLen, len)` | 受信行を解析する |

- `im920Cmd` / `im920Send` はブロッキングで、RX バッファも捨てる。**受信機のメインループでは使わない**。メインループ用には、ノンブロッキングの行読みを別に書く。

## 5. 実装してほしいもの

### 5.1 推奨リポジトリ構成

```
wireless_gamepad/
├── docs/                 # 本資料, 回路図, BOM
├── firmware/
│   ├── common/
│   │   ├── im920sl_config.h
│   │   └── packet.h      # 無線パケット定義（§5.2）
│   ├── tx/               # 送信機（platformio.ini, src/main.cpp）
│   └── rx/               # 受信機
└── tools/                # 必要なら PC 側の確認スクリプト
```

- **ビルド環境:** Arduino-Pico（earlephilhower コア）、ボードは Seeed XIAO RP2040、USB Stack は **Adafruit TinyUSB**。送信機は CPU を 240MHz に設定する。
- **PlatformIO を使う場合:** `platform = https://github.com/maxgerhardt/platform-raspberrypi.git`、`board_build.core = earlephilhower`、`build_flags = -DUSE_TINYUSB`（詳細は要確認）。
- **土台にするサンプル:** Adafruit TinyUSB Arduino の DualRole/HID 系サンプル（PIO-USB ホスト＋デバイス）と、HID ゲームパッドのサンプル。

### 5.2 無線パケット（案。12バイト → TXDA の HEX は 24 文字）

| オフセット | 型 | 内容 |
|---|---|---|
| 0 | u8 | seq（毎回 +1） |
| 1–2 | u16 LE | buttons（bit0〜15） |
| 3 | u8 | hat（0〜7 が方向、8 = 中立） |
| 4–7 | u8 ×4 | LX, LY, RX, RY（0x80 が中立） |
| 8–9 | u8 ×2 | L2, R2（アナログトリガ。無ければ 0） |
| 10 | u8 | flags（bit0: パッド接続中） |
| 11 | u8 | checksum（byte0〜10 の XOR） |

- **帯域の見積り:** 電波の送信時間は約 5.5ms。UART では `TXDA …` の約31文字に約16ms、応答に数ms かかる。実効の上限は 30〜50Hz 程度と見込む。
- **SBRT での高速化:** 必要なら `SBRT` で 115200bps に上げる。その場合、XIAO 側のボーレート変更と、起動時のボーレート自動判別も実装すること。

### 5.3 送信機（firmware/tx）

- **コア1:** PIO-USB ホスト（`setup1` / `loop1` で `USBHost.task()`）。§2.2 の設定を使う。
- **パッドの読み取り:**
  - HID レポートは機種ごとに形式が違う。まず **dump モード**（生レポートを CDC に16進で出す）を実装し、元パッドのレポート形式を調べる。
  - 次に **VID:PID ごとのマッピング表**（バイト位置・ビット・軸）で §5.2 の形式に変換する。
  - XInput（Xbox 系）は初期段階では対応しない。
- **コア0:**
  - 起動: `im920Begin` → `im920HardReset` → `im920AutoConfig`（変更があれば再度 HardReset）。
  - 送信: **状態が変わったら即送信＋変化がなくても 100ms ごとに送信**。最短間隔は 30ms 程度。`TXDA` が NG でも再送せず、次の周期で送る。
  - パッドが外れたら flags.bit0 = 0 にし、中立状態を送る。
- **USB-C（ネイティブ）:** CDC シリアルとして §5.4 のコンソールを載せる。

### 5.4 設定・ペアリングの操作（Rev.3 で SW1 は廃止し BOOTSEL のみ）

どちらの方法でも使えるようにする。

1. **CDC コンソール（主な方法）:** 行単位で次のコマンドを受け付ける。
   - `help`
   - `status`（RDNN/RDRT/RDCH/RDPO/RDGN と RSSI の表示）
   - `config`（AutoConfig の実行）
   - `pair`（受信機は親機、送信機は子機として im920Pair）
   - `bridge`（透過ブリッジ。抜けるにはリセット）
   - `dump`（送信機のみ。HID の生レポートを出力）
2. **ボタン:** SW1（GPIO2 が LOW）または XIAO の BOOT ボタン（Arduino-Pico の `BOOTSEL` で読む）。
   - 短押し = bridge、3秒以上の長押し = pair。
   - **BOOT ボタンを押したまま電源を入れると書込みモードに入る**ので、「起動後 2 秒以内に押した場合だけ有効」にする。

### 5.5 受信機（firmware/rx）

- **USB 複合デバイス:** HID ゲームパッドと CDC コンソール（§5.4）。
  - HID ディスクリプタは TinyUSB の `TUD_HID_REPORT_DESC_GAMEPAD`（hid_gamepad_report_t）を基本にしてよい。
  - 軸とボタンの並びは、**元パッドの `/dev/input/js0` での番号と一致させる**（§1）。
- **IM920sL の受信:** ノンブロッキングで行を組み立てて `im920ParseRx` に渡す。**送信元ノードが 0x0002 で、checksum が合うものだけ**採用する。
- **フェイルセーフ（必須）:**
  - 有効なパケットが **300ms** 届かない、または flags.bit0 = 0 のときは、全ボタン OFF・軸中立の HID レポートを出す。
  - 途絶が始まった時刻と復帰した時刻を CDC にログ出力する。
- **HID の送信タイミング:** 状態が変わったときと、一定周期（例: 10ms）。
- **RSSI と seq の欠番:** `status` コマンドで確認できるようにする（電波状況のデバッグ用）。

## 6. 実機で確認すること（優先順）

1. [ ] IM920sL が起動メッセージを出し、`RDNN` などに応答するか（BUSY の極性も確認）
2. [ ] `STRT 1` の後の `STCH 31` が OK になるか（ダメなら ch01〜29 にし、送信頻度を下げる設計に変更）
3. [ ] ペアリング（STGN → RESET 端子で再起動）が成功し、`RDGN` が親機と子機で一致するか
4. [ ] PIO-USB（`DPDM` 指定）で元パッドが列挙されるか。dump でレポート形式を確認する
5. [ ] 送信頻度ごとの NG 率と遅延（20Hz / 30Hz / 50Hz）
6. [ ] 受信機が `/dev/input/js0` として認識され、`gamepad_related` がそのまま動くか
7. [ ] フェイルセーフ（送信機の電源断、アンテナ遮蔽）の動作
8. [ ] モバイルバッテリーが低電流で自動 OFF しないか（パッド＋XIAO＋IM920sL の消費電流）

## 7. 参考資料

- IM920sL 取扱説明書 Rev.1.5（2025-08-22、インタープラン）：コマンドは §10、端子は表15、チャネルは表1・表2
- IM920sL-ADP 製品資料 Rev.1.0（2026-02-10）：J1/J2 の対応表と回路図
- Pico-PIO-USB `src/pio_usb_configuration.h`（`PIO_USB_PINOUT`）
- 添付: `gamepad_wireless_schematic.{png,pdf}`、`gamepad_wireless_BOM.xlsx`、`im920sl_config.h`
