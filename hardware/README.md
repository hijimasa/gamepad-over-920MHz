# 基板データ（KiCad）

原理試作用の基板。送信機と受信機は **1 種類の基板**でまかなう
（受信機は USB-A まわりを実装しないだけ。[../docs/netlist.md](../docs/netlist.md) 参照）。

## 構成

```
hardware/
├── make_footprints.py   モジュール用フットプリントの生成
├── make_board.py        基板（配置・配線・ベタ・DRC・作図）の生成
├── make_checksheet.py   寸法確認用 1:1 印刷シートの生成
├── footprints.pretty/   生成されたフットプリント
├── board/               基板データ（.kicad_pcb）と確認用 PDF、DRC レポート
└── check/               確認シート（PDF/SVG）
```

フットプリントは手書きではなくスクリプトで生成している。寸法が判明したら
`make_footprints.py` の定数を直して再生成する。

## ⚠ 実装前に寸法を確認すること

**`check/footprint_check.pdf` を 100%（拡大縮小なし）で印刷し、実物を重ねて確認する。**
シートに 50.0mm の基準線を入れてあるので、まずそれを定規で測って印刷倍率を確かめる。

| 部品 | 寸法 | 根拠 |
|---|---|---|
| IM920sL-ADP | J1–J2 間 **25.4mm**、ピン1 は上端から 5.71mm、ピッチ 2.54mm×10、外形 30.0×40.0mm | 製品資料 Rev.1.0 外形寸法図で確認済み |
| XIAO RP2040 | 2 列の間隔 **15.24mm**（0.6in）、ピッチ 2.54mm×7 | 実測で確認済み |

どちらも実測で確定済みだが、発注前にもう一度確認シートを重ねること。
ここを間違えると基板が使えない。

## 生成と確認

```bash
python3 hardware/make_footprints.py     # フットプリント
python3 hardware/make_board.py          # 基板 + DRC + 確認用 PDF
python3 hardware/make_checksheet.py     # 確認シート
```

`make_board.py` は実行のたびに `board/` を作り直し、最後に DRC を回して
`board/drc.txt` に結果を書く。**現状 0 violations / 0 unconnected pads。**

KiCad 6 の Python API（`pcbnew`）を使っている。KiCad 7 以降なら `kicad-cli` で
同じことがもっと簡単にできる。

## 基板の概要

| 項目 | 値 |
|---|---|
| 外形 | 100 × 62 mm、2 層 |
| 固定穴 | φ3.2（M3）× 4、四隅（4,4）（96,4）（4,58）（96,58） |
| 配線幅 | 0.4mm（全ネット） |
| クリアランス | 0.2mm（ベタは 0.3mm） |
| ベタ | 両面 GND。左上 22 × 20mm はアンテナ用に銅を抜いてある（シルクに `ANT KEEPOUT`） |

確認用の作図:

| ファイル | 内容 |
|---|---|
| `board/gamepad_over_920MHz-top.pdf` | 表面（F.Cu + シルク） |
| `board/gamepad_over_920MHz-bottom.pdf` | 裏面（B.Cu） |
| `board/gamepad_over_920MHz-layout.pdf` | 両面重ね |
| `board/placement.pdf` | 部品配置のみ |

配置は次の条件で決めてある:

- **U2（IM920sL-ADP）は回転 0。** アンテナが J1 のピン1 付近から出るので、
  それが基板左上の空き側（ANT KEEPOUT）に来る。ADP は裏面にコネクタがあるため、
  **ピンソケットで嵩上げして実装する**
- **U1（XIAO RP2040）は 180 度回転。** USB-C が基板下端を向き、
  モバイルバッテリーのケーブルを基板外へ逃がせる
- **J1（USB-A）は変換基板を使わず直付け。** 右端に開口を向ける。
  シェル固定パッドは GND に落としてある

### D+/D− を表裏に分けている理由

XIAO 側は D0(D+) が D1(D−) の**下**、J1 側は D+ が D− の**上**で順序が逆転しており、
1 層では必ず交差する。そのため **D+ は表面、D− は裏面**に分けた。
フルスピード（12Mbps）で配線長 20mm 程度なので実用上は問題ないが、
インピーダンス整合は取っていない。

## 設計方針

レイアウトの制約は [../docs/netlist.md](../docs/netlist.md) にまとめてある。要点だけ再掲:

1. **IM920sL のワイヤーアンテナ周辺はベタ GND・配線・金属を置かない。** 基板端に寄せ、
   できればアンテナを基板外へ出す。飛距離に直結する
2. **USB の D+/D− は短く・等長。** R1/R2（22Ω）は XIAO 側に寄せる。
   PIO-USB はタイミング余裕が少ない（CPU 120MHz では列挙に失敗する）。
   ※ 実際の配線では並走ではなく表裏に分けている（理由は上記）
3. C3（0.1µF）は IM920sL の VCC 直近、C2（10µF）はその外側
4. C1（47µF）は USB-A の VBUS 直近

RF 性能は保証できない。原理試作として作り、実測しながら直す前提。
