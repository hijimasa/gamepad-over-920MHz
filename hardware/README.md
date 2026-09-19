# 基板データ（KiCad）

原理試作用の基板。送信機と受信機は **1 種類の基板**でまかなう
（受信機は USB-A まわりを実装しないだけ。[../docs/netlist.md](../docs/netlist.md) 参照）。

## 構成

```
hardware/
├── make_footprints.py   モジュール用フットプリントの生成
├── make_checksheet.py   寸法確認用 1:1 印刷シートの生成
├── footprints.pretty/   生成されたフットプリント
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
| XIAO RP2040 | 2 列の間隔 **17.78mm**（暫定） | **未確認。要実測** |

XIAO の列間隔は資料により 15.24mm（0.6in）と 17.78mm（0.7in）の記述があり確定できていない。
**ノギスで実測して `make_footprints.py` の `XIAO_ROW_SPACING` を直すこと。**
ここを間違えると基板が使えない。

## 生成と確認

```bash
python3 hardware/make_footprints.py     # フットプリント
python3 hardware/make_checksheet.py     # 確認シート
```

KiCad 6 の Python API（`pcbnew`）を使っている。KiCad 7 以降なら `kicad-cli` で
同じことがもっと簡単にできる。

## 設計方針

レイアウトの制約は [../docs/netlist.md](../docs/netlist.md) にまとめてある。要点だけ再掲:

1. **IM920sL のワイヤーアンテナ周辺はベタ GND・配線・金属を置かない。** 基板端に寄せ、
   できればアンテナを基板外へ出す。飛距離に直結する
2. **USB の D+/D− は短く・等長・並走。** R1/R2（22Ω）は XIAO 側に寄せる。
   PIO-USB はタイミング余裕が少ない（CPU 120MHz では列挙に失敗する）
3. C3（0.1µF）は IM920sL の VCC 直近、C2（10µF）はその外側
4. C1（47µF）は USB-A の VBUS 直近

RF 性能は保証できない。原理試作として作り、実測しながら直す前提。
