#!/usr/bin/env python3
"""モジュール用フットプリントを生成する。

寸法の根拠:
  IM920sL-ADP … 製品資料 Rev.1.0 外形寸法図
                 外形 30.0 x 40.0mm、J1-J2 間 25.4mm、ピン1 は上端から 5.71mm、
                 ピッチ 2.54mm x 10（span 22.86mm）
  XIAO RP2040 … ROW_SPACING は実測で確定させること（後述）
"""
import os

OUT = os.path.join(os.path.dirname(__file__), "footprints.pretty")

# ★ 要実測 ★ XIAO RP2040 の 2 列のピン間隔（中心間・mm）
# 資料により 15.24mm（0.6in）と 17.78mm（0.7in）の両方の記述があり確定できていない。
# 実物をノギスで測って正しい値に直すこと。1:1 印刷での確認手段は hardware/README.md 参照。
XIAO_ROW_SPACING = 15.24   # 実測で確定（0.6 inch）
XIAO_PITCH   = 2.54
XIAO_PINS    = 7          # 片側 7 ピン
XIAO_W, XIAO_H = 17.8, 21.0   # 外形（参考値）

ADP_ROW_SPACING = 25.4    # J1-J2 間（製品資料で確認済み）
ADP_PITCH    = 2.54
ADP_PINS     = 10
ADP_W, ADP_H = 30.0, 40.0
ADP_PIN1_FROM_TOP = 5.71

def pad(n, x, y, shape="circle"):
    r = "rect" if shape == "rect" else "circle"
    return (f'  (pad "{n}" thru_hole {r} (at {x:.3f} {y:.3f}) (size 1.7 1.7) '
            f'(drill 1.0) (layers *.Cu *.Mask) (zone_connect 2))')

def rect(layer, x1, y1, x2, y2, width=0.12):
    return (f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y1:.3f}) '
            f'(layer "{layer}") (width {width}))\n'
            f'  (fp_line (start {x2:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(layer "{layer}") (width {width}))\n'
            f'  (fp_line (start {x2:.3f} {y2:.3f}) (end {x1:.3f} {y2:.3f}) '
            f'(layer "{layer}") (width {width}))\n'
            f'  (fp_line (start {x1:.3f} {y2:.3f}) (end {x1:.3f} {y1:.3f}) '
            f'(layer "{layer}") (width {width}))')

def build(name, descr, w, h, rowspan, pitch, npins, pin1_y, left_nums, right_nums):
    """左列 = left_nums の順、右列 = right_nums の順でパッド番号を振る"""
    lines = [f'(footprint "{name}" (version 20211014) (generator wg_make_footprints)',
             '  (layer "F.Cu")',
             f'  (descr "{descr}")',
             '  (attr through_hole)',
             f'  (fp_text reference "REF**" (at 0 {-h/2-1.5:.2f}) (layer "F.SilkS")'
             ' (effects (font (size 1 1) (thickness 0.15))))',
             f'  (fp_text value "{name}" (at 0 {h/2+1.5:.2f}) (layer "F.Fab")'
             ' (effects (font (size 1 1) (thickness 0.15))))']
    lines.append(rect("F.SilkS", -w/2, -h/2, w/2, h/2))
    lines.append(rect("F.CrtYd", -w/2-0.3, -h/2-0.3, w/2+0.3, h/2+0.3, 0.05))
    for i in range(npins):
        y = pin1_y + i*pitch
        lines.append(pad(left_nums[i],  -rowspan/2, y, "rect" if i == 0 else "circle"))
        lines.append(pad(right_nums[i],  rowspan/2, y))
    lines.append(')')
    open(os.path.join(OUT, name + ".kicad_mod"), "w").write("\n".join(lines) + "\n")
    print("生成:", name)

# ---- XIAO RP2040 ----
# 左列（上から）D0..D6 = パッド 1..7 / 右列（上から）5V,GND,3V3,D10,D9,D8,D7 = 8..14
build("XIAO_RP2040", "Seeed XIAO RP2040 (through-hole header). ROW SPACING REQUIRES VERIFICATION",
      XIAO_W, XIAO_H, XIAO_ROW_SPACING, XIAO_PITCH, XIAO_PINS,
      -(XIAO_PINS-1)*XIAO_PITCH/2,
      [str(i) for i in range(1, 8)], [str(i) for i in range(8, 15)])

# ---- IM920sL-ADP ----
# 左列 = J1-1..J1-10 → パッド 1..10 / 右列 = J2-1..J2-10 → パッド 11..20
build("IM920sL_ADP", "IM920sL + IM920sL-ADP. Dimensions from ADP product data Rev.1.0",
      ADP_W, ADP_H, ADP_ROW_SPACING, ADP_PITCH, ADP_PINS,
      ADP_PIN1_FROM_TOP - ADP_H/2,
      [str(i) for i in range(1, 11)], [str(i) for i in range(11, 21)])
