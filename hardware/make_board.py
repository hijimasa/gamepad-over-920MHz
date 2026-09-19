#!/usr/bin/env python3
"""基板（送信機・受信機 共用）を生成する。

受信機は J1(USB-A)・C1・R1・R2 を実装しないだけで、基板は同じものを使う。
接続は docs/netlist.md と同じ。
"""
import os, sys, pcbnew
HERE = os.path.dirname(os.path.abspath(__file__))
LIB  = os.path.join(HERE, "footprints.pretty")
KFP  = "/usr/share/kicad/footprints"
OUT  = os.path.join(HERE, "board")
os.makedirs(OUT, exist_ok=True)

def mm(v): return pcbnew.FromMM(v)
def pt(x, y): return pcbnew.wxPoint(mm(x), mm(y))

BOARD_W, BOARD_H = 90.0, 50.0

board = pcbnew.BOARD()

# ---- 外形 ----
edge = [(0,0),(BOARD_W,0),(BOARD_W,BOARD_H),(0,BOARD_H),(0,0)]
for (x1,y1),(x2,y2) in zip(edge, edge[1:]):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pt(x1,y1)); s.SetEnd(pt(x2,y2))
    s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.15)); board.Add(s)

# ---- 部品の配置 ----
#   U2(ADP) を左端に寄せ、その左側をアンテナ用に空ける
#   U1(XIAO) を右に、USB-A の DIP 基板(J1)を右端に置く
# U2 は 180 度回し、信号の多い J1 側（BUSY/RESET/TxD/VCC）を XIAO に向ける。
# 左端はアンテナ用に空けておく。
PLACE = [
  ("U2", LIB, "IM920sL_ADP",                                      22.0, 25.0, 180),
  ("U1", LIB, "XIAO_RP2040",                                      57.0, 18.0,   0),
  ("C2", KFP+"/Capacitor_THT.pretty", "CP_Radial_D5.0mm_P2.00mm",  43.0, 13.0,   0),
  ("C3", KFP+"/Capacitor_THT.pretty", "C_Disc_D5.0mm_W2.5mm_P5.00mm", 43.0, 23.0, 90),
  ("R2", KFP+"/Resistor_THT.pretty", "R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 57.0, 33.0, 0),
  ("R1", KFP+"/Resistor_THT.pretty", "R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 57.0, 38.0, 0),
  ("J1", KFP+"/Connector_PinHeader_2.54mm.pretty", "PinHeader_1x04_P2.54mm_Vertical", 74.0, 35.5, 0),
  ("C1", KFP+"/Capacitor_THT.pretty", "CP_Radial_D5.0mm_P2.00mm",  82.0, 35.5,   0),
]
fps = {}
for ref, lib, name, x, y, rot in PLACE:
    fp = pcbnew.FootprintLoad(lib, name)
    assert fp, f"読み込み失敗: {lib}/{name}"
    fp.SetPosition(pt(x, y))
    if rot: fp.SetOrientationDegrees(rot)
    fp.SetReference(ref)
    board.Add(fp)
    fps[ref] = fp

# ---- ネット ----
NETS = {
 "+5V":        [("U1",8), ("J1",1), ("C1",1)],
 "+3V3":       [("U1",10), ("U2",9), ("C2",1), ("C3",1)],
 "GND":        [("U1",9), ("U2",19), ("J1",4), ("C1",2), ("C2",2), ("C3",2)],
 "USB_DP":     [("U1",1), ("R2",1)],
 "USB_DP_CON": [("R2",2), ("J1",3)],
 "USB_DM":     [("U1",2), ("R1",1)],
 "USB_DM_CON": [("R1",2), ("J1",2)],
 "IM_BUSY":    [("U1",3), ("U2",1)],
 "IM_RESET":   [("U1",4), ("U2",10)],
 "IM_RXD":     [("U1",7), ("U2",13)],
 "IM_TXD":     [("U1",14), ("U2",4)],
}
for name, pads in NETS.items():
    net = pcbnew.NETINFO_ITEM(board, name)
    board.Add(net)
    for ref, num in pads:
        pad = fps[ref].FindPadByNumber(str(num))
        assert pad, f"パッドがありません: {ref}.{num}"
        pad.SetNet(net)

pcbnew.SaveBoard(os.path.join(OUT, "gamepad_over_920MHz.kicad_pcb"), board)
print(f"部品 {len(fps)} / ネット {len(NETS)} / 外形 {BOARD_W}x{BOARD_H}mm")

# ---- 描画（確認用）----
def plot(tag, layers):
    pc = pcbnew.PLOT_CONTROLLER(board)
    po = pc.GetPlotOptions()
    po.SetOutputDirectory(OUT); po.SetPlotFrameRef(False)
    po.SetAutoScale(False); po.SetScale(1); po.SetMirror(False)
    po.SetPlotReference(True); po.SetPlotValue(False)
    pc.OpenPlotfile(tag, pcbnew.PLOT_FORMAT_PDF, tag)
    for l in layers:
        pc.SetLayer(l); pc.PlotLayer()
    pc.ClosePlot()

plot("layout", [pcbnew.F_SilkS, pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.Edge_Cuts])
print("描画しました")

# ---- DRC ----
rpt = os.path.join(OUT, "drc.txt")
pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
print("DRC:", rpt)
