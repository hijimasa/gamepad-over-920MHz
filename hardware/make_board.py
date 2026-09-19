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

BOARD_W, BOARD_H = 100.0, 62.0
MOUNT_D   = 3.2                       # 固定穴の径（M3 用）
MOUNT_POS = [(4.0,4.0),(96.0,4.0),(4.0,58.0),(96.0,58.0)]

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
# 配置の方針
#  ・U2(ADP) は回転 0。アンテナは J1 のピン1 付近から出るので、それが基板の
#    左上（空き側）に来る。ADP は裏面にコネクタがあるのでピンソケットで嵩上げする
#  ・U1(XIAO) は 180 度回転。USB-C が下端を向き、バッテリーのケーブルを逃がせる
#  ・J1 は USB-A レセプタクルを直付け（変換基板なし）。右端に開口を向ける
#  ・D+/D− は XIAO の D0/D1 から R2/R1 を経て J1 へ、短く並走させる
RUSB = KFP+"/Resistor_THT.pretty/R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal"
PLACE = [
  ("U2", LIB, "IM920sL_ADP",                                      22.0, 32.0,   0),
  ("U1", LIB, "XIAO_RP2040",                                      62.0, 41.5, 180),
  ("C3", KFP+"/Capacitor_THT.pretty", "C_Disc_D5.0mm_W2.5mm_P5.00mm",  3.6, 38.0, 90),
  ("C2", KFP+"/Capacitor_THT.pretty", "CP_Radial_D5.0mm_P2.00mm",      3.6, 46.0, 90),
  ("R2", KFP+"/Resistor_THT.pretty", "R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal", 76.0, 49.0, 0),
  ("R1", KFP+"/Resistor_THT.pretty", "R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal", 76.0, 44.0, 0),
  ("J1", KFP+"/Connector_USB.pretty", "USB_A_CONNFLY_DS1095-WNR0",     86.5, 44.0, 90),
  ("C1", KFP+"/Capacitor_THT.pretty", "CP_Radial_D5.0mm_P2.00mm",      93.0, 22.0, 90),
]
# 四隅の固定穴（φ3.2 / M3）。部品と配線を避けた位置に置く
MH = KFP+"/MountingHole.pretty"
for i,(mx,my) in enumerate(MOUNT_POS, 1):
    PLACE.append((f"H{i}", MH, "MountingHole_3.2mm_M3", mx, my, 0))

fps = {}
for ref, lib, name, x, y, rot in PLACE:
    fp = pcbnew.FootprintLoad(lib, name)
    assert fp, f"読み込み失敗: {lib}/{name}"
    fp.SetPosition(pt(x, y))
    if rot: fp.SetOrientationDegrees(rot)
    fp.SetReference(ref)
    board.Add(fp)
    fps[ref] = fp

# シルクが基板外や穴に掛からないように調整する
for ref in ("H1","H2","H3","H4"):
    fps[ref].Reference().SetVisible(False)      # 固定穴は部品ではないので番号を出さない
fps["C2"].Reference().SetPosition(pt(3.6, 50.5))
fps["C3"].Reference().SetPosition(pt(3.6, 41.0))

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
    if name == "GND":
        # J1 のシェル固定パッド（どちらも番号 5）も GND に落とす
        for pad in fps["J1"].Pads():
            if pad.GetNumber() == "5": pad.SetNet(net)

# ---- 配線 ----
# F = 表面、B = 裏面。
# D+ は表面、D− は裏面に分けている。XIAO 側は D0(D+) が D1(D−) の下、
# J1 側は D+ が D− の上なので順序が逆転しており、1 層では必ず交差するため。
# フルスピード(12Mbps) で 20mm 程度なので、層を分けても実用上は問題ない。
F, B = pcbnew.F_Cu, pcbnew.B_Cu
ROUTES = [
  ("USB_DP",     F, [(69.62,49.12),(76.0,49.0)]),
  ("USB_DP_CON", F, [(81.08,49.0),(84.0,49.0),(84.0,39.5),(86.5,39.5)]),
  ("USB_DM",     B, [(69.62,46.58),(74.5,46.58),(74.5,44.0),(76.0,44.0)]),
  ("USB_DM_CON", B, [(81.08,44.0),(83.5,44.0),(83.5,41.5),(86.5,41.5)]),
  # 電源。J1 のシェルパッド(φ3.5)を避けて右端を通す
  ("+5V",  F, [(54.38,49.12),(54.38,54.0),(92.0,54.0),(92.0,44.0),(86.5,44.0)]),
  ("+5V",  F, [(92.0,44.0),(92.0,22.0),(93.0,22.0)]),
  ("+3V3", F, [(3.6,46.0),(1.5,46.0),(1.5,38.0),(3.6,38.0)]),
  ("+3V3", F, [(3.6,38.0),(9.3,38.03)]),
  ("+3V3", F, [(9.3,38.03),(7.0,38.03),(7.0,42.5),(42.0,42.5),(42.0,44.04),(54.38,44.04)]),
  # 制御線
  ("IM_BUSY",  F, [(9.3,17.71),(13.0,17.71),(13.0,14.0),(50.0,14.0),(50.0,26.0),
                   (73.0,26.0),(73.0,44.04),(69.62,44.04)]),
  ("IM_RESET", B, [(9.3,40.57),(9.3,45.5),(46.0,45.5),(46.0,57.0),(78.0,57.0),
                   (78.0,41.5),(69.62,41.5)]),
  ("IM_TXD",   B, [(9.3,25.33),(13.0,25.33),(13.0,29.14),(40.0,29.14),(40.0,33.88),(54.38,33.88)]),
  ("IM_RXD",   B, [(34.7,22.79),(44.0,22.79),(44.0,18.0),(74.0,18.0),(74.0,33.88),(69.62,33.88)]),
]
nets = {n.GetNetname(): n for n in board.GetNetsByName().values()}
for name, layer, pts in ROUTES:
    net = nets[name]
    for (x1,y1),(x2,y2) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pt(x1,y1)); t.SetEnd(pt(x2,y2))
        t.SetWidth(mm(0.4)); t.SetLayer(layer); t.SetNet(net)
        board.Add(t)



PCB = os.path.join(OUT, "gamepad_over_920MHz.kicad_pcb")
pcbnew.SaveBoard(PCB, board)

# ---- GND ベタ ----
# 生成直後の BOARD では ZONE_FILLER が落ちるので、一度保存して読み直してから張る。
# アンテナ直下に銅を残さないよう、左上（U2 のピン1 側）を切り欠いた外形にする。
ANT_KEEPOUT_X, ANT_KEEPOUT_Y = 22.0, 20.0
board = pcbnew.LoadBoard(PCB)
gnd = board.GetNetsByName()["GND"]
zone_outline = [
  (ANT_KEEPOUT_X, 1.0), (BOARD_W-1.0, 1.0), (BOARD_W-1.0, BOARD_H-1.0),
  (1.0, BOARD_H-1.0), (1.0, ANT_KEEPOUT_Y), (ANT_KEEPOUT_X, ANT_KEEPOUT_Y),
]
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer); z.SetNet(gnd)
    z.SetLocalClearance(mm(0.3)); z.SetMinThickness(mm(0.25))
    poly = z.Outline(); poly.NewOutline()
    for (x, y) in zone_outline: poly.Append(mm(x), mm(y))
    board.Add(z)
pcbnew.ZONE_FILLER(board).Fill(board.Zones())

# ---- シルク ----
def silk(text, x, y, size=1.2, layer=pcbnew.F_SilkS):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text); t.SetPosition(pt(x, y)); t.SetLayer(layer)
    t.SetTextSize(pcbnew.wxSize(mm(size), mm(size))); t.SetTextThickness(mm(size/6))
    board.Add(t)

silk("gamepad-over-920MHz", 50.0, 60.0, 1.6)
silk("ANT KEEPOUT", 13.0, 9.0, 1.2)      # ベタを抜いた領域。アンテナ線はここから外へ
silk("TX: mount all / RX: omit J1,C1,R1,R2", 50.0, 3.0, 1.2)

pcbnew.SaveBoard(PCB, board)
print(f"部品 {len(fps)} / ネット {len(NETS)} / 外形 {BOARD_W}x{BOARD_H}mm / 固定穴 φ{MOUNT_D}x{len(MOUNT_POS)}")

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

plot("layout",  [pcbnew.F_SilkS, pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.Edge_Cuts])
plot("top",     [pcbnew.F_SilkS, pcbnew.F_Cu, pcbnew.Edge_Cuts])
plot("bottom",  [pcbnew.F_SilkS, pcbnew.B_Cu, pcbnew.Edge_Cuts])
print("描画しました")

# ---- DRC ----
rpt = os.path.join(OUT, "drc.txt")
pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
print("DRC:", rpt)
