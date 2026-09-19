#!/usr/bin/env python3
"""フットプリント確認用の 1:1 印刷シートを作る。

印刷して実物を重ね、パッド位置が合うかを確かめる。
「用紙に合わせて拡大縮小」は必ず切って、100% で印刷すること。
"""
import os, sys, pcbnew
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_footprints import XIAO_ROW_SPACING

HERE = os.path.dirname(os.path.abspath(__file__))
LIB  = os.path.join(HERE, "footprints.pretty")
OUT  = os.path.join(HERE, "check")
os.makedirs(OUT, exist_ok=True)

def mm(v): return pcbnew.FromMM(v)

board = pcbnew.BOARD()
for name, x, y in (("IM920sL_ADP", 30, 35), ("XIAO_RP2040", 75, 35)):
    fp = pcbnew.FootprintLoad(LIB, name)
    assert fp, f"フットプリントを読めません: {name}"
    fp.SetPosition(pcbnew.wxPoint(mm(x), mm(y)))
    board.Add(fp)

# 外形（印刷範囲）
pts = [(5,5),(105,5),(105,75),(5,75),(5,5)]
for (x1,y1),(x2,y2) in zip(pts, pts[1:]):
    seg = pcbnew.PCB_SHAPE(board)
    seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
    seg.SetStart(pcbnew.wxPoint(mm(x1), mm(y1)))
    seg.SetEnd(pcbnew.wxPoint(mm(x2), mm(y2)))
    seg.SetLayer(pcbnew.Edge_Cuts)
    seg.SetWidth(mm(0.15))
    board.Add(seg)

def text(x, y, s, size=2.0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(pcbnew.wxPoint(mm(x), mm(y)))
    t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.wxSize(mm(size), mm(size)))
    t.SetTextThickness(mm(size/8))
    t.SetHorizJustify(pcbnew.GR_TEXT_HJUSTIFY_LEFT)
    board.Add(t)

def hline(x1, x2, y):
    seg = pcbnew.PCB_SHAPE(board)
    seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
    seg.SetStart(pcbnew.wxPoint(mm(x1), mm(y)))
    seg.SetEnd(pcbnew.wxPoint(mm(x2), mm(y)))
    seg.SetLayer(pcbnew.F_SilkS); seg.SetWidth(mm(0.25)); board.Add(seg)

text(8, 10, "footprint check sheet - PRINT AT 100% (no scaling)", 2.2)
text(15, 60, "IM920sL-ADP", 2.0)
text(15, 63, "J1-J2 = 25.4mm (from datasheet)", 1.6)
text(62, 60, "XIAO RP2040", 2.0)
text(62, 63, "row = %.2fmm  <-- VERIFY" % XIAO_ROW_SPACING, 1.6)
# 1:1 確認用の 50mm 基準線
hline(10, 60, 70)
for x in (10, 60):
    seg = pcbnew.PCB_SHAPE(board); seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
    seg.SetStart(pcbnew.wxPoint(mm(x), mm(68.5))); seg.SetEnd(pcbnew.wxPoint(mm(x), mm(71.5)))
    seg.SetLayer(pcbnew.F_SilkS); seg.SetWidth(mm(0.25)); board.Add(seg)
text(24, 68, "50.0mm reference", 2.0)

pcbnew.SaveBoard(os.path.join(OUT, "checksheet.kicad_pcb"), board)

pc = pcbnew.PLOT_CONTROLLER(board)
po = pc.GetPlotOptions()
po.SetOutputDirectory(OUT)
po.SetPlotFrameRef(False)
po.SetAutoScale(False)
po.SetScale(1)
po.SetMirror(False)
po.SetUseGerberAttributes(False)
for fmt, ext in ((pcbnew.PLOT_FORMAT_SVG, "svg"), (pcbnew.PLOT_FORMAT_PDF, "pdf")):
    pc.SetLayer(pcbnew.F_SilkS)
    pc.OpenPlotfile("checksheet", fmt, "footprint check")
    pc.PlotLayer()
    pc.SetLayer(pcbnew.F_Cu); pc.PlotLayer()
    pc.SetLayer(pcbnew.Edge_Cuts); pc.PlotLayer()
    pc.ClosePlot()
print("出力:", os.listdir(OUT))
