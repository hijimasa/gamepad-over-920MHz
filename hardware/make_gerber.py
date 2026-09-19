#!/usr/bin/env python3
"""発注用のガーバー + ドリルデータを出力する。

`make_board.py` が作った board/gamepad_over_920MHz.kicad_pcb から
board/gerber/ に一式を書き出し、最後に ZIP にまとめる。
"""
import os, shutil, pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PCB  = os.path.join(HERE, "board", "gamepad_over_920MHz.kicad_pcb")
OUT  = os.path.join(HERE, "board", "gerber")

shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT)

board = pcbnew.LoadBoard(PCB)

# ---- ガーバー ----
LAYERS = [
  (pcbnew.F_Cu,      "F_Cu",      "表面 銅箔"),
  (pcbnew.B_Cu,      "B_Cu",      "裏面 銅箔"),
  (pcbnew.F_Paste,   "F_Paste",   "表面 メタルマスク"),
  (pcbnew.B_Paste,   "B_Paste",   "裏面 メタルマスク"),
  (pcbnew.F_SilkS,   "F_Silkscreen", "表面 シルク"),
  (pcbnew.B_SilkS,   "B_Silkscreen", "裏面 シルク"),
  (pcbnew.F_Mask,    "F_Mask",    "表面 レジスト"),
  (pcbnew.B_Mask,    "B_Mask",    "裏面 レジスト"),
  (pcbnew.Edge_Cuts, "Edge_Cuts", "外形"),
]

pc = pcbnew.PLOT_CONTROLLER(board)
po = pc.GetPlotOptions()
po.SetOutputDirectory(OUT)
po.SetPlotFrameRef(False)
po.SetPlotValue(False)
po.SetPlotReference(True)
po.SetUseGerberProtelExtensions(True)   # .GTL/.GBL など。どの業者でも通りやすい
po.SetUseGerberX2format(False)
po.SetIncludeGerberNetlistInfo(False)
po.SetCreateGerberJobFile(False)
po.SetSubtractMaskFromSilk(True)        # レジスト上のシルクを削る
po.SetExcludeEdgeLayer(True)            # 外形は Edge_Cuts 単独で出す
po.SetUseAuxOrigin(False)

for layer, name, desc in LAYERS:
    pc.SetLayer(layer)
    pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_GERBER, desc)
    pc.PlotLayer()
pc.ClosePlot()

# ---- ドリル ----
dw = pcbnew.EXCELLON_WRITER(board)
dw.SetMapFileFormat(pcbnew.PLOT_FORMAT_PDF)
dw.SetOptions(
    False,                      # ミラーなし
    False,                      # ヘッダを省略しない
    board.GetDesignSettings().GetAuxOrigin(),
    False,                      # PTH と NPTH は分けて出す（固定穴は NPTH）
)
dw.SetFormat(True)              # メトリック
dw.CreateDrillandMapFilesSet(OUT, True, True)

# ---- ZIP ----
zip_base = os.path.join(HERE, "board", "gamepad_over_920MHz-gerber")
shutil.make_archive(zip_base, "zip", OUT)

files = sorted(os.listdir(OUT))
print(f"{OUT} に {len(files)} ファイル")
for f in files: print("  ", f)
print("ZIP:", zip_base + ".zip")
