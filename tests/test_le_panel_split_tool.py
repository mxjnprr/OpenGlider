"""Headless test of the FreeCAD LE Panel Split tool widget.

FreeCAD, FreeCADGui and pivy are stubbed; ``PySide`` is emulated with PySide6
(old PySide kept the widgets in QtGui).  The 3D drawing is replaced by a
recorder so the recoloured preview glider can be inspected.
"""
import importlib
import os
import sys
import types
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import openglider

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
TOOL_MODULE = "freecad.glider.tools.le_panel_split_tool"
LE_CUT = 0.10

try:
    from PySide6 import QtCore, QtGui, QtWidgets

    HAVE_QT = True
except ImportError:  # pragma: no cover
    HAVE_QT = False


# --------------------------------------------------------------------------
# stubs
# --------------------------------------------------------------------------
class _Meta(type):
    def __getattr__(cls, name):
        return _Node()


class _Node(metaclass=_Meta):
    """Accepts anything: used for every pivy / coin symbol."""

    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, name):
        return _Node()

    def __call__(self, *args, **kwargs):
        return _Node()

    def __iadd__(self, other):
        return self

    def __len__(self):
        return 0

    def __iter__(self):
        return iter(())

    def __getitem__(self, item):
        return _Node()


class _AnyModule(types.ModuleType):
    def __getattr__(self, name):
        return _Node


def _install_stubs():
    added = []

    def add(name, module):
        sys.modules[name] = module
        added.append(name)

    pivy = _AnyModule("pivy")
    coin = _AnyModule("pivy.coin")
    graphics = _AnyModule("pivy.graphics")
    graphics.COLORS = {}
    pivy.coin = coin
    pivy.graphics = graphics
    add("pivy", pivy)
    add("pivy.coin", coin)
    add("pivy.graphics", graphics)

    freecad_mod = types.ModuleType("FreeCAD")
    freecad_mod.ActiveDocument = types.SimpleNamespace(Objects=[], recompute=lambda: None)
    add("FreeCAD", freecad_mod)

    view = types.SimpleNamespace(
        getViewer=lambda: types.SimpleNamespace(getSoRenderManager=lambda: _Node()),
        getSceneGraph=lambda: _Node(),
        getNavigationType=lambda: "",
        viewTop=lambda: None,
    )
    gui = types.ModuleType("FreeCADGui")
    gui.ActiveDocument = types.SimpleNamespace(ActiveView=view)
    gui.Selection = types.SimpleNamespace(clearSelection=lambda: None)
    gui.Control = types.SimpleNamespace(closeDialog=lambda: None)
    add("FreeCADGui", gui)

    pyside = types.ModuleType("PySide")
    qtgui = types.ModuleType("PySide.QtGui")
    for src in (QtGui, QtWidgets):
        for name in dir(src):
            if not name.startswith("_"):
                setattr(qtgui, name, getattr(src, name))
    pyside.QtGui = qtgui
    pyside.QtCore = QtCore
    add("PySide", pyside)
    add("PySide.QtGui", qtgui)
    add("PySide.QtCore", QtCore)

    for name, rel in (
        ("freecad", "freecad"),
        ("freecad.glider", "freecad/glider"),
        ("freecad.glider.tools", "freecad/glider/tools"),
    ):
        pkg = types.ModuleType(name)
        pkg.__path__ = [os.path.join(ROOT, rel)]
        add(name, pkg)

    return added


class _Proxy:
    def __init__(self, g2d):
        self.g2d = g2d
        self.set_calls = []

    def getParametricGlider(self):
        return self.g2d

    def setParametricGlider(self, g2d):
        self.g2d = g2d
        self.set_calls.append(g2d)

    def drawGlider(self):
        pass


class _FreeCADObject:
    def __init__(self, g2d):
        self.Proxy = _Proxy(g2d)
        self.ViewObject = types.SimpleNamespace(Visibility=True)


def _glider():
    g2d = openglider.load(DEMOKITE)
    g2d.elements["cuts"].append(
        {"cells": [4, 5, 6], "left": -LE_CUT, "right": -LE_CUT, "type": "cut_3d"}
    )
    # legacy entry: stale cut_limit, one cell without LE cut (0), one out of range
    g2d.elements["le_panel_splits"] = [
        {"cut_limit": 0.25, "material_code": "", "cells": [0, 5, 6, 42]}
    ]
    return g2d


@unittest.skipUnless(HAVE_QT, "PySide6 not available")
class TestLEPanelSplitTool(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._added = _install_stubs()
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        for name in (
            TOOL_MODULE,
            "freecad.glider.tools.glider",
            "freecad.glider.tools.tools",
        ):
            sys.modules.pop(name, None)
        cls.module = importlib.import_module(TOOL_MODULE)

    @classmethod
    def tearDownClass(cls):
        for name in list(sys.modules):
            if name.startswith("freecad"):
                sys.modules.pop(name, None)
        for name in cls._added:
            sys.modules.pop(name, None)

    def setUp(self):
        self.drawn = []
        self.module.draw_glider = lambda glider, *a, **k: self.drawn.append(glider)
        self.obj = _FreeCADObject(_glider())
        self.tool = self.module.LEPanelSplitTool(self.obj)

    def _cells(self):
        return self.tool.parametric_glider.elements.get("le_panel_splits")

    def _item(self, cell_no):
        return self.tool.cells_list.item(cell_no)

    def test_list_reflects_le_cuts_and_stored_splits(self):
        self.assertEqual(self.tool.cells_list.count(), 7)
        for cell_no in range(4):
            item = self._item(cell_no)
            self.assertFalse(item.flags() & QtCore.Qt.ItemIsEnabled)
            self.assertIn("pas de coupe BA", item.text())
        for cell_no in (4, 5, 6):
            self.assertTrue(self._item(cell_no).flags() & QtCore.Qt.ItemIsUserCheckable)
        self.assertEqual(self._item(6).text(), "C7 : panneau BA de -10.0 % à +6.0 %")
        self.assertEqual(self._item(4).checkState(), QtCore.Qt.Unchecked)
        self.assertEqual(self._item(5).checkState(), QtCore.Qt.Checked)
        self.assertEqual(self._item(6).checkState(), QtCore.Qt.Checked)

    def test_legacy_entry_is_normalised_and_orphans_reported(self):
        self.assertEqual(self._cells(), [{"cells": [5, 6], "material_code": ""}])
        status = self.tool.status_label.text()
        self.assertIn("2 cellule(s) avec LE split : C6, C7", status)
        self.assertIn("C1, C43", status)
        # the document is untouched until accept()
        self.assertEqual(self.obj.Proxy.g2d.elements["le_panel_splits"][0]["cut_limit"], 0.25)

    def test_preview_recolours_the_two_halves(self):
        self.assertEqual(len(self.drawn), 1)
        glider = self.drawn[0]
        for cell_no in (5, 6):
            halves = [p for p in glider.cells[cell_no].panels if p.y_end - p.y_start < 1]
            self.assertEqual(
                sorted(p.material_code for p in halves),
                sorted([self.module.SPLIT_COLOR_INNER, self.module.SPLIT_COLOR_OUTER]),
            )
        self.assertEqual(
            [p for p in glider.cells[4].panels if p.y_end - p.y_start < 1], []
        )

    def test_toggling_a_cell_updates_data_and_preview(self):
        self._item(4).setCheckState(QtCore.Qt.Checked)
        self.assertEqual(self._cells()[0]["cells"], [4, 5, 6])
        self.assertEqual(len(self.drawn), 2)
        self._item(5).setCheckState(QtCore.Qt.Unchecked)
        self.assertEqual(self._cells()[0]["cells"], [4, 6])
        self.assertIn("C5, C7", self.tool.status_label.text())

    def test_range_buttons_skip_cells_without_le_cut(self):
        self.tool.range_start.setValue(1)
        self.tool.range_end.setValue(7)
        self.tool.check_range_button.click()
        self.assertEqual(self._cells()[0]["cells"], [4, 5, 6])
        self.tool.range_start.setValue(6)
        self.tool.range_end.setValue(5)  # reversed bounds are accepted
        self.tool.uncheck_range_button.click()
        self.assertEqual(self._cells()[0]["cells"], [6])

    def test_clear_all_removes_the_key(self):
        self.tool.clear_button.click()
        self.assertIsNone(self._cells())
        self.assertEqual(self.tool.status_label.text(), "Aucun LE split.")
        self.assertEqual(len(self.drawn), 2)

    def test_accept_writes_back_and_reject_keeps_document(self):
        self._item(4).setCheckState(QtCore.Qt.Checked)
        self.tool.accept()
        self.assertEqual(len(self.obj.Proxy.set_calls), 1)
        self.assertEqual(
            self.obj.Proxy.g2d.elements["le_panel_splits"],
            [{"cells": [4, 5, 6], "material_code": ""}],
        )
        other = _FreeCADObject(_glider())
        tool = self.module.LEPanelSplitTool(other)
        tool.clear_button.click()
        tool.reject()
        self.assertEqual(other.Proxy.set_calls, [])
        self.assertEqual(other.Proxy.g2d.elements["le_panel_splits"][0]["cells"], [0, 5, 6, 42])


if __name__ == "__main__":
    unittest.main()
