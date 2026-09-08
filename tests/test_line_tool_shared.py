"""Headless test of the *shared L/R* checkbox of the FreeCAD line tool.

FreeCAD, FreeCADGui and pivy are stubbed (see test_le_panel_split_tool); the
pivy container of the tool is patched to record the markers and lines the
tool creates, so the load / edit / accept round trip of ``Line2D.shared`` can
be checked without FreeCAD.
"""
import importlib
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import test_le_panel_split_tool as tool_stubs  # noqa: E402

import openglider  # noqa: E402
from openglider.glider.parametric.lines import UpperNode2D  # noqa: E402

try:
    from PySide6 import QtWidgets

    HAVE_QT = True
except ImportError:  # pragma: no cover
    HAVE_QT = False

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
TOOL_MODULE = "freecad.glider.tools.line_tool"


class _Proxy(tool_stubs._Proxy):
    def getGliderInstance(self):
        return self.g2d.get_glider_3d()


class _FreeCADObject(tool_stubs._FreeCADObject):
    def __init__(self, g2d):
        super().__init__(g2d)
        self.Proxy = _Proxy(g2d)


def _pick_lowest_line(lineset):
    """a lowest line (handle -> knot) long enough to reach the symmetry plane"""
    for node in lineset.get_lower_attachment_points():
        for line in lineset.get_upper_connected_lines(node):
            if not isinstance(line.upper_node, UpperNode2D) and (
                line.target_length or 0
            ) > abs(node.pos_3D[1]) + 0.1:
                return line
    return None


@unittest.skipUnless(HAVE_QT, "PySide6 not available")
class TestLineToolShared(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls._added = tool_stubs._install_stubs()
        view = sys.modules["FreeCADGui"].ActiveDocument.ActiveView
        view.addEventCallbackPivy = lambda *a, **k: tool_stubs._Node()
        view.removeEventCallbackPivy = lambda *a, **k: None
        # Marker/Line of the tool call super().set_enabled(): the stub base
        # class needs real (no-op) methods, instance __getattr__ is not enough
        tool_stubs._Node.set_enabled = lambda self: None
        tool_stubs._Node.set_disabled = lambda self: None

        # remember the points a marker/line is built with (NodeMarker.pos)
        def init(self, points=None, *args, **kwargs):
            self.__dict__["_points"] = list(points) if points is not None else []

        tool_stubs._Node.__init__ = init
        cls.module = importlib.import_module(TOOL_MODULE)
        cls.module.NodeMarker.points = property(
            lambda self: self.__dict__.get("_points", []),
            lambda self, value: self.__dict__.__setitem__("_points", list(value)),
        )
        # colour changes only
        for klass in (cls.module.Marker, cls.module.Line):
            klass.set_enabled = lambda self: None
            klass.set_disabled = lambda self: None

        def record(self, objs):
            self.__dict__.setdefault("dynamic_objects", []).extend(objs)
            return self

        cls.module.LineContainer.__iadd__ = record

    @classmethod
    def tearDownClass(cls):
        for name in list(sys.modules):
            if name.startswith("freecad"):
                sys.modules.pop(name, None)
        for name in cls._added:
            sys.modules.pop(name, None)

    def setUp(self):
        g2d = openglider.load(DEMOKITE)
        flagged = _pick_lowest_line(g2d.lineset)
        if flagged is None:
            self.skipTest("no lowest line long enough in the demo kite")
        flagged.shared = True
        flagged.name = "flagged"
        self.obj = _FreeCADObject(g2d)
        self.tool = self.module.LineTool(self.obj)

    def _lines(self):
        return [
            obj
            for obj in self.tool.shape.dynamic_objects
            if isinstance(obj, self.module.ConnectionLine)
        ]

    def _plain_lines(self):
        return [
            line
            for line in self._lines()
            if line.name != "flagged" and not line.is_uppermost_line()
        ]

    def test_flag_is_loaded_and_shown_by_the_checkbox(self):
        flagged = [line for line in self._lines() if line.name == "flagged"]
        self.assertEqual(len(flagged), 1)
        self.assertTrue(flagged[0].shared)
        plain = self._plain_lines()
        self.assertTrue(plain)
        self.assertFalse(any(line.shared for line in plain))

        self.tool.shape.selected_objects = flagged
        self.tool.selection_changed()
        self.assertTrue(self.tool.QLineShared.isChecked())

        self.tool.shape.selected_objects = plain[:1]
        self.tool.selection_changed()
        self.assertFalse(self.tool.QLineShared.isChecked())

    def test_checkbox_edits_the_selected_lines(self):
        plain = self._plain_lines()[:2]
        self.tool.shape.selected_objects = plain
        self.tool.selection_changed()
        self.tool.QLineShared.setChecked(True)
        self.assertTrue(all(line.shared for line in plain))
        self.tool.QLineShared.setChecked(False)
        self.assertFalse(any(line.shared for line in plain))

    def test_accept_writes_the_flag_to_the_parametric_glider(self):
        plain = self._plain_lines()[0]
        plain.name = "newly_shared"
        plain.shared = True
        self.tool.accept()
        lineset = self.obj.Proxy.g2d.lineset
        shared = sorted(line.name for line in lineset.lines if line.shared)
        self.assertEqual(shared, ["flagged", "newly_shared"])
        self.assertEqual(
            len([line for line in lineset.lines if line.shared]), 2
        )
        # the accepted lineset is what the 3d glider is built from
        leg = self.obj.Proxy.g2d.get_glider_3d().lineset["flagged"]
        self.assertTrue(leg.shared)
        self.assertAlmostEqual(leg.upper_node.vec[1], 0.0, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
