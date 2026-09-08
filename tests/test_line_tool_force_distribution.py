"""Headless test of the attachment-point forces in the FreeCAD line tool.

FreeCAD, FreeCADGui and pivy are stubbed (see test_le_panel_split_tool /
test_line_tool_shared). Checks that

- selecting several attachment points only *shows* the first one's force /
  position (setValue() used to emit valueChanged and overwrite every selected
  point),
- the "Distribute forces..." button writes the automatic forces to the
  parametric nodes and the markers, and remembers the dialog parameters.
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

try:
    from PySide6 import QtWidgets

    HAVE_QT = True
except ImportError:  # pragma: no cover
    HAVE_QT = False

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
TOOL_MODULE = "freecad.glider.tools.line_tool"
DIALOG_MODULE = "freecad.glider.tools.lines_force_distribution_dialog"


class _Proxy(tool_stubs._Proxy):
    def getGliderInstance(self):
        return self.g2d.get_glider_3d()


class _FreeCADObject(tool_stubs._FreeCADObject):
    def __init__(self, g2d):
        super().__init__(g2d)
        self.Proxy = _Proxy(g2d)


@unittest.skipUnless(HAVE_QT, "PySide6 not available")
class TestLineToolForceDistribution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls._added = tool_stubs._install_stubs()
        view = sys.modules["FreeCADGui"].ActiveDocument.ActiveView
        view.addEventCallbackPivy = lambda *a, **k: tool_stubs._Node()
        view.removeEventCallbackPivy = lambda *a, **k: None
        tool_stubs._Node.set_enabled = lambda self: None
        tool_stubs._Node.set_disabled = lambda self: None

        def init(self, points=None, *args, **kwargs):
            self.__dict__["_points"] = list(points) if points is not None else []

        tool_stubs._Node.__init__ = init
        cls.module = importlib.import_module(TOOL_MODULE)
        cls.dialog_module = importlib.import_module(DIALOG_MODULE)
        cls.module.NodeMarker.points = property(
            lambda self: self.__dict__.get("_points", []),
            lambda self, value: self.__dict__.__setitem__("_points", list(value)),
        )
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
        self.g2d = openglider.load(DEMOKITE)
        self.obj = _FreeCADObject(self.g2d)
        self.tool = self.module.LineTool(self.obj)

    def _markers(self):
        return [
            obj
            for obj in self.tool.shape.dynamic_objects
            if isinstance(obj, self.module.Upper_Att_Marker)
        ]

    def test_multi_selection_keeps_every_point_force_and_position(self):
        first, second = self._markers()[:2]
        second.force = first.force + 5.0
        second.rib_pos = min(first.rib_pos + 0.2, 1.0)
        expected = [(m.force, m.rib_pos) for m in (first, second)]

        self.tool.shape.selected_objects = [first, second]
        self.tool.selection_changed()

        self.assertEqual([(m.force, m.rib_pos) for m in (first, second)], expected)
        self.assertAlmostEqual(self.tool.up_att_force.value(), first.force, places=3)

    def test_force_spinbox_accepts_newtons(self):
        marker = self._markers()[0]
        marker.force = 250.0
        self.tool.shape.selected_objects = [marker]
        self.tool.selection_changed()
        self.assertAlmostEqual(self.tool.up_att_force.value(), 250.0, places=3)
        self.assertEqual(marker.force, 250.0)

    def test_dialog_parameters_roundtrip(self):
        Dialog = self.dialog_module.LinesForceDistributionDialog
        dialog = Dialog(self.g2d)
        params = dialog.get_parameters()
        self.assertEqual(params["normalize"], "load")
        self.assertEqual(params["total_load"], 100.0)
        self.assertTrue(dialog.total_load.isEnabled())

        self.g2d.force_distribution_config = {
            "spanwise": "chord",
            "chordwise": "uniform",
            "normalize": "max",
            "total_load": 85.0,
        }
        dialog = Dialog(self.g2d)
        params = dialog.get_parameters()
        self.assertEqual(params["spanwise"], "chord")
        self.assertEqual(params["chordwise"], "uniform")
        self.assertEqual(params["normalize"], "max")
        self.assertEqual(params["total_load"], 85.0)
        self.assertFalse(dialog.total_load.isEnabled())

    def test_button_distributes_forces_to_nodes_and_markers(self):
        Dialog = self.dialog_module.LinesForceDistributionDialog
        accepted = self.module.QtGui.QDialog.Accepted
        Dialog.exec_ = lambda self: accepted
        markers = self._markers()
        for marker in markers:
            marker.force = 1.0

        self.tool.open_force_distribution_dialog()

        forces = [marker.force for marker in markers]
        self.assertGreater(max(forces), 10.0)  # newtons at 1 g, 100 kg
        self.assertTrue(all(f > 0 for f in forces))
        self.assertNotEqual(min(forces), max(forces))
        for marker in markers:
            self.assertEqual(marker.force, marker._node.force)
        # the tool works on a copy of the parametric glider
        config = self.tool.parametric_glider.force_distribution_config
        self.assertEqual(config["normalize"], "load")

        # accept() hands the distributed forces and the dialog parameters to
        # the document's parametric glider
        self.tool.accept()
        g2d = self.obj.Proxy.getParametricGlider()
        self.assertEqual(g2d.force_distribution_config["total_load"], 100.0)
        self.assertEqual(
            sorted(n.force for n in g2d.lineset.get_upper_nodes()),
            sorted(marker.force for marker in markers),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
