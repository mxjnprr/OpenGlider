"""Headless test of the FreeCAD Airfoil Control tool (profile list, rib
overrides, distribution).  FreeCAD / pivy are stubbed like in
test_le_panel_split_tool; the 3D spline control points are replaced by a
minimal container since pivy interaction cannot run offscreen."""
import importlib
import os
import sys
import types
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import openglider
from openglider.airfoil import Profile2D

import test_le_panel_split_tool as stubs

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
TOOL_MODULE = "freecad.glider.tools.profile_control_tool"

try:
    from PySide6 import QtCore, QtWidgets

    HAVE_QT = True
except ImportError:  # pragma: no cover
    HAVE_QT = False


class _FakeControlPoints:
    """Stand-in for tools.ControlPointContainer (pivy interaction)."""

    def __init__(self, rm, points=None):
        self.on_drag = []
        self.drag_release = []
        self.control_pos = points or []

    @property
    def control_pos(self):
        return np.array(self._pos)

    @control_pos.setter
    def control_pos(self, pts):
        self._pos = [list(map(float, p)) + ([0.0] if len(p) == 2 else []) for p in pts]
        self.control_points = [types.SimpleNamespace(constrained=[1.0, 1.0, 0.0]) for _ in self._pos]

    def remove_callbacks(self):
        pass


def nose_shelf_level(profile, start=0.03, end=0.08):
    p = Profile2D(profile.data)
    xs = p.data[p.noseindex :, 0]
    ys = p.data[p.noseindex :, 1]
    mask = (xs >= start) & (xs <= end)
    return float(np.mean(ys[mask]))


@unittest.skipUnless(HAVE_QT, "PySide6 not available")
class TestAirfoilControlTool(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._added = stubs._install_stubs()
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        for name in (TOOL_MODULE, "freecad.glider.tools.glider", "freecad.glider.tools.tools"):
            sys.modules.pop(name, None)
        cls.module = importlib.import_module(TOOL_MODULE)
        cls.module.ControlPointContainer = _FakeControlPoints
        cls.plain_dat = os.path.join(ROOT, "tests", "common", "_plain_test_profile.dat")

    @classmethod
    def tearDownClass(cls):
        for name in list(sys.modules):
            if name.startswith("freecad"):
                sys.modules.pop(name, None)
        for name in cls._added:
            sys.modules.pop(name, None)
        if os.path.exists(cls.plain_dat):
            os.remove(cls.plain_dat)

    def setUp(self):
        g2d = openglider.load(DEMOKITE)
        plain = Profile2D(g2d.profiles[0].data)
        plain.name = "plain"
        plain.export_dat(self.plain_dat)
        g2d.sharknose_enabled = True
        shark = Profile2D(g2d._apply_sharknose(plain.copy(), 1.0).data)
        shark.name = "shark"
        g2d.sharknose_enabled = False
        other = plain.copy()
        other.name = "other"
        g2d.profiles = [shark, other]
        self.shark_level = nose_shelf_level(shark)
        self.plain_level = nose_shelf_level(plain)
        self.obj = stubs._FreeCADObject(g2d)
        self.tool = self.module.AirfoilControlTool(self.obj)
        self.n = self.tool.override_table.rowCount()
        QtWidgets.QFileDialog.getOpenFileNames = staticmethod(lambda *a, **k: ([self.plain_dat], ""))

    # helpers -------------------------------------------------------------
    def names(self):
        return [self.tool.airfoil_list.item(i).text() for i in range(self.tool.airfoil_list.count())]

    def combo(self, rib):
        return self.tool.override_table.cellWidget(rib, 2)

    def combo_entries(self, rib=0):
        c = self.combo(rib)
        return [c.itemText(i) for i in range(c.count())]

    def default_text(self, rib):
        return self.tool.override_table.item(rib, 1).text()

    def profiles(self):
        return [p.name for p in self.tool.parametric_glider.profiles]

    def rib_levels(self):
        glider = self.obj.Proxy.g2d.get_glider_3d()
        offset = 1 if self.obj.Proxy.g2d.shape.has_center_cell else 0
        return [round(nose_shelf_level(glider.ribs[i + offset].profile_2d), 5) for i in range(self.n - 1)]

    def simulate_drag_move(self, src_row, dst_row):
        """Qt's InternalMove inserts a decoded copy of the item (data roles
        only) and then removes the source row."""
        item = self.tool.airfoil_list.item(src_row)
        clone = QtWidgets.QListWidgetItem(item.text())
        clone.setData(self.module.AIRFOIL_UID_ROLE, item.data(self.module.AIRFOIL_UID_ROLE))
        clone.setCheckState(item.checkState())
        self.tool.airfoil_list.insertItem(dst_row, clone)
        self.tool.airfoil_list.takeItem(self.tool.airfoil_list.row(item))
        self.tool._on_airfoil_list_reordered()

    # tests ---------------------------------------------------------------
    def test_initial_lists(self):
        self.assertEqual(self.names(), ["shark", "other"])
        self.assertEqual(self.combo_entries(), ["(Use distribution)", "0. shark", "1. other"])
        self.assertEqual(self.default_text(0), "shark")
        self.assertEqual(
            [self.tool.profile_list.item(i).text() for i in range(self.tool.profile_list.count())],
            ["0. shark", "1. other"],
        )

    def test_import_is_visible_immediately_in_overrides_and_distribution(self):
        self.tool._airfoil_import()
        self.assertEqual(self.names(), ["shark", "other", "plain"])
        self.assertEqual(self.profiles(), ["shark", "other", "plain"])
        self.assertEqual(self.combo_entries(), ["(Use distribution)", "0. shark", "1. other", "2. plain"])
        self.assertEqual(self.tool.profile_list.item(2).text(), "2. plain")

    def test_override_with_plain_profile_removes_shark_nose_even_with_sharknose_tab(self):
        self.tool._airfoil_import()
        self.tool.overrides_enabled.setChecked(True)
        for rib in (self.n - 3, self.n - 2):
            self.combo(rib).setCurrentIndex(3)  # "2. plain"
        g2d = self.obj.Proxy.g2d
        self.assertEqual(g2d.profile_overrides, {str(self.n - 3): 2, str(self.n - 2): 2})
        self.assertTrue(g2d.profile_overrides_enabled)
        levels = self.rib_levels()
        self.assertAlmostEqual(levels[self.n - 2], self.plain_level, places=4)
        self.assertAlmostEqual(levels[0], self.shark_level, places=4)
        # procedural shark nose on all cells does not touch the overridden ribs
        self.tool.sharknose_enabled.setChecked(True)
        levels = self.rib_levels()
        self.assertAlmostEqual(levels[self.n - 2], self.plain_level, places=4)
        self.assertGreater(levels[0], self.plain_level + 0.005)

    def test_sharknose_checkbox_unchecked_means_not_applied_at_all(self):
        # start from plain profiles only
        self.tool.airfoil_list.setCurrentRow(1)  # other (plain geometry)
        self.tool._airfoil_apply_as_default()
        self.tool.airfoil_list.setCurrentRow(1)  # shark, now at row 1
        self.tool._airfoil_delete()
        self.assertEqual(self.profiles(), ["other"])
        self.assertFalse(self.tool.sharknose_enabled.isChecked())
        self.assertIn("NOT applied", self.tool.sharknose_preview_label.text())
        # tuning the parameters / cells while unchecked changes nothing on the wing
        self.tool.sharknose_angle_spin.setValue(12.0)
        self.tool._select_all_sharknose_cells()
        self.assertFalse(self.obj.Proxy.g2d.sharknose_enabled)
        for level in self.rib_levels():
            self.assertAlmostEqual(level, self.plain_level, places=4)
        # on -> deformed, off again -> plain everywhere
        self.tool.sharknose_enabled.setChecked(True)
        self.assertTrue(self.obj.Proxy.g2d.sharknose_enabled)
        self.assertIn("applied to the selected cells", self.tool.sharknose_preview_label.text())
        self.assertGreater(self.rib_levels()[0], self.plain_level + 0.005)
        self.tool.sharknose_enabled.setChecked(False)
        self.assertFalse(self.obj.Proxy.g2d.sharknose_enabled)
        for level in self.rib_levels():
            self.assertAlmostEqual(level, self.plain_level, places=4)
        self.tool.accept()
        self.assertFalse(self.obj.Proxy.g2d.sharknose_enabled)

    def test_rename_updates_override_combos(self):
        self.tool.airfoil_list.item(1).setText("renamed")
        self.assertEqual(self.combo_entries()[2], "1. renamed")
        self.assertEqual(self.profiles()[1], "renamed")

    def test_reorder_keeps_override_on_the_same_profile(self):
        self.tool._airfoil_import()
        self.tool.overrides_enabled.setChecked(True)
        self.combo(5).setCurrentIndex(3)  # plain
        self.simulate_drag_move(2, 0)  # plain becomes profile 0
        self.assertEqual(self.names(), ["plain", "shark", "other"])
        self.assertEqual(self.profiles(), ["plain", "shark", "other"])
        self.assertEqual(self.tool.parametric_glider.profile_overrides, {"5": 0})
        self.assertEqual(self.combo(5).currentText(), "0. plain")
        self.assertTrue(self.tool.airfoil_list.item(0).flags() & QtCore.Qt.ItemIsEditable)

    def test_delete_drops_its_override_and_reindexes_the_others(self):
        self.tool._airfoil_import()
        self.tool.overrides_enabled.setChecked(True)
        self.combo(4).setCurrentIndex(2)  # other
        self.combo(5).setCurrentIndex(3)  # plain
        self.tool.airfoil_list.setCurrentRow(1)
        self.tool._airfoil_delete()
        self.assertEqual(self.profiles(), ["shark", "plain"])
        self.assertEqual(self.tool.parametric_glider.profile_overrides, {"5": 1})
        self.assertEqual(self.combo(4).currentIndex(), 0)
        self.assertEqual(self.combo(5).currentText(), "1. plain")

    def test_last_profile_cannot_be_deleted(self):
        self.tool.airfoil_list.setCurrentRow(1)
        self.tool._airfoil_delete()
        self.tool.airfoil_list.setCurrentRow(0)
        self.tool._airfoil_delete()
        self.assertEqual(self.profiles(), ["shark"])

    def test_apply_as_default_keeps_the_other_profiles(self):
        self.tool._airfoil_import()
        self.tool.overrides_enabled.setChecked(True)
        self.combo(5).setCurrentIndex(3)  # plain
        self.tool.airfoil_list.setCurrentRow(1)  # other
        self.tool._airfoil_apply_as_default()
        self.assertEqual(self.profiles(), ["other", "shark", "plain"])
        self.assertEqual(self.tool.parametric_glider.profile_overrides, {"5": 2})
        self.assertEqual(self.default_text(0), "other")
        # the live preview still resolves the override
        self.assertAlmostEqual(self.rib_levels()[5], self.plain_level, places=4)

    def test_default_column_describes_the_morph(self):
        g2d = self.tool.parametric_glider
        span = g2d.profile_merge_curve.controlpoints[-1][0]
        g2d.profile_merge_curve.controlpoints = [[0.0, 0.0], [0.5 * span, 0.5], [span, 1.0]]
        self.tool.dist_controlpoints.control_pos = g2d.profile_merge_curve.controlpoints
        self.tool._on_distribution_release()
        self.assertEqual(self.default_text(self.n - 1), "other")
        self.assertIn("%", self.default_text(self.n // 2))
        self.assertIn("shark", self.default_text(self.n // 2))

    def test_stored_overrides_are_restored_and_accept_writes_back(self):
        self.tool._airfoil_import()
        self.tool.overrides_enabled.setChecked(True)
        self.combo(5).setCurrentIndex(3)
        self.tool.accept()
        g2d = self.obj.Proxy.g2d
        self.assertEqual([p.name for p in g2d.profiles], ["shark", "other", "plain"])
        self.assertEqual(g2d.profile_overrides, {"5": 2})
        tool = self.module.AirfoilControlTool(self.obj)
        self.assertEqual(tool.override_table.cellWidget(5, 2).currentText(), "2. plain")
        self.assertTrue(tool.overrides_enabled.isChecked())


if __name__ == "__main__":
    unittest.main()
