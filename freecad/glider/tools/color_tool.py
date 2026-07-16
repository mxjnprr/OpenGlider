

import FreeCADGui
from pivy.graphics import InteractionSeparator, Polygon
from PySide import QtGui

from .tools import (
    BaseTool,
    hex_to_rgb,
    input_field,
    rgb_to_hex,
)


class ColorPolygon(Polygon):
    std_col = [0.5, 0.5, 0.5]

    def set_enabled(self):
        self.color.diffuseColor = self.std_col
        self.enabled = True

    def set_color(self, col=None):
        self.std_col = col or self.std_col
        self.color.diffuseColor = self.std_col

    def unset_mouse_over(self):
        if self.enabled:
            self.color.diffuseColor = self.std_col

    def unselect(self):
        if self.enabled:
            self.color.diffuseColor = self.std_col


def refresh():
    pass


def _rgb_css(rgb):
    r, g, b = (max(0, min(255, int(round(c * 255)))) for c in rgb)
    return f"rgb({r}, {g}, {b})"


def _swatch_style(rgb):
    return (
        f"background-color: {_rgb_css(rgb)}; border: 1px solid #444;"
    )


def _color_key(rgb):
    """Canonical 8-bit RGB key — robust to float-roundtrip drift between
    QColorDialog (float) and hex_to_rgb (int/255 division)."""
    return tuple(max(0, min(255, int(round(c * 255)))) for c in rgb)


class ReplaceColorDialog(QtGui.QDialog):
    SCOPE_SELECTION = "selection"
    SCOPE_ALL = "all"

    def __init__(self, old_color, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Replace color")
        self._old_color = tuple(old_color)
        self._new_color = tuple(old_color)

        layout = QtGui.QVBoxLayout(self)

        grid = QtGui.QGridLayout()
        grid.addWidget(QtGui.QLabel("Color to replace:"), 0, 0)
        self._old_swatch = QtGui.QLabel()
        self._old_swatch.setFixedSize(60, 22)
        self._old_swatch.setStyleSheet(_swatch_style(self._old_color))
        grid.addWidget(self._old_swatch, 0, 1)

        grid.addWidget(QtGui.QLabel("New color:"), 1, 0)
        self._new_swatch = QtGui.QPushButton("")
        self._new_swatch.setFixedSize(60, 22)
        self._new_swatch.setStyleSheet(_swatch_style(self._new_color))
        self._new_swatch.clicked.connect(self._pick_new_color)
        grid.addWidget(self._new_swatch, 1, 1)
        grid.addWidget(QtGui.QLabel("(click to pick)"), 1, 2)
        layout.addLayout(grid)

        scope_box = QtGui.QGroupBox("Apply to")
        scope_layout = QtGui.QVBoxLayout(scope_box)
        self._scope_selection = QtGui.QRadioButton("Selected panels only")
        self._scope_all = QtGui.QRadioButton("All panels with this color")
        self._scope_all.setChecked(True)
        scope_layout.addWidget(self._scope_selection)
        scope_layout.addWidget(self._scope_all)
        layout.addWidget(scope_box)

        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _pick_new_color(self):
        initial = QtGui.QColor.fromRgbF(*self._new_color)
        picked = QtGui.QColorDialog.getColor(initial, self, "New color")
        if not picked.isValid():
            return
        self._new_color = tuple(picked.getRgbF()[:-1])
        self._new_swatch.setStyleSheet(_swatch_style(self._new_color))

    def new_color(self):
        return list(self._new_color)

    def scope(self):
        if self._scope_selection.isChecked():
            return self.SCOPE_SELECTION
        return self.SCOPE_ALL


class ColorTool(BaseTool):
    widget_name = "Color Tool"

    def __init__(self, obj):
        super().__init__(obj)

        # Build both wings so the tool can paint them independently (asymmetric
        # decoupe).  For a symmetric glider the two lists are identical, so the
        # save logic below keeps it symmetric.
        self.right_panels = self.parametric_glider.get_panels(side="right")
        self.left_panels = self.parametric_glider.get_panels(side="left")
        self.panels = self.right_panels  # kept for backwards-compat references

        # panel.materialcode
        # panel.cut_back
        # panel.cut_front
        # cut_front['left']

        # setup the GUI
        self.setup_widget()
        self.setup_pivy()

    def setup_widget(self):
        self.Qcolore_select = QtGui.QPushButton("select color")
        self.layout.setWidget(0, input_field, self.Qcolore_select)
        self.color_dialog = QtGui.QColorDialog()
        self.Qcolore_select.clicked.connect(self.color_dialog.open)
        self.color_dialog.accepted.connect(self.set_color)

        self.Qcolore_replace = QtGui.QPushButton("replace color...")
        self.layout.setWidget(1, input_field, self.Qcolore_replace)
        self.Qcolore_replace.clicked.connect(self.replace_color)

    def setup_pivy(self):
        # get 2d shape properties

        self.selector = InteractionSeparator(self.rm)
        self.task_separator += [self.selector]
        x_values = self.parametric_glider.shape.rib_x_values
        if self.parametric_glider.shape.has_center_cell:
            x_values = [-x_values[0]] + x_values

        has_center = self.parametric_glider.shape.has_center_cell

        # Right wing on the +x side (index 0 is the centre cell for
        # centre-cell gliders and is only drawn here).
        for i, cell in enumerate(self.right_panels):
            self._add_wing_cell(cell, i, x_values, sign=1)

        # Left wing mirrored on the -x side.  Skip the centre cell (index 0),
        # which is single and already drawn from the right list.
        start = 1 if has_center else 0
        for i in range(start, len(self.left_panels)):
            self._add_wing_cell(self.left_panels[i], i, x_values, sign=-1)

        self.selector.register()

    def _add_wing_cell(self, panel_list, i, x_values, sign):
        """Add the ColorPolygon visuals for one cell's panels, placed on the
        +x (sign=1, right) or -x (sign=-1, left) side of the planform."""
        from openglider.glider.cell.polygon_panel import PolygonPanel

        x0 = x_values[i]
        x1 = x_values[i + 1]
        # Mirroring the left wing (sign=-1) negates x, which flips the polygon
        # winding. Coin then shows the (unlit) back face -> the panels render
        # black. Reversing the vertex order on the mirrored side keeps them
        # front-facing, exactly like the right wing.
        for panel in panel_list:
            if isinstance(panel, PolygonPanel):
                # Crossing-region panel: build its planform polygon from the
                # region boundary (y, chord) -> (x_span, chord).
                pts = [
                    [sign * (x0 + y * (x1 - x0)), chord, 0.0]
                    for (y, chord) in panel.region.boundary
                ]
                if sign < 0:
                    pts = pts[::-1]
                vis_panel = ColorPolygon(pts, True)
            else:
                # Get panel y_start/y_end for split panels
                y_start = getattr(panel, 'y_start', 0.0)
                y_end = getattr(panel, 'y_end', 1.0)

                # Interpolate X positions based on y range
                x_left = sign * (x0 + y_start * (x1 - x0))
                x_right = sign * (x0 + y_end * (x1 - x0))

                # Interpolate cut positions based on y range
                front_left = panel.cut_front["left"] + y_start * (panel.cut_front["right"] - panel.cut_front["left"])
                front_right = panel.cut_front["left"] + y_end * (panel.cut_front["right"] - panel.cut_front["left"])
                back_left = panel.cut_back["left"] + y_start * (panel.cut_back["right"] - panel.cut_back["left"])
                back_right = panel.cut_back["left"] + y_end * (panel.cut_back["right"] - panel.cut_back["left"])

                p1 = [x_left, front_left, 0.0]
                p2 = [x_left, back_left, 0.0]
                p3 = [x_right, back_right, 0.0]
                p4 = [x_right, front_right, 0.0]
                quad = [p1, p2, p3, p4]
                # right wing uses reversed order; mirror flips it back
                quad = quad[::-1] if sign > 0 else quad
                vis_panel = ColorPolygon(quad, True)
            panel.vis_panel = vis_panel
            if panel.material_code:
                vis_panel.set_color(hex_to_rgb(panel.material_code))
            self.selector += [vis_panel]

    def set_color(self):
        color = self.color_dialog.currentColor().getRgbF()[:-1]
        for panel in self.selector.selected_objects:
            panel.set_color(color)

    def replace_color(self):
        selected = list(self.selector.selected_objects)
        if not selected:
            QtGui.QMessageBox.information(
                FreeCADGui.getMainWindow(),
                "Replace color",
                "Select at least one panel in the 3D view first — its "
                "color will be used as the color to replace.",
            )
            return

        old_color = list(selected[0].std_col)
        old_key = _color_key(old_color)
        dialog = ReplaceColorDialog(old_color, FreeCADGui.getMainWindow())
        if dialog.exec_() != QtGui.QDialog.Accepted:
            return

        new_color = dialog.new_color()
        if _color_key(new_color) == old_key:
            return

        if dialog.scope() == ReplaceColorDialog.SCOPE_SELECTION:
            targets = [p for p in selected if _color_key(p.std_col) == old_key]
        else:
            targets = [
                p for p in self.selector.dynamic_objects
                if _color_key(p.std_col) == old_key
            ]

        for panel in targets:
            panel.set_color(new_color)

    def accept(self):
        self.selector.unregister()

        def collect(panel_lists):
            """Read the painted colours back off each panel's visual.  Panels
            that were never drawn (e.g. the left-wing centre cell) have no
            vis_panel and are skipped."""
            by_name = {}
            positional = []
            for cell in panel_lists:
                cell_colors = []
                for panel in cell:
                    vis = getattr(panel, "vis_panel", None)
                    if vis is None:
                        continue
                    color_code = rgb_to_hex(vis.std_col, "skytex32_")
                    panel.material_code = color_code
                    cell_colors.append(color_code)
                    by_name[panel.name] = color_code
                positional.append(cell_colors)
            return by_name, positional

        right_by_name, positional = collect(self.right_panels)
        left_by_name, _ = collect(self.left_panels)

        elements = self.parametric_glider.elements
        # The right wing is the base (positional list + by-name map), matching
        # the legacy symmetric format.
        elements["materials"] = positional
        elements["materials_by_name"] = right_by_name

        # Persist a left override only where it genuinely differs from the right
        # wing, so a symmetric paint job leaves the glider symmetric
        # (is_asymmetric stays False).
        left_overrides = {
            name: col
            for name, col in left_by_name.items()
            if col != right_by_name.get(name)
        }
        if left_overrides:
            elements["materials_left_by_name"] = left_overrides
        else:
            elements.pop("materials_left_by_name", None)
        # Right-only overrides are folded into the base map; keep the dedicated
        # key clear to avoid double bookkeeping.
        elements.pop("materials_right_by_name", None)

        super().accept()
        self.update_view_glider()

    def reject(self):
        self.selector.unregister()
        super().reject()
