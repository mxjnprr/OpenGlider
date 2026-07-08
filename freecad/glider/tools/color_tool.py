

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

        self.panels = self.parametric_glider.get_panels()

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
        for i, cell in enumerate(self.panels):
            for j, panel in enumerate(cell):
                # Get panel y_start/y_end for split panels
                y_start = getattr(panel, 'y_start', 0.0)
                y_end = getattr(panel, 'y_end', 1.0)
                
                # Interpolate X positions based on y range
                x_left = x_values[i] + y_start * (x_values[i + 1] - x_values[i])
                x_right = x_values[i] + y_end * (x_values[i + 1] - x_values[i])
                
                # Interpolate cut positions based on y range
                front_left = panel.cut_front["left"] + y_start * (panel.cut_front["right"] - panel.cut_front["left"])
                front_right = panel.cut_front["left"] + y_end * (panel.cut_front["right"] - panel.cut_front["left"])
                back_left = panel.cut_back["left"] + y_start * (panel.cut_back["right"] - panel.cut_back["left"])
                back_right = panel.cut_back["left"] + y_end * (panel.cut_back["right"] - panel.cut_back["left"])
                
                p1 = [x_left, front_left, 0.0]
                p2 = [x_left, back_left, 0.0]
                p3 = [x_right, back_right, 0.0]
                p4 = [x_right, front_right, 0.0]
                vis_panel = ColorPolygon([p1, p2, p3, p4][::-1], True)
                panel.vis_panel = vis_panel
                if panel.material_code:
                    vis_panel.set_color(hex_to_rgb(panel.material_code))
                self.selector += [vis_panel]

        self.selector.register()

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
        colors = []
        colors_by_name = {}  # New: store by panel name for split panels
        
        for cell in self.panels:
            cell_colors = []
            for panel in cell:
                # Get the color and save it directly to the panel
                color_code = rgb_to_hex(panel.vis_panel.std_col, "skytex32_")
                panel.material_code = color_code
                cell_colors.append(color_code)
                # Also store by panel name for split panels
                colors_by_name[panel.name] = color_code
            colors.append(cell_colors)

        self.parametric_glider.elements["materials"] = colors
        self.parametric_glider.elements["materials_by_name"] = colors_by_name
        super().accept()
        self.update_view_glider()

    def reject(self):
        self.selector.unregister()
        super().reject()
