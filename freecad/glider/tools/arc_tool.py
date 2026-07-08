import os

import FreeCADGui as Gui
from pivy import coin
from pivy.graphics import Line, Point
from PySide import QtCore, QtGui

from openglider.vector.polygon import CirclePart

from .background_image import BackgroundImage
from .tools import (
    BaseTool,
    ControlPointContainer,
    input_field,
    spline_select,
    text_field,
    vector2D,
    vector3D,
)


class ArcTool(BaseTool):
    hide = False
    widget_name = "ArcTool"

    def __init__(self, obj):
        """adds a symmetric spline to the scene"""
        super().__init__(obj)
        sbrot1 = coin.SbRotation()
        sbrot1.setValue(coin.SbVec3f(1, 0, 0), coin.SbVec3f(0, 1, 0))
        sbrot2 = coin.SbRotation()
        sbrot2.setValue(coin.SbVec3f(0, 0, 1), coin.SbVec3f(0, 1, 0))
        self.obj.ViewObject.Proxy.rotate(sbrot1 * sbrot2)

        controlpoints = list(
            map(vector3D, self.parametric_glider.arc.curve.controlpoints)
        )
        self.arc_cpc = ControlPointContainer(self.rm, controlpoints)
        self.Qnum_arc = QtGui.QSpinBox(self.base_widget)
        self.spline_select = spline_select(
            [self.parametric_glider.arc.curve],
            self.update_spline_type,
            self.base_widget,
        )
        self.shape = coin.SoSeparator()
        self.circle = coin.SoSeparator()
        self.task_separator += self.arc_cpc, self.shape, self.circle

        # Background reference image (front-view plane)
        self.bg_image = BackgroundImage(
            self.rm,
            default_width=2 * self._get_half_span(),
            on_change=self.refresh_bg_widgets,
        )
        self.bg_image.add_to(self.task_separator)

        # Background image widgets
        self.Qbg_load = QtGui.QPushButton("Load image...", self.base_widget)
        self.Qbg_clear = QtGui.QPushButton("Clear", self.base_widget)
        self.Qbg_angle = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qbg_scale = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qbg_pos_x = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qbg_pos_y = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qbg_opacity = QtGui.QSlider(QtCore.Qt.Horizontal, self.base_widget)
        self.Qwing_transp = QtGui.QSlider(QtCore.Qt.Horizontal, self.base_widget)

        # Arc properties widgets (read-only info)
        self.Qarc_length = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qprojected_span = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qarc_height = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qflattening = QtGui.QDoubleSpinBox(self.base_widget)

        # Control points table (normalized)
        self.Qpoints_table = QtGui.QTableWidget(self.base_widget)
        self.Qapply_points = QtGui.QPushButton("Apply Points", self.base_widget)

        self.setup_widget()
        self.setup_pivy()
        self.restore_bg_image()

    def setup_widget(self):
        self.Qnum_arc.setMaximum(9)
        self.Qnum_arc.setMinimum(2)
        self.Qnum_arc.setValue(len(self.parametric_glider.arc.curve.controlpoints))
        self.parametric_glider.arc.curve.numpoints = self.Qnum_arc.value()

        # Basic controls
        self.layout.setWidget(0, text_field, QtGui.QLabel("arc num_points"))
        self.layout.setWidget(0, input_field, self.Qnum_arc)
        self.layout.setWidget(1, text_field, QtGui.QLabel("bspline type"))
        self.layout.setWidget(1, input_field, self.spline_select)

        # Separator - Arc Properties
        separator1 = QtGui.QFrame()
        separator1.setFrameShape(QtGui.QFrame.HLine)
        separator1.setFrameShadow(QtGui.QFrame.Sunken)
        self.layout.setWidget(2, text_field, separator1)
        self.layout.setWidget(2, input_field, QtGui.QLabel("Arc Properties"))

        # Arc properties (read-only)
        self._setup_readonly_spinbox(self.Qarc_length, " m", 3)
        self._setup_readonly_spinbox(self.Qprojected_span, " m", 3)
        self._setup_readonly_spinbox(self.Qarc_height, " m", 3)
        self._setup_readonly_spinbox(self.Qflattening, " %", 1)
        self.Qflattening.setMaximum(100.0)

        self.layout.setWidget(3, text_field, QtGui.QLabel("Arc length"))
        self.layout.setWidget(3, input_field, self.Qarc_length)
        self.layout.setWidget(4, text_field, QtGui.QLabel("Projected span"))
        self.layout.setWidget(4, input_field, self.Qprojected_span)
        self.layout.setWidget(5, text_field, QtGui.QLabel("Arc height"))
        self.layout.setWidget(5, input_field, self.Qarc_height)
        self.layout.setWidget(6, text_field, QtGui.QLabel("Flattening"))
        self.layout.setWidget(6, input_field, self.Qflattening)

        # Separator - Control Points
        separator2 = QtGui.QFrame()
        separator2.setFrameShape(QtGui.QFrame.HLine)
        separator2.setFrameShadow(QtGui.QFrame.Sunken)
        self.layout.setWidget(7, text_field, separator2)
        self.layout.setWidget(7, input_field, QtGui.QLabel("Control Points (%)"))

        # Control points table (normalized)
        self.Qpoints_table.setColumnCount(3)
        self.Qpoints_table.setHorizontalHeaderLabels(["#", "Y (%)", "Z (%)"])
        self.Qpoints_table.horizontalHeader().setStretchLastSection(True)
        self.Qpoints_table.setColumnWidth(0, 30)
        self.Qpoints_table.setMaximumHeight(180)
        self.Qpoints_table.setEditTriggers(QtGui.QAbstractItemView.DoubleClicked)

        self.layout.setWidget(8, text_field, self.Qpoints_table)
        self.layout.setWidget(9, input_field, self.Qapply_points)

        # Separator - Background reference image
        separator3 = QtGui.QFrame()
        separator3.setFrameShape(QtGui.QFrame.HLine)
        separator3.setFrameShadow(QtGui.QFrame.Sunken)
        self.layout.setWidget(10, text_field, separator3)
        self.layout.setWidget(10, input_field, QtGui.QLabel("Background Image"))

        bg_buttons = QtGui.QHBoxLayout()
        bg_buttons.addWidget(self.Qbg_load)
        bg_buttons.addWidget(self.Qbg_clear)
        bg_buttons_widget = QtGui.QWidget(self.base_widget)
        bg_buttons_widget.setLayout(bg_buttons)
        self.layout.setWidget(11, input_field, bg_buttons_widget)

        self.Qbg_angle.setRange(-360.0, 360.0)
        self.Qbg_angle.setDecimals(1)
        self.Qbg_angle.setSuffix(" deg")
        self.Qbg_scale.setRange(0.001, 1000.0)
        self.Qbg_scale.setDecimals(3)
        self.Qbg_scale.setSingleStep(0.05)
        for spinbox in (self.Qbg_pos_x, self.Qbg_pos_y):
            spinbox.setRange(-9999.0, 9999.0)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.05)
        self.Qbg_opacity.setRange(0, 100)
        self.Qwing_transp.setRange(0, 100)

        self.layout.setWidget(12, text_field, QtGui.QLabel("Rotation"))
        self.layout.setWidget(12, input_field, self.Qbg_angle)
        self.layout.setWidget(13, text_field, QtGui.QLabel("Scale"))
        self.layout.setWidget(13, input_field, self.Qbg_scale)
        self.layout.setWidget(14, text_field, QtGui.QLabel("Position X"))
        self.layout.setWidget(14, input_field, self.Qbg_pos_x)
        self.layout.setWidget(15, text_field, QtGui.QLabel("Position Y"))
        self.layout.setWidget(15, input_field, self.Qbg_pos_y)
        self.layout.setWidget(16, text_field, QtGui.QLabel("Image opacity"))
        self.layout.setWidget(16, input_field, self.Qbg_opacity)
        self.layout.setWidget(17, text_field, QtGui.QLabel("Wing transparency"))
        self.layout.setWidget(17, input_field, self.Qwing_transp)

        # Connections
        self.Qnum_arc.valueChanged.connect(self.update_num)
        self.Qapply_points.clicked.connect(self.apply_control_points)
        self.Qbg_load.clicked.connect(self.load_bg_image)
        self.Qbg_clear.clicked.connect(self.clear_bg_image)
        self.Qbg_angle.valueChanged.connect(self.update_bg_from_widgets)
        self.Qbg_scale.valueChanged.connect(self.update_bg_from_widgets)
        self.Qbg_pos_x.valueChanged.connect(self.update_bg_from_widgets)
        self.Qbg_pos_y.valueChanged.connect(self.update_bg_from_widgets)
        self.Qbg_opacity.valueChanged.connect(self.update_bg_opacity)
        self.Qwing_transp.valueChanged.connect(self.update_wing_transparency)

    def _setup_readonly_spinbox(self, spinbox, suffix, decimals):
        """Configure a spinbox for read-only display"""
        spinbox.setReadOnly(True)
        spinbox.setButtonSymbols(QtGui.QAbstractSpinBox.NoButtons)
        spinbox.setSuffix(suffix)
        spinbox.setDecimals(decimals)
        spinbox.setMaximum(9999.0)
        spinbox.setMinimum(-9999.0)
        spinbox.setStyleSheet("QDoubleSpinBox { background-color: #f0f0f0; }")

    def setup_pivy(self):
        self.arc_cpc.on_drag.append(self.update_spline)
        self.arc_cpc.on_drag_release.append(self.update_real_arc)
        self.arc_cpc.on_drag_release.append(self.update_arc_properties)

        self.update_spline()
        self.update_real_arc()
        self.update_num()
        self.update_arc_properties()

    def update_spline(self):
        self.shape.removeAllChildren()
        self.parametric_glider.arc.curve.controlpoints = [
            vector2D(i) for i in self.arc_cpc.control_pos
        ]
        l = Line(vector3D(self.parametric_glider.arc.curve.get_sequence(num=30)))
        l.drawstyle.lineWidth = 2
        self.shape += l
        self.draw_circle()

    def draw_circle(self):
        self.circle.removeAllChildren()
        p1, p2, p3 = self.parametric_glider.arc.curve.get_sequence(num=30)[[0, 15, -1]]
        circle = CirclePart(p1, p2, p3)
        self.circle += Line(vector3D(circle.get_sequence()))
        self.circle += Point(vector3D([circle.center]))
        self.circle += Line(vector3D([p2, circle.center, p3]))

    def update_spline_type(self):
        self.arc_cpc.control_pos = self.parametric_glider.arc.curve.controlpoints
        self.update_spline()

    def get_arc_positions(self):
        return self.parametric_glider.arc.get_arc_positions(
            self.parametric_glider.shape.rib_x_values
        )

    def update_real_arc(self):
        l = Line(vector3D(self.get_arc_positions()))
        l.drawstyle.lineWidth = 2
        l.set_color("red")
        self.shape += l

    def update_num(self, *arg):
        self.parametric_glider.arc.curve.numpoints = self.Qnum_arc.value()
        self.arc_cpc.control_pos = self.parametric_glider.arc.curve.controlpoints
        self.update_spline()
        self.update_arc_properties()

    def _get_half_span(self):
        """Get half span from the last control point's Y coordinate"""
        controlpoints = self.parametric_glider.arc.curve.controlpoints
        if len(controlpoints) > 0:
            return abs(controlpoints[-1][0])
        return 1.0  # fallback

    def update_arc_properties(self):
        """Update the read-only arc properties display"""
        x_values = self.parametric_glider.shape.rib_x_values
        arc = self.parametric_glider.arc

        # Block signals
        self.Qarc_length.blockSignals(True)
        self.Qprojected_span.blockSignals(True)
        self.Qarc_height.blockSignals(True)
        self.Qflattening.blockSignals(True)

        # Update values (multiply by 2 for full span)
        self.Qarc_length.setValue(arc.get_arc_length(x_values) * 2)
        self.Qprojected_span.setValue(arc.get_projected_span(x_values) * 2)
        self.Qarc_height.setValue(arc.get_height(x_values))
        self.Qflattening.setValue(arc.get_flattening(x_values) * 100)

        self.Qarc_length.blockSignals(False)
        self.Qprojected_span.blockSignals(False)
        self.Qarc_height.blockSignals(False)
        self.Qflattening.blockSignals(False)

        # Update control points table
        self.update_points_table()

    def update_points_table(self):
        """Update the control points table with normalized values (%)"""
        controlpoints = self.parametric_glider.arc.curve.controlpoints
        half_span = self._get_half_span()

        self.Qpoints_table.blockSignals(True)
        self.Qpoints_table.setRowCount(len(controlpoints))

        for i, pt in enumerate(controlpoints):
            # Point number (read-only)
            num_item = QtGui.QTableWidgetItem(str(i + 1))
            num_item.setFlags(num_item.flags() & ~QtCore.Qt.ItemIsEditable)
            num_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.Qpoints_table.setItem(i, 0, num_item)

            # Y % (normalized)
            y_pct = (pt[0] / half_span) * 100 if half_span != 0 else 0
            y_item = QtGui.QTableWidgetItem(f"{y_pct:.2f}")
            # First point Y is fixed at 0%, last point Y is fixed at 100%
            if i == 0 or i == len(controlpoints) - 1:
                y_item.setFlags(y_item.flags() & ~QtCore.Qt.ItemIsEditable)
                y_item.setBackground(QtGui.QColor(240, 240, 240))
            self.Qpoints_table.setItem(i, 1, y_item)

            # Z % (normalized - negative values are below)
            z_pct = (pt[1] / half_span) * 100 if half_span != 0 else 0
            z_item = QtGui.QTableWidgetItem(f"{z_pct:.2f}")
            self.Qpoints_table.setItem(i, 2, z_item)

        self.Qpoints_table.blockSignals(False)

    def apply_control_points(self):
        """Apply edited control points from the table"""
        import numpy as np

        half_span = self._get_half_span()
        new_controlpoints = []

        for i in range(self.Qpoints_table.rowCount()):
            y_item = self.Qpoints_table.item(i, 1)
            z_item = self.Qpoints_table.item(i, 2)

            if y_item and z_item:
                try:
                    y_pct = float(y_item.text())
                    z_pct = float(z_item.text())

                    # Denormalize: convert % back to absolute values
                    y_abs = (y_pct / 100) * half_span
                    z_abs = (z_pct / 100) * half_span

                    new_controlpoints.append(np.array([y_abs, z_abs]))
                except ValueError:
                    # Keep original point if parsing fails
                    original = self.parametric_glider.arc.curve.controlpoints[i]
                    new_controlpoints.append(np.array(original))

        # Update the curve
        self.parametric_glider.arc.curve.controlpoints = new_controlpoints

        # Update visual controls
        self.arc_cpc.control_pos = self.parametric_glider.arc.curve.controlpoints

        # Refresh display
        self.update_spline()
        self.update_real_arc()
        self.update_arc_properties()

    # ---------------------------------------------------- background image
    def restore_bg_image(self):
        """Load a background image previously saved with the document."""
        try:
            self.bg_image.load_from(self.obj)
        except Exception as e:
            print("could not restore background image:", e)
        self.refresh_bg_widgets()

    def load_bg_image(self):
        filename = QtGui.QFileDialog.getOpenFileName(
            parent=self.base_widget,
            caption="load reference image",
            filter="Images (*.png *.jpg *.jpeg *.bmp *.gif *.tif *.tiff)",
        )
        path = filename[0]
        if path and os.path.isfile(path):
            if not self.bg_image.load(path):
                print("could not load image:", path)

    def clear_bg_image(self):
        self.bg_image.clear()

    def update_bg_from_widgets(self, *args):
        self.bg_image.set_position(self.Qbg_pos_x.value(), self.Qbg_pos_y.value())
        self.bg_image.set_scale(self.Qbg_scale.value())
        self.bg_image.set_angle(self.Qbg_angle.value())

    def update_bg_opacity(self, *args):
        self.bg_image.set_opacity(self.Qbg_opacity.value() / 100.0)

    def update_wing_transparency(self, *args):
        self.obj.ViewObject.Proxy.set_transparency(self.Qwing_transp.value() / 100.0)

    def refresh_bg_widgets(self):
        """Push the background image state into the panel widgets."""
        widgets = (
            self.Qbg_angle,
            self.Qbg_scale,
            self.Qbg_pos_x,
            self.Qbg_pos_y,
            self.Qbg_opacity,
        )
        for w in widgets:
            w.blockSignals(True)
        self.Qbg_angle.setValue(self.bg_image.angle)
        self.Qbg_scale.setValue(self.bg_image.scale)
        self.Qbg_pos_x.setValue(self.bg_image.pos[0])
        self.Qbg_pos_y.setValue(self.bg_image.pos[1])
        self.Qbg_opacity.setValue(int(round(self.bg_image.opacity * 100)))
        for w in widgets:
            w.blockSignals(False)

        has_image = self.bg_image.has_image()
        for w in widgets:
            w.setEnabled(has_image)
        self.Qbg_clear.setEnabled(has_image)

    def accept(self):
        self.arc_cpc.remove_callbacks()
        self.bg_image.remove_callbacks()
        self.bg_image.save_to(self.obj)
        self.obj.ViewObject.Proxy.set_transparency(0.0)
        super().accept()
        self.obj.ViewObject.Proxy.rotate()
        self.update_view_glider()
        Gui.activeDocument().activeView().viewFront()

    def reject(self):
        self.arc_cpc.remove_callbacks()
        self.bg_image.remove_callbacks()
        self.obj.ViewObject.Proxy.set_transparency(0.0)
        self.obj.ViewObject.Proxy.rotate()
        Gui.activeDocument().activeView().viewFront()
        super().reject()
