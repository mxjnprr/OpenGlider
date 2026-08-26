"""Twist (vrillage) tool.

Extends the AoA tool with the per-rib "kite" diagnostic of
:mod:`openglider.glider.twist` and a twist <-> sweep corrector:

* **red**     designed AoA (editable control points, as in the AoA tool)
* **blue**    absolute AoA (chord vs horizontal in the rib plane)
* **green**   moment arm -- signed distance between each rib's aerodynamic
              resultant and the apex of its line cone, plotted as deviation
              from the centre rib (mm, scaled); zero = same pitch balance as
              the centre rib.  Positive = resultant ahead of the apex
              (nose-up tendency, front lines overloaded).
* **magenta** lean of the line cone out of the rib plane (deg, + = inboard);
              geometric, a fabric-tension indicator, not corrected here.
* **orange**  proposed AoA after correction (dashed look: thinner line) and
              the ghost planform when the correction includes sweep.

The *mix* slider chooses how the spanwise variation of the arm is removed:
100 % by twisting the ribs (AoA), 0 % by sliding them along the flight axis
(chord kept, planform changes).  *Apply* bakes the proposal into the AoA
spline and/or the shape curves; OK then writes the glider as usual.
"""

import numpy as np
from pivy import coin
from PySide import QtCore, QtGui

from openglider.glider.twist import TwistModel

from .span_mapping import AoaTool
from .tools import Line_old, input_field, text_field

HIDDEN = [[0.0, 0.0, 0.0]]  # a Line needs at least one point
GREEN = (0.1, 0.6, 0.1)
MAGENTA = (0.7, 0.1, 0.7)
ORANGE = (0.95, 0.55, 0.05)
LIGHT_ORANGE = (1.0, 0.75, 0.4)


class TwistTool(AoaTool):
    widget_name = "Twist"

    def __init__(self, obj):
        self.model = None
        self.proposal = None
        self._arm_scale = 10.0  # mm of arm per degree of the AoA grid
        super().__init__(obj)

    # ------------------------------------------------------------------ #
    # widget                                                             #
    # ------------------------------------------------------------------ #
    def setup_widget(self):
        super().setup_widget()  # num_points, spline type, glide number
        row = 4

        self.Qmix = QtGui.QSlider(QtCore.Qt.Horizontal, self.base_widget)
        self.Qmix.setRange(0, 100)
        self.Qmix.setValue(100)
        self.Qmix.setTickInterval(25)
        self.Qmix.setTickPosition(QtGui.QSlider.TicksBelow)
        self.Qmix.setToolTip("0 % = correct by sweep (move ribs), 100 % = correct by twist (AoA)")
        self.Qmix_label = QtGui.QLabel("twist 100 %")
        mix_widget = QtGui.QWidget(self.base_widget)
        mix_layout = QtGui.QHBoxLayout(mix_widget)
        mix_layout.setContentsMargins(0, 0, 0, 0)
        mix_layout.addWidget(self.Qmix)
        mix_layout.addWidget(self.Qmix_label)
        self.layout.setWidget(row, text_field, QtGui.QLabel("sweep <-> twist"))
        self.layout.setWidget(row, input_field, mix_widget)
        row += 1

        self.Qtarget = QtGui.QComboBox(self.base_widget)
        self.Qtarget.addItem("match centre rib (twist only)", "center")
        self.Qtarget.addItem("moment-free ribs (incl. pitch trim)", "zero")
        self.Qtarget.setToolTip(
            "'centre rib': remove only the spanwise variation of the arm.\n"
            "'moment-free': also remove the common offset -- that part is a "
            "pitch-trim matter (riser position, glide number), not twist."
        )
        self.layout.setWidget(row, text_field, QtGui.QLabel("target"))
        self.layout.setWidget(row, input_field, self.Qtarget)
        row += 1

        self.Qsmooth = QtGui.QSpinBox(self.base_widget)
        self.Qsmooth.setRange(2, 9)
        self.Qsmooth.setValue(4)
        self.Qsmooth.setToolTip("control points of the sweep curve dx(y): fewer = smoother planform")
        self.layout.setWidget(row, text_field, QtGui.QLabel("sweep smoothing"))
        self.layout.setWidget(row, input_field, self.Qsmooth)
        row += 1

        self.Qarm_scale = QtGui.QDoubleSpinBox(self.base_widget)
        self.Qarm_scale.setRange(0.5, 500.0)
        self.Qarm_scale.setValue(self._arm_scale)
        self.Qarm_scale.setSuffix(" mm / °")
        self.Qarm_scale.setToolTip("plot scale of the green arm curve: mm of arm per degree of the grid")
        self.layout.setWidget(row, text_field, QtGui.QLabel("arm scale"))
        self.layout.setWidget(row, input_field, self.Qarm_scale)
        row += 1

        self.Qshow_span = QtGui.QCheckBox("show cone lean (magenta, °)", self.base_widget)
        self.Qshow_span.setChecked(False)
        self.layout.setWidget(row, input_field, self.Qshow_span)
        row += 1

        self.Qapply = QtGui.QPushButton("Apply proposal", self.base_widget)
        self.Qapply.setToolTip("bake the orange curves into the AoA spline / shape curves")
        self.layout.setWidget(row, input_field, self.Qapply)
        row += 1

        self.Qinfo = QtGui.QLabel("", self.base_widget)
        self.Qinfo.setWordWrap(True)
        self.layout.setWidget(row, text_field, self.Qinfo)
        self.layout.setItem(row, input_field, QtGui.QSpacerItem(0, 0))
        row += 1

        self.Qtable = QtGui.QTableWidget(self.base_widget)
        self.Qtable.setColumnCount(8)
        self.Qtable.setHorizontalHeaderLabels(
            ["rib", "AoA °", "arm mm", "arm %c", "lean °", "Δ AoA °", "Δ x mm", "lines"]
        )
        self.Qtable.verticalHeader().setVisible(False)
        self.Qtable.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.Qtable.setMinimumHeight(220)
        self.layout.setWidget(row, text_field, self.Qtable)
        self.layout.setItem(row, input_field, QtGui.QSpacerItem(0, 0))

        self.Qmix.valueChanged.connect(self._on_mix_moved)
        self.Qmix.sliderReleased.connect(self.update_proposal)
        self.Qtarget.currentIndexChanged.connect(self.update_proposal)
        self.Qsmooth.valueChanged.connect(self.update_proposal)
        self.Qarm_scale.valueChanged.connect(self._on_scale)
        self.Qshow_span.toggled.connect(self.update_curves)
        self.Qapply.clicked.connect(self.apply_proposal)

    # ------------------------------------------------------------------ #
    # scene                                                              #
    # ------------------------------------------------------------------ #
    def setup_pivy(self):
        self.arm_curve = Line_old([], color=GREEN, width=2)
        self.span_curve = Line_old([], color=MAGENTA, width=1)
        self.proposed_aoa_curve = Line_old([], color=ORANGE, width=1)
        self.ghost = coin.SoSeparator()
        self.task_separator.addChild(self.arm_curve.object)
        self.task_separator.addChild(self.span_curve.object)
        self.task_separator.addChild(self.proposed_aoa_curve.object)
        self.task_separator.addChild(self.ghost)
        super().setup_pivy()  # draws shape, red/blue curves, calls update_glide
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # model                                                              #
    # ------------------------------------------------------------------ #
    def rebuild_model(self):
        try:
            self.model = TwistModel(self.parametric_glider)
        except Exception as e:  # keep the AoA tool usable even if lines are odd
            self.model = None
            self.Qinfo.setText("twist model unavailable: {}".format(e))

    def update_glide(self, *args):
        super().update_glide(*args)  # sets parametric_glider.glide, redraws aoa
        self.rebuild_model()
        if hasattr(self, "Qmix"):
            self.update_proposal()

    def update_aoa(self):
        super().update_aoa()
        if self.model is not None:
            self.model.set_aoa(self.parametric_glider.aoa)
            self.update_curves()

    def on_release(self):
        super().on_release()
        self.update_proposal()

    def update_num(self):
        super().update_num()
        self.update_proposal()

    def update_spline_type(self):
        super().update_spline_type()
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # proposal                                                           #
    # ------------------------------------------------------------------ #
    @property
    def mix(self):
        return self.Qmix.value() / 100.0

    def _on_mix_moved(self, value):
        self.Qmix_label.setText("twist {} %".format(value))
        if not self.Qmix.isSliderDown():
            self.update_proposal()

    def _on_scale(self, value):
        self._arm_scale = float(value)
        self.update_curves()

    def update_proposal(self, *args):
        if self.model is None:
            self.proposal = None
            self.update_curves()
            return
        try:
            self.proposal = self.model.propose(
                mix=self.mix,
                target=self.Qtarget.currentData(),
                dx_numpoints=self.Qsmooth.value(),
            )
        except Exception as e:
            self.proposal = None
            self.Qinfo.setText("solve failed: {}".format(e))
        self.update_curves()

    # ------------------------------------------------------------------ #
    # drawing                                                            #
    # ------------------------------------------------------------------ #
    def _y(self, deg_equivalent):
        """Grid y for a value expressed in 'degrees of the AoA grid'."""
        return np.radians(deg_equivalent) * self.scale[1]

    def update_curves(self, *args):
        if self.model is None:
            for curve in (self.arm_curve, self.span_curve, self.proposed_aoa_curve):
                curve.update(HIDDEN)
            self.ghost.removeAllChildren()
            return

        states = self.model.diagnose()
        ref = states[0].moment_arm
        xs = [s.x * self.scale[0] for s in self.model.stations]

        arm_pts = [
            [x, self._y((st.moment_arm - ref) * 1000.0 / self._arm_scale)]
            for x, st in zip(xs, states)
        ]
        self.arm_curve.update(arm_pts)

        if self.Qshow_span.isChecked():
            self.span_curve.update([[x, self._y(st.spanwise_deg)] for x, st in zip(xs, states)])
        else:
            self.span_curve.update(HIDDEN)

        self.ghost.removeAllChildren()
        if self.proposal is not None:
            pr = self.proposal
            if pr.aoa_curve is not None:
                self.proposed_aoa_curve.update(
                    pr.aoa_curve.get_sequence(num=self.num_on_drag) * self.scale
                )
            else:
                self.proposed_aoa_curve.update(HIDDEN)
            if pr.front_curve is not None:
                num = self.num_on_drag
                front = pr.front_curve.get_sequence(num=num)
                back = pr.back_curve.get_sequence(num=num)
                half = [p for p in front if p[0] >= 0]
                half_b = [p for p in back if p[0] >= 0]
                self.ghost += [Line_old(half, color=ORANGE, width=2).object]
                self.ghost += [Line_old(half_b, color=ORANGE, width=2).object]
                for rib in pr.ribs_2d():
                    self.ghost += [Line_old(rib, color=LIGHT_ORANGE).object]
        else:
            self.proposed_aoa_curve.update(HIDDEN)

        self._fill_table(states)
        self._fill_info(states, ref)

    def _fill_info(self, states, ref):
        dev = [(st.moment_arm - ref) * 1000.0 for st in states]
        txt = [
            "centre rib arm: {:+.0f} mm (pitch trim: riser / glide)".format(ref * 1000.0),
            "spanwise deviation: {:+.0f} .. {:+.0f} mm".format(min(dev), max(dev)),
        ]
        if self.proposal is not None:
            sol = self.proposal.solution
            after = [(a.moment_arm - sol.target) * 1000.0 for a in self.proposal.after]
            txt.append(
                "proposal: Δaoa {:+.1f} .. {:+.1f} °, Δx {:+.0f} .. {:+.0f} mm, "
                "residual after smoothing {:.0f} mm".format(
                    np.degrees(sol.d_aoa.min()), np.degrees(sol.d_aoa.max()),
                    sol.dx.min() * 1000.0, sol.dx.max() * 1000.0,
                    max(abs(a) for a in after),
                )
            )
            if abs(self.proposal.riser_dx) > 1e-4:
                txt.append(
                    "common sweep {:+.0f} mm cannot go into the shape (centre rib "
                    "stays at x=0): move the risers by {:+.0f} mm instead".format(
                        self.proposal.riser_dx * 1000.0, -self.proposal.riser_dx * 1000.0
                    )
                )
            if not np.all(sol.reachable):
                bad = [str(s.index) for s, ok in zip(sol.stations, sol.reachable) if not ok]
                txt.append("twist alone cannot reach the target on ribs " + ", ".join(bad)
                           + " (sweep added)")
        self.Qinfo.setText("\n".join(txt))

    def _fill_table(self, states):
        table = self.Qtable
        stations = self.model.stations
        table.setRowCount(len(stations))
        sol = self.proposal.solution if self.proposal is not None else None
        for i, (s, st) in enumerate(zip(stations, states)):
            d_aoa = np.degrees(sol.d_aoa[i]) if sol is not None else 0.0
            dx = sol.dx[i] * 1000.0 if sol is not None else 0.0
            lines = ",".join(n for n, p in s.line_attachments) or "-"
            if s.apex_from is not None:
                lines = "(rib {})".format(s.apex_from)
            values = [
                str(s.index),
                "{:.2f}".format(np.degrees(st.aoa_rel)),
                "{:+.0f}".format(st.moment_arm * 1000.0),
                "{:+.1f}".format(100.0 * st.moment_arm / s.chord if s.chord else 0.0),
                "{:+.1f}".format(st.spanwise_deg),
                "{:+.2f}".format(d_aoa),
                "{:+.0f}".format(dx),
                lines,
            ]
            for j, v in enumerate(values):
                item = QtGui.QTableWidgetItem(v)
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                table.setItem(i, j, item)
        table.resizeColumnsToContents()

    # ------------------------------------------------------------------ #
    # apply                                                              #
    # ------------------------------------------------------------------ #
    def apply_proposal(self):
        if self.proposal is None:
            return
        self.proposal.apply_to(self.parametric_glider)
        # aoa spline object was replaced: rebind the control points
        self.spline_controlpoints.control_pos = (
            np.array(self.spline.controlpoints) * self.scale
        )
        self.spline_controlpoints.control_points[-1].constrained = [0.0, 1.0, 0.0]
        self.Qnum_aoa.blockSignals(True)
        self.Qnum_aoa.setValue(len(self.spline.controlpoints))
        self.Qnum_aoa.blockSignals(False)
        # shape may have changed: redraw the grey planform
        self.ribs = self.parametric_glider.shape.ribs
        self.front = [rib[0] for rib in self.ribs]
        self.back = [rib[1] for rib in self.ribs]
        self.draw_shape()
        self.rebuild_model()
        self.update_aoa()
        self.update_grid(drag_release=True)
        self.update_proposal()

    def accept(self):
        self.spline_controlpoints.remove_callbacks()
        super(AoaTool, self).accept()
