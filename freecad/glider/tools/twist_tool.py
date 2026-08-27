"""Twist (vrillage) tool.

Extends the AoA tool with the per-rib "kite" diagnostic of
:mod:`openglider.glider.twist` and a twist <-> sweep corrector.  See the
in-tool "How to read this" panel for the meaning of every curve; the physics
is documented in :mod:`openglider.glider.twist.model`.
"""

import numpy as np
from pivy import coin
from PySide import QtCore, QtGui

from openglider.glider.twist import TabulatedPolar, TwistModel

from .span_mapping import AoaTool
from .tools import Line_old

HIDDEN = [[0.0, 0.0, 0.0]]  # a Line needs at least one point

# colours (r, g, b), also used as html in the legend
GREEN = (0.10, 0.60, 0.10)
MAGENTA = (0.70, 0.10, 0.70)
ORANGE = (0.95, 0.55, 0.05)
LIGHT_ORANGE = (1.00, 0.75, 0.40)
CYAN = (0.05, 0.60, 0.75)
LIGHT_CYAN = (0.55, 0.80, 0.90)
DARK_GREY = (0.35, 0.35, 0.35)


def _html(rgb):
    return "#%02x%02x%02x" % tuple(int(c * 255) for c in rgb)


def _swatch(rgb, text):
    return ('<span style="color:{}; font-weight:bold">&#9644;</span> {}'
            .format(_html(rgb), text))


HELP_HTML = """
<p><b>What this tool shows</b></p>
<p>Every rib is treated as a small kite hanging from its own lines.
In flight the air pushes on the rib with one resultant force, applied at the
rib's centre of pressure.  The lines can only hold that force if it points
straight at the <i>apex</i> of the line cone (the riser).  When it does not,
the rib wants to pitch: the distance between the force line and the apex is
the <b>moment arm</b>.</p>
<ul>
<li><b>arm &gt; 0</b>: force passes <i>ahead</i> of the apex &rarr; the rib
wants to pitch <i>nose-up</i>; front (A) lines carry more, rear lines less.</li>
<li><b>arm &lt; 0</b>: force passes <i>behind</i> the apex &rarr; the rib
wants to pitch <i>nose-down</i>; rear lines carry more, A lines go slack
first in a surge.</li>
</ul>
<p>Only the <b>change of the arm along the span</b> is a twist matter.  If all
ribs have the same arm, the wing as a whole is simply trimmed nose-up or
nose-down: fix that with the riser position or the glide number, not with
twist.  That is why the green curve is drawn <i>relative to the centre rib</i>
and the common offset is printed as a number.</p>

<p><b>The two ways to correct it</b></p>
<ul>
<li><b>Twist</b> (change the AoA of the rib): the centre of pressure moves
along the chord and the force tilts a little.  It is a <i>weak</i> lever:
several degrees are often needed, and the rib's lift changes with it.</li>
<li><b>Sweep</b> (slide the rib forward/backward, chord unchanged): moves the
whole rib relative to its apex.  Direct and strong, but it changes the
planform.</li>
</ul>
<p>The slider mixes the two.  The orange curve is the AoA after correction,
the orange planform is the swept shape.  <i>Apply proposal</i> writes both
into the model; OK saves the glider.</p>

<p><b>Assumptions</b>: section forces from a thin-airfoil polar of each rib's
profile (camber only) unless XFoil polars are loaded; the wind direction
comes from the glide number; lines are straight from the attachment points to
the riser; ribs without lines borrow the cone of the nearest suspended rib.</p>
"""


class TwistTool(AoaTool):
    widget_name = "Twist"

    def __init__(self, obj):
        self.model = None
        self.proposal = None
        self._arm_scale = 10.0  # mm of arm per degree of the AoA grid
        self._polar_factory = None
        super().__init__(obj)

    # ------------------------------------------------------------------ #
    # widget                                                             #
    # ------------------------------------------------------------------ #
    def setup_widget(self):
        super().setup_widget()  # rows 0-1: num_points / spline type; row 3: glide
        self.QGlide.setToolTip(
            "Glide number used for the wind direction (flight path angle = "
            "atan(1 / glide)).\nIt sets the direction of the air force on every rib."
        )
        row = 4
        span = QtGui.QFormLayout.SpanningRole

        # -- 1. legend ---------------------------------------------------- #
        legend = QtGui.QGroupBox("Curves in the graph", self.base_widget)
        lay = QtGui.QVBoxLayout(legend)
        lines = [
            _swatch((1, 0, 0), "<b>AoA you design</b> (drag the black points) - degrees"),
            _swatch((0, 0, 1), "<b>absolute AoA</b>: chord vs horizontal, in the rib plane - degrees"),
            _swatch(GREEN, "<b>moment arm, relative to the centre rib</b> - mm "
                    "(scale below). 0 = same pitch balance as the centre. "
                    "Up = nose-up tendency, down = nose-down."),
            _swatch(ORANGE, "<b>AoA after correction</b> and, in the planform, the "
                    "<b>swept ribs</b> (light orange)"),
            _swatch(CYAN, "<b>section load</b> Cl&middot;chord (1 = elliptic at the centre); "
                    "light cyan: the elliptic reference"),
            _swatch(MAGENTA, "<b>cone lean</b>: angle of the line cone out of the rib plane - "
                    "degrees. Not corrected here (arc / riser spacing); a fabric-tension hint"),
            _swatch(DARK_GREY, "<b>hinge rib</b> (vertical line): arc angle 45&deg;. Inboard the "
                    "twist is an aerodynamic choice, outboard it mostly sets tip-line tension"),
        ]
        for text in lines:
            lbl = QtGui.QLabel(text)
            lbl.setWordWrap(True)
            lbl.setTextFormat(QtCore.Qt.RichText)
            lay.addWidget(lbl)
        self.layout.setWidget(row, span, legend)
        row += 1

        # -- 2. diagnosis ------------------------------------------------ #
        diag = QtGui.QGroupBox("Diagnosis", self.base_widget)
        dlay = QtGui.QVBoxLayout(diag)
        self.Qinfo = QtGui.QLabel("")
        self.Qinfo.setWordWrap(True)
        self.Qinfo.setTextFormat(QtCore.Qt.RichText)
        dlay.addWidget(self.Qinfo)
        self.Qtable = QtGui.QTableWidget()
        self.Qtable.setColumnCount(9)
        self.Qtable.setHorizontalHeaderLabels(
            ["rib", "AoA °", "arm mm", "rel. mm", "arm % chord", "load", "lean °", "lines", "note"]
        )
        self.Qtable.horizontalHeader().setToolTip(
            "arm mm: moment arm of the rib (+ nose-up)\n"
            "rel. mm: arm minus the centre rib's arm (what twist/sweep can change)\n"
            "arm % chord: arm divided by the rib chord\n"
            "load: Cl x chord, 1 = elliptic reference at the centre\n"
            "lean °: line cone out of the rib plane, + = towards the centre\n"
            "lines: attachment points found on this rib, or the rib whose cone is borrowed"
        )
        self.Qtable.verticalHeader().setVisible(False)
        self.Qtable.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.Qtable.setMinimumHeight(200)
        dlay.addWidget(self.Qtable)
        self.layout.setWidget(row, span, diag)
        row += 1

        # -- 3. correction ----------------------------------------------- #
        corr = QtGui.QGroupBox("Correction", self.base_widget)
        form = QtGui.QFormLayout(corr)

        self.Qmix = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.Qmix.setRange(0, 100)
        self.Qmix.setValue(100)
        self.Qmix.setTickInterval(25)
        self.Qmix.setTickPosition(QtGui.QSlider.TicksBelow)
        self.Qmix.setToolTip(
            "How the spanwise variation of the arm is removed:\n"
            "left (0 %): only by sweeping the ribs - planform changes, AoA unchanged\n"
            "right (100 %): only by twisting the ribs - AoA changes, planform unchanged\n"
            "in between: the twist share is applied first, sweep closes the rest"
        )
        self.Qmix_label = QtGui.QLabel()
        mix_widget = QtGui.QWidget()
        mix_layout = QtGui.QHBoxLayout(mix_widget)
        mix_layout.setContentsMargins(0, 0, 0, 0)
        mix_layout.addWidget(QtGui.QLabel("sweep"))
        mix_layout.addWidget(self.Qmix)
        mix_layout.addWidget(QtGui.QLabel("twist"))
        form.addRow("correct by", mix_widget)
        form.addRow("", self.Qmix_label)

        self.Qtarget = QtGui.QComboBox()
        self.Qtarget.addItem("make every rib balance like the centre rib", "center")
        self.Qtarget.addItem("make every rib moment-free (arm = 0)", "zero")
        self.Qtarget.setToolTip(
            "'like the centre rib' (recommended): remove only the spanwise variation "
            "of the arm - the twist question.\n"
            "'moment-free': also remove the common offset. Twist is weak for that and "
            "the planform cannot shift as a whole (the centre rib is pinned at x = 0): "
            "the tool then tells you how far to move the risers."
        )
        form.addRow("goal", self.Qtarget)

        self.Qsmooth = QtGui.QSpinBox()
        self.Qsmooth.setRange(2, 9)
        self.Qsmooth.setValue(4)
        self.Qsmooth.setToolTip(
            "Number of control points of the sweep curve along the span.\n"
            "Fewer = smoother leading edge but a larger leftover arm; "
            "more = follows every rib."
        )
        form.addRow("sweep smoothing", self.Qsmooth)

        self.Qproposal = QtGui.QLabel("")
        self.Qproposal.setWordWrap(True)
        self.Qproposal.setTextFormat(QtCore.Qt.RichText)
        form.addRow(self.Qproposal)

        self.Qapply = QtGui.QPushButton("Apply proposal")
        self.Qapply.setToolTip(
            "Write the orange AoA curve into the AoA spline (same number of control "
            "points) and the orange planform into the shape's front/back curves.\n"
            "You can still edit afterwards; nothing is saved until OK."
        )
        form.addRow(self.Qapply)
        self.layout.setWidget(row, span, corr)
        row += 1

        # -- 4. display / options ---------------------------------------- #
        opts = QtGui.QGroupBox("Display and physics options", self.base_widget)
        oform = QtGui.QFormLayout(opts)

        self.Qarm_scale = QtGui.QDoubleSpinBox()
        self.Qarm_scale.setRange(0.5, 500.0)
        self.Qarm_scale.setValue(self._arm_scale)
        self.Qarm_scale.setSuffix(" mm per grid degree")
        self.Qarm_scale.setToolTip("Vertical scale of the green arm curve on the AoA grid.")
        oform.addRow("arm scale", self.Qarm_scale)

        self.Qshow_load = QtGui.QCheckBox("show section load (cyan)")
        self.Qshow_load.setChecked(True)
        self.Qshow_load.setToolTip(
            "Cl x chord of every rib from the polar, compared with an elliptic "
            "distribution of the same total. Plotted on the grid as 'degrees': "
            "1.0 = 10 grid degrees."
        )
        oform.addRow(self.Qshow_load)

        self.Qshow_span = QtGui.QCheckBox("show cone lean (magenta)")
        self.Qshow_span.setChecked(False)
        oform.addRow(self.Qshow_span)

        self.Qxfoil = QtGui.QPushButton("Load XFoil polars (slow)")
        self.Qxfoil.setToolTip(
            "Replace the thin-airfoil estimate by XFoil polars of each rib's profile "
            "(needs the 'xfoil' binary in PATH; a few seconds per rib). "
            "Changes Cl, Cd and the centre of pressure, hence the arm."
        )
        self.Qpolar_status = QtGui.QLabel("polars: thin-airfoil (camber line)")
        oform.addRow(self.Qxfoil, self.Qpolar_status)
        self.layout.setWidget(row, span, opts)
        row += 1

        # -- 5. help ------------------------------------------------------ #
        helpbox = QtGui.QGroupBox("How to read this", self.base_widget)
        helpbox.setCheckable(True)
        helpbox.setChecked(False)
        hlay = QtGui.QVBoxLayout(helpbox)
        self.Qhelp = QtGui.QLabel(HELP_HTML)
        self.Qhelp.setWordWrap(True)
        self.Qhelp.setTextFormat(QtCore.Qt.RichText)
        self.Qhelp.setVisible(False)
        hlay.addWidget(self.Qhelp)
        helpbox.toggled.connect(self.Qhelp.setVisible)
        self.layout.setWidget(row, span, helpbox)

        self._on_mix_moved(self.Qmix.value(), solve=False)
        self.Qmix.valueChanged.connect(self._on_mix_moved)
        self.Qmix.sliderReleased.connect(self.update_proposal)
        self.Qtarget.currentIndexChanged.connect(self.update_proposal)
        self.Qsmooth.valueChanged.connect(self.update_proposal)
        self.Qarm_scale.valueChanged.connect(self._on_scale)
        self.Qshow_load.toggled.connect(self.update_curves)
        self.Qshow_span.toggled.connect(self.update_curves)
        self.Qapply.clicked.connect(self.apply_proposal)
        self.Qxfoil.clicked.connect(self.load_xfoil_polars)

    # ------------------------------------------------------------------ #
    # scene                                                              #
    # ------------------------------------------------------------------ #
    def setup_pivy(self):
        self.arm_curve = Line_old([], color=GREEN, width=2)
        self.span_curve = Line_old([], color=MAGENTA, width=1)
        self.load_curve = Line_old([], color=CYAN, width=2)
        self.load_ref_curve = Line_old([], color=LIGHT_CYAN, width=1)
        self.proposed_aoa_curve = Line_old([], color=ORANGE, width=2)
        self.hinge_line = Line_old([], color=DARK_GREY, width=1)
        self.ghost = coin.SoSeparator()
        for curve in (self.arm_curve, self.span_curve, self.load_curve,
                      self.load_ref_curve, self.proposed_aoa_curve, self.hinge_line):
            self.task_separator.addChild(curve.object)
        self.task_separator.addChild(self.ghost)
        super().setup_pivy()  # draws shape, red/blue curves, calls update_glide
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # model                                                              #
    # ------------------------------------------------------------------ #
    def rebuild_model(self):
        try:
            self.model = TwistModel(self.parametric_glider, polar_factory=self._polar_factory)
        except Exception as e:  # keep the AoA tool usable even if lines are odd
            self.model = None
            self.Qinfo.setText("<b>twist model unavailable:</b> {}".format(e))

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

    def load_xfoil_polars(self):
        """Swap the thin-airfoil polars for XFoil ones (blocking)."""
        import shutil

        if shutil.which("xfoil") is None:
            self.Qpolar_status.setText("polars: thin-airfoil - 'xfoil' binary not found in PATH")
            return
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            cache = {}

            def factory(profile):
                key = id(profile)
                if key not in cache:
                    cache[key] = TabulatedPolar.from_xfoil(
                        profile, alpha_deg=list(range(-6, 19, 2)))
                return cache[key]

            self._polar_factory = factory
            self.rebuild_model()
            self.Qpolar_status.setText("polars: XFoil, Re 2.0e6")
        except Exception as e:
            self._polar_factory = None
            self.rebuild_model()
            self.Qpolar_status.setText("polars: thin-airfoil - XFoil failed: {}".format(e))
        finally:
            QtGui.QApplication.restoreOverrideCursor()
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # proposal                                                           #
    # ------------------------------------------------------------------ #
    @property
    def mix(self):
        return self.Qmix.value() / 100.0

    def _on_mix_moved(self, value, solve=True):
        if value >= 100:
            txt = "100 % twist: only the AoA changes"
        elif value <= 0:
            txt = "100 % sweep: only the planform changes"
        else:
            txt = "{} % of the correction by twist, the rest by sweep".format(value)
        self.Qmix_label.setText(txt)
        if solve and not self.Qmix.isSliderDown():
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
            self.Qproposal.setText("<b>solve failed:</b> {}".format(e))
        self.update_curves()

    # ------------------------------------------------------------------ #
    # drawing                                                            #
    # ------------------------------------------------------------------ #
    def _y(self, deg_equivalent):
        """Grid y for a value expressed in 'degrees of the AoA grid'."""
        return np.radians(deg_equivalent) * self.scale[1]

    def update_curves(self, *args):
        curves = (self.arm_curve, self.span_curve, self.load_curve,
                  self.load_ref_curve, self.proposed_aoa_curve, self.hinge_line)
        if self.model is None:
            for curve in curves:
                curve.update(HIDDEN)
            self.ghost.removeAllChildren()
            return

        states = self.model.diagnose()
        ref = states[0].moment_arm
        xs = [s.x * self.scale[0] for s in self.model.stations]

        self.arm_curve.update([
            [x, self._y((st.moment_arm - ref) * 1000.0 / self._arm_scale)]
            for x, st in zip(xs, states)
        ])

        if self.Qshow_span.isChecked():
            self.span_curve.update([[x, self._y(st.spanwise_deg)] for x, st in zip(xs, states)])
        else:
            self.span_curve.update(HIDDEN)

        load, load_ref = self.model.lift_distribution(states)
        if self.Qshow_load.isChecked():
            self.load_curve.update([[x, self._y(10.0 * v)] for x, v in zip(xs, load)])
            self.load_ref_curve.update([[x, self._y(10.0 * v)] for x, v in zip(xs, load_ref)])
        else:
            self.load_curve.update(HIDDEN)
            self.load_ref_curve.update(HIDDEN)

        hinge = self.model.hinge_station()
        if hinge is not None:
            top = max(self._y(np.degrees(st.aoa_rel)) for st in states) * 1.2
            self.hinge_line.update([[hinge * self.scale[0], 0.0], [hinge * self.scale[0], top]])
        else:
            self.hinge_line.update(HIDDEN)

        self.ghost.removeAllChildren()
        self.proposed_aoa_curve.update(HIDDEN)
        if self.proposal is not None:
            pr = self.proposal
            if pr.aoa_curve is not None:
                self.proposed_aoa_curve.update(
                    pr.aoa_curve.get_sequence(num=self.num_on_drag) * self.scale
                )
            if pr.front_curve is not None:
                num = self.num_on_drag
                front = [p for p in pr.front_curve.get_sequence(num=num) if p[0] >= 0]
                back = [p for p in pr.back_curve.get_sequence(num=num) if p[0] >= 0]
                self.ghost += [Line_old(front, color=ORANGE, width=2).object]
                self.ghost += [Line_old(back, color=ORANGE, width=2).object]
                for rib in pr.ribs_2d():
                    self.ghost += [Line_old(rib, color=LIGHT_ORANGE).object]

        self._fill_table(states, load)
        self._fill_info(states, ref, hinge)
        self._fill_proposal()

    def _fill_info(self, states, ref, hinge):
        dev = [(st.moment_arm - ref) * 1000.0 for st in states]
        i_min, i_max = int(np.argmin(dev)), int(np.argmax(dev))
        if ref > 0.005:
            trim = "the whole wing tends <b>nose-up</b> (force ahead of the risers)"
        elif ref < -0.005:
            trim = "the whole wing tends <b>nose-down</b> (force behind the risers)"
        else:
            trim = "the wing is balanced about the risers"
        txt = [
            "<b>Common offset</b> (centre rib arm): {:+.0f} mm - {}. "
            "This part is pitch trim (riser position / glide number), not twist.".format(
                ref * 1000.0, trim),
            "<b>Spanwise variation</b> (what twist or sweep can fix): "
            "{:+.0f} mm at rib {} to {:+.0f} mm at rib {}. "
            "Ribs above the centre value pitch nose-up relative to it, ribs below nose-down."
            .format(dev[i_min], i_min, dev[i_max], i_max),
        ]
        if hinge is not None:
            txt.append("<b>Hinge rib</b> at y = {:.2f} m (arc angle 45°): outboard of it the "
                       "twist mainly sets how hard the tip lines are pulled.".format(hinge))
        self.Qinfo.setText("<br>".join(txt))

    def _fill_proposal(self):
        if self.proposal is None:
            self.Qproposal.setText("")
            return
        pr = self.proposal
        sol = pr.solution
        after = [(a.moment_arm - sol.target) * 1000.0 for a in pr.after]
        parts = []
        if np.any(np.abs(sol.d_aoa) > 1e-9):
            parts.append("twist from {:+.1f}° to {:+.1f}° (orange AoA curve)".format(
                np.degrees(sol.d_aoa.min()), np.degrees(sol.d_aoa.max())))
        if np.any(np.abs(pr.dx_values) > 1e-4):
            parts.append("sweep from {:+.0f} mm to {:+.0f} mm, + = towards the trailing edge "
                         "(orange planform)".format(pr.dx_values.min() * 1000.0,
                                                    pr.dx_values.max() * 1000.0))
        if not parts:
            parts.append("nothing to change")
        txt = ["<b>Proposal:</b> " + "; ".join(parts) + ".",
               "Leftover arm after smoothing: {:.0f} mm.".format(max(abs(a) for a in after))]
        if abs(pr.riser_dx) > 1e-4:
            txt.append("A common sweep of {:+.0f} mm cannot go into the planform (the centre "
                       "rib stays at x = 0): <b>move the risers by {:+.0f} mm</b> instead."
                       .format(pr.riser_dx * 1000.0, -pr.riser_dx * 1000.0))
        if not np.all(sol.reachable):
            bad = ", ".join(str(s.index) for s, ok in zip(sol.stations, sol.reachable) if not ok)
            txt.append("Twist alone cannot reach the goal on ribs {} (the polar runs out of "
                       "range): sweep was added there.".format(bad))
        self.Qproposal.setText("<br>".join(txt))

    def _fill_table(self, states, load):
        table = self.Qtable
        stations = self.model.stations
        ref = states[0].moment_arm
        table.setRowCount(len(stations))
        for i, (s, st) in enumerate(zip(stations, states)):
            lines = ",".join(n for n, p in s.line_attachments) or "-"
            note = ""
            if s.apex_from is not None:
                lines = "none"
                note = "cone of rib {}".format(s.apex_from)
            rel = (st.moment_arm - ref) * 1000.0
            if i > 0 and abs(rel) > 5:
                note = (note + "; " if note else "") + ("nose-up" if rel > 0 else "nose-down")
            values = [
                str(s.index),
                "{:.2f}".format(np.degrees(st.aoa_rel)),
                "{:+.0f}".format(st.moment_arm * 1000.0),
                "{:+.0f}".format(rel),
                "{:+.1f}".format(100.0 * st.moment_arm / s.chord if s.chord else 0.0),
                "{:.2f}".format(load[i]),
                "{:+.1f}".format(st.spanwise_deg),
                lines,
                note,
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
        # the aoa spline was rewritten in place: rebind the control points
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
