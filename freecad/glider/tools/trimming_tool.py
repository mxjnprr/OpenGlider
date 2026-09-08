"""
Trimming tool.

Transcribe suspension-line length changes measured on a prototype back onto the
parametric model.  The designer edits the length of any cascade line -- either in
the table or by clicking the line in the 3d view -- and sees the resulting new
arch (voute) proposal versus the baseline, per attachment-point row (A, B, C ...
up to the brakes), plus the implied per-profile deformation.  On accept, the
proposed (smoothed) arch + AoA curves and the baked profile deformations are
written into the glider.

The full sail is hidden so the row arches and profiles stay visible; only the
suspension lines, the suspended rib profiles and one arch per attachment-point
row are drawn.

Backed by the FreeCAD-independent solver in ``openglider.glider.trim``.
"""

import FreeCAD
import numpy as np
from pivy import coin
from pivy.graphics import InteractionSeparator, Line
from PySide import QtCore, QtGui

from openglider.airfoil import Profile2D
from openglider.glider.trim import TrimModel
from openglider.glider.trim.solver import rib_local_frame

from .tools import BaseTool, input_field, text_field


NUMBER_ROLE = QtCore.Qt.UserRole

# valid pivy.graphics colour names (see pivy/graphics/colors.py):
# black white grey red blue green yellow
COL_NEUTRAL = "white"
COL_TRIMMED = "blue"
COL_PROFILE_BASE = "grey"
COL_PROFILE_PROP = "green"
# per-layer arch colours (A, B, C ... cycled)
LAYER_COLORS = ["red", "green", "blue", "yellow", "grey", "white"]


class TrimmingTool(BaseTool):
    hide = True   # hide the full sail; we draw profiles + row arches ourselves
    turn = False  # keep the current 3d view so lines stay pickable from any angle
    widget_name = "TrimmingTool"

    def __init__(self, obj):
        super().__init__(obj)

        # inverse solver working on a copy of the parametric glider
        self.trim = TrimModel(self.parametric_glider)
        self.layers = self.trim.infer_layers()
        self.layer_order = sorted(set(self.layers.values()))
        self.deltas = {}          # line.number -> length change [m]
        self.proposal = None
        self._syncing = False     # guard against selection feedback loops

        self.obj_by_number = {}   # line.number -> pivy Line
        self.item_by_number = {}  # line.number -> QTreeWidgetItem
        self.spin_by_number = {}  # line.number -> QDoubleSpinBox
        self.comp_lines = set()   # line.numbers currently filled by compensation
        self._last_result = None  # last TrimResult (for baking deformed profiles)

        # The displayed glider lives under a rotation (ViewProvider.rot); mirror
        # it so our overlay coincides with the (now hidden) rendered geometry.
        self.overlay_root = coin.SoSeparator()
        overlay_rot = coin.SoRotation()
        try:
            overlay_rot.rotation.setValue(self.obj.ViewObject.Proxy.rot.rotation.getValue())
        except Exception:
            sbrot = coin.SbRotation()
            sbrot.setValue(coin.SbVec3f(0, 1, 0), coin.SbVec3f(-1, 0, 0))
            overlay_rot.rotation.setValue(sbrot)

        self.line_sep = InteractionSeparator(self.rm)
        self.line_sep.selection_changed = self.on_3d_selection
        self.profile_base_sep = coin.SoSeparator()
        self.profile_prop_sep = coin.SoSeparator()
        self.arch_base_sep = coin.SoSeparator()
        self.arch_prop_sep = coin.SoSeparator()
        self.overlay_root += (
            overlay_rot,
            self.profile_base_sep,
            self.profile_prop_sep,
            self.arch_base_sep,
            self.arch_prop_sep,
            self.line_sep,
        )
        self.task_separator += self.overlay_root

        # widgets
        self.tree = QtGui.QTreeWidget()
        self.Qarc_smooth = QtGui.QSpinBox(self.base_widget)
        self.Qaoa_smooth = QtGui.QSpinBox(self.base_widget)
        self.Qmode = QtGui.QComboBox(self.base_widget)
        self.Qmode.addItems([
            "Deform profile",
            "Keep rigid (compensate other rows)",
        ])
        self.Qiterative = QtGui.QCheckBox("physical re-solve (forces)", self.base_widget)
        self.Qbake = QtGui.QCheckBox(
            "also deform profiles (may open/roughen TE)", self.base_widget
        )
        self.Qsolve = QtGui.QPushButton("Compute preview", self.base_widget)
        self.Qreset = QtGui.QPushButton("Reset trims", self.base_widget)
        self.Qexport = QtGui.QPushButton("Export airfoils (.dat)...", self.base_widget)
        self.Qversion = QtGui.QPushButton("Create new version", self.base_widget)
        self.Qinfo = QtGui.QLabel("", self.base_widget)

        self.setup_widget()
        self.setup_pivy()

    # ------------------------------------------------------------------ #
    # widget                                                             #
    # ------------------------------------------------------------------ #
    def setup_widget(self):
        # the line table lives in its own full-width panel so it grows with the
        # dialog instead of being crammed into a form column
        tree_panel = QtGui.QWidget()
        tree_panel.setWindowTitle("Line plan")
        tree_layout = QtGui.QVBoxLayout(tree_panel)
        tree_layout.setContentsMargins(0, 0, 0, 0)

        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["line / layer", "L [mm]", "delta [mm]"])
        self.tree.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.tree.setSizePolicy(
            QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding
        )
        self.tree.setMinimumHeight(360)
        header = self.tree.header()
        header.setStretchLastSection(False)
        set_mode = getattr(header, "setSectionResizeMode", None) or header.setResizeMode
        set_mode(0, QtGui.QHeaderView.Stretch)
        set_mode(1, QtGui.QHeaderView.ResizeToContents)
        set_mode(2, QtGui.QHeaderView.ResizeToContents)
        self.tree.itemSelectionChanged.connect(self.on_tree_selection)
        tree_layout.addWidget(self.tree)
        self.form.append(tree_panel)

        self._populate_tree()

        self.Qarc_smooth.setRange(3, 6)
        self.Qarc_smooth.setValue(3)
        self.Qaoa_smooth.setRange(2, 6)
        self.Qaoa_smooth.setValue(3)
        self.Qbake.setChecked(False)
        self.Qsolve.clicked.connect(self.solve)
        self.Qreset.clicked.connect(self.reset_trims)
        self.Qexport.clicked.connect(self.export_airfoils)
        self.Qversion.clicked.connect(self.create_version)

        row = 0
        self.layout.setWidget(row, text_field, QtGui.QLabel("profile mode"))
        self.layout.setWidget(row, input_field, self.Qmode)
        row += 1
        self.layout.setWidget(row, text_field, QtGui.QLabel("arch smoothing"))
        self.layout.setWidget(row, input_field, self.Qarc_smooth)
        row += 1
        self.layout.setWidget(row, text_field, QtGui.QLabel("AoA smoothing"))
        self.layout.setWidget(row, input_field, self.Qaoa_smooth)
        row += 1
        self.layout.setWidget(row, input_field, self.Qiterative)
        row += 1
        self.layout.setWidget(row, input_field, self.Qbake)
        row += 1
        self.layout.setWidget(row, text_field, self.Qsolve)
        self.layout.setWidget(row, input_field, self.Qreset)
        row += 1
        self.layout.setWidget(row, text_field, self.Qexport)
        self.layout.setWidget(row, input_field, self.Qversion)
        row += 1
        self.layout.setWidget(row, text_field, self.Qinfo)

    def _line_label(self, line):
        names = [
            n.name
            for n in self.trim.lineset.get_upper_influence_nodes(line)
            if getattr(n, "name", None)
        ]
        layers = sorted({self.layers.get(n, "?") for n in names})
        tag = "".join(layers) if layers else "-"
        return f"{tag}: {', '.join(sorted(names)) or line.name}"

    def _populate_tree(self):
        def add(parent, nodes):
            for line, children in nodes:
                item = QtGui.QTreeWidgetItem(parent)
                item.setText(0, self._line_label(line))
                item.setText(1, f"{self.trim.baseline_length.get(id(line), 0.0) * 1000.0:.0f}")
                item.setData(0, NUMBER_ROLE, line.number)
                self.item_by_number[line.number] = item

                spin = QtGui.QDoubleSpinBox()
                spin.setRange(-500.0, 500.0)
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setValue(self.deltas.get(line.number, 0.0) * 1000.0)
                spin.valueChanged.connect(
                    lambda value, num=line.number: self._on_delta(num, value)
                )
                self.spin_by_number[line.number] = spin
                self.tree.setItemWidget(item, 2, spin)
                add(item, children)

        self.tree.clear()
        self.item_by_number.clear()
        self.spin_by_number.clear()
        add(self.tree.invisibleRootItem(), self.trim.lineset.create_tree())
        self.tree.expandAll()

    def _on_delta(self, number, value_mm):
        self.deltas[number] = value_mm / 1000.0
        # a manual edit takes ownership of the line back from the compensation
        if number in self.comp_lines:
            self.comp_lines.discard(number)
            self._highlight_comp(number, False)
        self._color_lines()

    def _highlight_comp(self, number, on):
        spin = self.spin_by_number.get(number)
        if spin is not None:
            spin.setStyleSheet("background-color: #ff8080;" if on else "")

    def _clear_compensation(self):
        for number in list(self.comp_lines):
            self.deltas.pop(number, None)
            spin = self.spin_by_number.get(number)
            if spin is not None:
                spin.blockSignals(True)
                spin.setValue(0.0)
                spin.blockSignals(False)
            self._highlight_comp(number, False)
        self.comp_lines.clear()

    def reset_trims(self):
        self.deltas = {}
        self.comp_lines.clear()
        for number, spin in self.spin_by_number.items():
            spin.blockSignals(True)
            spin.setValue(0.0)
            spin.blockSignals(False)
            self._highlight_comp(number, False)
        self.proposal = None
        self._last_result = None
        self.proposed_sep_clear()
        self.Qinfo.setText("")
        self._color_lines()

    def proposed_sep_clear(self):
        self.arch_prop_sep.removeAllChildren()
        self.profile_prop_sep.removeAllChildren()

    # ------------------------------------------------------------------ #
    # pivy                                                               #
    # ------------------------------------------------------------------ #
    def setup_pivy(self):
        # pickable lines, drawn exactly like the ViewProvider (same instance,
        # sag, get_line_points) so they overlay the (hidden) rendered lines
        display_glider = self.obj.Proxy.getGliderInstance()
        try:
            display_glider.lineset.recalc(calculate_sag=True)
        except Exception:
            pass
        for line in display_glider.lineset.lines:
            pts = [list(map(float, p)) for p in line.get_line_points(numpoints=3)]
            if len(pts) < 2:
                continue
            obj = Line(pts, dynamic=True)
            obj.trim_number = line.number
            self.obj_by_number[line.number] = obj
            self.line_sep += [obj]
        self.line_sep.register()
        self._color_lines()

        # baseline: suspended profiles + one arch per attachment-point row
        self._draw_profiles(self.profile_base_sep, self.trim.glider, COL_PROFILE_BASE, 1)
        self._baseline_positions = {
            ap.name: self.trim.baseline_pos[id(ap)] for ap in self.trim.attachment_points
        }
        self._draw_row_arches(self.arch_base_sep, self._baseline_positions, width=1)

    def _color_lines(self):
        for number, obj in self.obj_by_number.items():
            trimmed = abs(self.deltas.get(number, 0.0)) > 1e-9
            obj.set_color(COL_TRIMMED if trimmed else COL_NEUTRAL)
        self.line_sep.color_selected()

    def _layer_color(self, layer):
        idx = self.layer_order.index(layer) if layer in self.layer_order else 0
        return LAYER_COLORS[idx % len(LAYER_COLORS)]

    @staticmethod
    def _catmull_rom(points, samples=10):
        """Smooth interpolating spline through ``points`` (no overshoot)."""
        pts = [np.asarray(p, dtype=float) for p in points]
        if len(pts) < 3:
            return [p.tolist() for p in pts]
        ext = [pts[0]] + pts + [pts[-1]]
        out = []
        for i in range(1, len(ext) - 2):
            p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
            for s in range(samples):
                t = s / samples
                out.append(
                    (
                        0.5
                        * (
                            2 * p1
                            + (-p0 + p2) * t
                            + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                            + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
                        )
                    ).tolist()
                )
        out.append(pts[-1].tolist())
        return out

    def _draw_row_arches(self, separator, positions_by_name, width):
        """One smooth spanwise voute per attachment-point layer (A, B, C ...)."""
        separator.removeAllChildren()
        by_layer = {}
        for name, pos in positions_by_name.items():
            by_layer.setdefault(self.layers.get(name, "?"), []).append(
                np.asarray(pos, dtype=float)
            )
        for layer, pts in by_layer.items():
            pts.sort(key=lambda p: p[1])  # order across the span (y)
            if len(pts) < 2:
                continue
            line = Line(self._catmull_rom(pts), dynamic=False)
            line.drawstyle.lineWidth = width
            line.set_color(self._layer_color(layer))
            separator += line

    def _draw_profiles(self, separator, glider, color, width):
        separator.removeAllChildren()
        for rib in glider.ribs:
            pts = [list(map(float, p)) for p in rib.profile_3d]
            if len(pts) < 2:
                continue
            line = Line(pts, dynamic=False)
            line.drawstyle.lineWidth = width
            line.set_color(color)
            separator += line

    def _aps_by_rib(self):
        by_rib = {}
        for ap in self.trim.attachment_points:
            rib = getattr(ap, "rib", None)
            if rib is not None:
                by_rib.setdefault(id(rib), []).append(ap)
        return by_rib

    def _deform_anchors(self, rib, result):
        """Chord-fraction deformation field for one rib (metres, rib plane).

        Returns ``(o, e_chord, e_thick, chord, xs, dch, dth)`` or ``None`` if the
        rib is not affected.  The leading edge and untrimmed rows *ahead* of the
        first trim are held; a trimmed row gets its displacement; untrimmed rows
        *behind* the trim are dropped so they follow the aft camber; the free
        trailing edge is extrapolated from the aft trend.
        """
        aps = self._aps_by_rib().get(id(rib))
        if not aps:
            return None
        o, e_chord, e_thick = rib_local_frame(rib)
        chord = float(rib.chord) or 1.0

        disp = {}
        for ap in aps:
            new = result.attachment_positions.get(ap.name)
            if new is None:
                continue
            d = np.asarray(new, dtype=float) - self.trim.baseline_pos[id(ap)]
            disp[float(np.clip(ap.rib_pos, 0.0, 1.0))] = (
                float(d @ e_chord),
                float(d @ e_thick),
                np.linalg.norm(d) > 5e-4,  # ignore sub-0.5mm noise (keeps tips clean)
            )
        trimmed_x = [x for x, v in disp.items() if v[2]]
        if not trimmed_x:
            return None
        first_trim = min(trimmed_x)

        anchors = {0.0: (0.0, 0.0)}
        for x, (dc, dt, is_trim) in disp.items():
            if is_trim:
                anchors[x] = (dc, dt)
            elif x < first_trim:
                anchors[x] = (0.0, 0.0)

        xs_sorted = sorted(anchors)
        if len(xs_sorted) >= 2 and xs_sorted[-1] < 0.999:
            xa, xb = xs_sorted[-2], xs_sorted[-1]
            va, vb = anchors[xa], anchors[xb]
            gap = xb - xa
            if gap > 1e-2:
                # clamp the extrapolation so a near-LE/degenerate anchor pair can't
                # send the trailing edge off to infinity (the "tip profile from
                # nowhere" artefact)
                f = min((1.0 - xb) / gap, 1.5)
                anchors[1.0] = (vb[0] + (vb[0] - va[0]) * f, vb[1] + (vb[1] - va[1]) * f)
            else:
                anchors[1.0] = vb  # anchors too close -> just hold, don't extrapolate

        xs = np.array(sorted(anchors))
        dch = np.array([anchors[x][0] for x in xs])
        dth = np.array([anchors[x][1] for x in xs])
        return o, e_chord, e_thick, chord, xs, dch, dth

    def _draw_proposed_profiles(self, separator, result, color, width):
        """Draw the chord-morphed (deformed) profiles in 3d."""
        separator.removeAllChildren()
        for rib in self.trim.glider.ribs:
            anchors = self._deform_anchors(rib, result)
            if anchors is None:
                continue
            o, e_chord, e_thick, chord, xs, dch, dth = anchors
            pts = []
            for p in rib.profile_3d:
                p = np.asarray(p, dtype=float)
                xf = float(np.clip(((p - o) @ e_chord) / chord, 0.0, 1.0))
                q = p + np.interp(xf, xs, dch) * e_chord + np.interp(xf, xs, dth) * e_thick
                pts.append(q.tolist())
            if len(pts) >= 2:
                line = Line(pts, dynamic=False)
                line.drawstyle.lineWidth = width
                line.set_color(color)
                separator += line

    def _deformed_profile_2d(self, rib, result):
        """The deformed profile as a normalized Profile2D (for baking/export).

        Only the thickness (camber) component is applied; the chord x-coordinate
        is preserved so the airfoil stays normalized (leading edge at 0, trailing
        edge at 1, closed) -- otherwise the TE drifts off x=1 and the TE cut /
        panel closure leave an open trailing edge.
        """
        anchors = self._deform_anchors(rib, result)
        if anchors is None:
            return None
        o, e_chord, e_thick, chord, xs, dch, dth = anchors
        data = []
        for x, y in rib.profile_2d.data:
            xf = float(np.clip(x, 0.0, 1.0))
            data.append([float(x), float(y + np.interp(xf, xs, dth) / chord)])
        return Profile2D(data, name="trim_deform")

    # ------------------------------------------------------------------ #
    # selection sync                                                     #
    # ------------------------------------------------------------------ #
    def on_3d_selection(self):
        if self._syncing:
            return
        self._syncing = True
        try:
            numbers = {
                getattr(o, "trim_number", None) for o in self.line_sep.selected_objects
            }
            self.tree.clearSelection()
            first = None
            for number in numbers:
                item = self.item_by_number.get(number)
                if item is not None:
                    item.setSelected(True)
                    first = first or item
            if first is not None:
                self.tree.scrollToItem(first)
                self.tree.setCurrentItem(first)
        finally:
            self._syncing = False

    def on_tree_selection(self):
        if self._syncing:
            return
        self._syncing = True
        try:
            for o in self.line_sep.selected_objects:
                o.unselect()
            self.line_sep.selected_objects = []
            for item in self.tree.selectedItems():
                obj = self.obj_by_number.get(item.data(0, NUMBER_ROLE))
                if obj is not None:
                    self.line_sep.selected_objects.append(obj)
            self.line_sep.color_selected()
        finally:
            self._syncing = False

    # ------------------------------------------------------------------ #
    # solve                                                              #
    # ------------------------------------------------------------------ #
    def _trimmed_layers(self, deltas):
        """Layers the user actually trimmed (to compensate with the others)."""
        trimmed = set()
        for line in self.trim.lineset.lines:
            if abs(deltas.get(line.number, 0.0)) > 1e-9:
                for node in self.trim.lineset.get_upper_influence_nodes(line):
                    name = getattr(node, "name", None)
                    if name in self.layers:
                        trimmed.add(self.layers[name])
        return trimmed

    def solve(self):
        # always recompute from the user's own trims, dropping stale compensation
        self._clear_compensation()
        deltas = {n: v for n, v in self.deltas.items() if abs(v) > 1e-9}
        if not deltas:
            self.proposal = None
            self._last_result = None
            self.proposed_sep_clear()
            self.Qinfo.setText("no trims set")
            return

        compensate = self.Qmode.currentIndex() == 1
        comp_txt = ""
        if compensate:
            comp_layers = [l for l in self.layer_order if l not in self._trimmed_layers(deltas)]
            comp = self.trim.compensate(deltas, comp_layers) if comp_layers else {}
            per_layer = {}
            for number, value in comp.items():
                # write the required change into the table, highlighted red
                self.deltas[number] = self.deltas.get(number, 0.0) + value
                self.comp_lines.add(number)
                spin = self.spin_by_number.get(number)
                if spin is not None:
                    spin.blockSignals(True)
                    spin.setValue(self.deltas[number] * 1000.0)
                    spin.blockSignals(False)
                self._highlight_comp(number, True)
                line = next((ln for ln in self.trim.lineset.lines if ln.number == number), None)
                if line is not None:
                    for node in self.trim.lineset.get_upper_influence_nodes(line):
                        lyr = self.layers.get(getattr(node, "name", None))
                        if lyr:
                            per_layer.setdefault(lyr, []).append(value)
            summary = ", ".join(
                f"{lyr} {np.mean(v) * 1000:+.0f}mm" for lyr, v in sorted(per_layer.items())
            )
            comp_txt = f"compensate: {summary or 'none'} | "
            deltas = {n: v for n, v in self.deltas.items() if abs(v) > 1e-9}
            self._color_lines()

        arc_np = self.Qarc_smooth.value()
        aoa_np = self.Qaoa_smooth.value()
        result = self.trim.solve(deltas)
        self._last_result = result
        self._compensate_mode = compensate
        if self.Qiterative.isChecked():
            self.proposal, info = self.trim.solve_iterative(
                deltas, arc_numpoints=arc_np, aoa_numpoints=aoa_np
            )
            gap_txt = f" | fit gap {info['best_gap'] * 1000:.0f}mm"
        else:
            self.proposal = self.trim.to_curves(
                result, arc_numpoints=arc_np, aoa_numpoints=aoa_np
            )
            gap_txt = ""

        # proposed smooth voutes, one per attachment-point layer
        self._draw_row_arches(self.arch_prop_sep, result.attachment_positions, width=3)
        # deformed profiles only when profile deformation is enabled (else the
        # version is arch+AoA only, so drawing them would be misleading)
        if self.Qbake.isChecked():
            self._draw_proposed_profiles(self.profile_prop_sep, result, COL_PROFILE_PROP, 1)
        else:
            self.profile_prop_sep.removeAllChildren()

        d_aoa = np.degrees(self.proposal.new_aoa - self.proposal.base_aoa)
        max_deform = 1000.0 * max((r.max_residual for r in result.ribs.values()), default=0.0)
        self.Qinfo.setText(
            f"{comp_txt}AoA {d_aoa.min():+.1f}..{d_aoa.max():+.1f} deg | "
            f"deform max {max_deform:.0f}mm{gap_txt}"
        )

    # ------------------------------------------------------------------ #
    # accept / reject                                                    #
    # ------------------------------------------------------------------ #
    def _bake_deformed_profiles(self, target=None):
        """Bake a profile override for EVERY station (deformed where trimmed,
        baseline otherwise).

        Overriding every station -- rather than only the deformed ones -- keeps
        the spanwise profile progression continuous, so the wing surface stays
        smooth instead of showing a saw-tooth where a deformed rib sits next to
        an interpolated one.
        """
        if self._last_result is None:
            return
        target = target if target is not None else self.parametric_glider
        overrides = dict(getattr(target, "profile_overrides", {}))
        for station, i in self._representative_ribs().items():
            rib = self.trim.glider.ribs[i]
            profile = self._deformed_profile_2d(rib, self._last_result)
            if profile is None:  # untrimmed rib: keep its (baseline) section
                profile = rib.profile_2d.copy()
            target.profiles.append(profile)
            overrides[str(station)] = len(target.profiles) - 1
        target.profile_overrides = overrides
        target.profile_overrides_enabled = True

    def _representative_ribs(self):
        """Rightmost 3d rib for each half-wing station (deduplicated)."""
        rep = {}
        for i, rib in enumerate(self.trim.glider.ribs):
            station = self.trim._rib_station.get(i)
            if station is None:
                continue
            if station not in rep or rib.pos[1] > self.trim.glider.ribs[rep[station]].pos[1]:
                rep[station] = i
        return rep

    def _deformed_profile_for_station(self, station, i):
        """(profile, is_deformed) for a station's representative rib ``i``."""
        rib = self.trim.glider.ribs[i]
        if self._last_result is not None:
            profile = self._deformed_profile_2d(rib, self._last_result)
            if profile is not None:
                return profile, True
        return rib.profile_2d, False

    def export_airfoils(self):
        """Let the user pick which station airfoils to write to .dat files."""
        rep = self._representative_ribs()
        # selection dialog: one checkable row per station, deformed pre-checked
        dialog = QtGui.QDialog(self.base_widget)
        dialog.setWindowTitle("Export airfoils — choose profiles")
        vbox = QtGui.QVBoxLayout(dialog)
        vbox.addWidget(QtGui.QLabel("Select the profiles to export:"))
        listw = QtGui.QListWidget()
        items = {}
        for station in sorted(rep):
            _, deformed = self._deformed_profile_for_station(station, rep[station])
            item = QtGui.QListWidgetItem(
                f"station {station:02d}" + (" — deformed" if deformed else " — baseline")
            )
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked if deformed else QtCore.Qt.Unchecked)
            listw.addItem(item)
            items[station] = item
        vbox.addWidget(listw)
        row = QtGui.QHBoxLayout()
        ball = QtGui.QPushButton("All")
        bnone = QtGui.QPushButton("None")
        ball.clicked.connect(lambda: [it.setCheckState(QtCore.Qt.Checked) for it in items.values()])
        bnone.clicked.connect(lambda: [it.setCheckState(QtCore.Qt.Unchecked) for it in items.values()])
        row.addWidget(ball)
        row.addWidget(bnone)
        vbox.addLayout(row)
        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        vbox.addWidget(buttons)
        if dialog.exec_() != QtGui.QDialog.Accepted:
            return
        chosen = [s for s, it in items.items() if it.checkState() == QtCore.Qt.Checked]
        if not chosen:
            return

        directory = QtGui.QFileDialog.getExistingDirectory(
            self.base_widget, "Export airfoils to folder"
        )
        if not directory:
            return
        import os

        name = getattr(self.obj, "Label", "glider")
        for station in chosen:
            profile, _ = self._deformed_profile_for_station(station, rep[station])
            profile.export_dat(os.path.join(directory, f"{name}_station{station:02d}.dat"))
        QtGui.QMessageBox.information(
            self.base_widget,
            "Export airfoils",
            f"Wrote {len(chosen)} airfoil(s) to:\n{directory}",
        )

    def _apply_result_to(self, target):
        """Apply the current proposal to ``target`` in place.

        By default only the smooth arch + AoA curves are written -- this keeps the
        original profiles (and their trailing-edge treatment) intact and gives a
        smooth wing.  Profile deformation is opt-in (Qbake), since per-rib
        overrides can roughen the surface and disturb the trailing edge.
        """
        if self.proposal is None:
            return
        self.proposal.apply_to(target, bake_profiles=False)  # smooth arch + AoA
        if self.Qbake.isChecked():
            self._bake_deformed_profiles(target)

    def _next_version_label(self, base):
        """`<root>_v<N+1>`, unique in the document."""
        import re

        match = re.match(r"^(.*?)(?:[_ ]v(\d+))?$", base or "Glider")
        root = match.group(1) or "Glider"
        n = int(match.group(2) or 1)
        existing = {o.Label for o in FreeCAD.ActiveDocument.Objects}
        n += 1
        while f"{root}_v{n}" in existing:
            n += 1
        return f"{root}_v{n}"

    def create_version(self):
        """Create a NEW versioned glider object from the current trim, leaving
        the source glider untouched."""
        if self.proposal is None:
            QtGui.QMessageBox.information(
                self.base_widget, "New version", "Run 'Compute preview' first."
            )
            return
        from copy import deepcopy

        from . import CreateGlider

        new_parametric = deepcopy(self.parametric_glider)
        self._apply_result_to(new_parametric)
        label = self._next_version_label(getattr(self.obj, "Label", "Glider"))
        new_obj = CreateGlider.create_glider(parametric_glider=new_parametric)
        new_obj.Label = label
        FreeCAD.ActiveDocument.recompute()
        QtGui.QMessageBox.information(
            self.base_widget,
            "New version",
            f"Created '{label}' from the current trim.\n"
            "The source glider is unchanged.",
        )

    def accept(self):
        # Non-destructive: closing the tool never edits the source glider.
        # Persisting a trim is done explicitly via "Create new version".
        self.line_sep.unregister()
        super().accept()

    def reject(self):
        self.line_sep.unregister()
        super().reject()
