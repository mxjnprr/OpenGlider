

from pivy.graphics import InteractionSeparator, Line, Marker
from PySide import QtGui

from openglider.glider.cell.elements import Panel

from .design_path import (
    EDGE_SNAP_CHORD_FACTOR,
    BezierPath,
    DesignPath,
    LinePath,
    PolylinePath,
)
from .tools import BaseTool, coin, input_field, text_field, vector3D


def refresh():
    pass


# idea: draw lines between ribs and fill the panels with color
# only straight lines, no curves
# later create some helpers to generate parametric cuts

# TODO

# switch upper lower        x
# add new line              x
# add new point             x
# create line of two nodes  x
# save to dict              x
# set cut type              x
# q set position            x
# delete                    x


class DesignTool(BaseTool):
    widget_name = "Design Tool"

    def __init__(self, obj):
        super().__init__(obj)
        self.side = "upper"
        # Fallback wing for cut lines whose points carry no side (half-wing
        # mode).  In full-span mode each cut's wing is derived from its planform
        # side instead.  See ParametricGlider.is_asymmetric.
        self.wing = "both"
        # Full-span (asymmetric) editing: open in full-span automatically when
        # the glider already carries asymmetric decoupe data.
        self.full_span = self.parametric_glider.is_asymmetric

        # Initialize shape properties (will be set by update_shape_display)
        self.front = None
        self.back = None
        self.ribs = None
        self.x_values = None
        self._rib_meta = None  # per-canvas-rib (rib_nr_arg, side) in full-span
        self._shape_lines = []  # Track shape lines for redraw

        CutLine.cuts_to_lines(self.parametric_glider, full_span=self.full_span)

        self._add_mode = False
        
        # Design paths (Bezier curves for panel design)
        self.design_paths = []  # List of DesignPath objects
        self._selected_path = None  # Currently selected design path
        self._path_counter = 0  # Counter for unique path IDs
        
        # setup the GUI
        self.setup_widget()
        self.setup_pivy()
        
        # Load existing design paths
        self._load_design_paths()
        
        # Build path visuals and register interactions
        self._refresh_path_visuals()

    def setup_widget(self):
        """set up the qt stuff"""
        self.Qtoggle_side = QtGui.QPushButton("show lower side")
        self.layout.setWidget(0, input_field, self.Qtoggle_side)


        self.tool_widget = QtGui.QWidget()
        self.tool_widget.setWindowTitle("object properties")
        self.tool_layout = QtGui.QFormLayout(self.tool_widget)
        self.form.append(self.tool_widget)

        self.Qcut_type = QtGui.QComboBox(self.tool_widget)
        for _, cut_type in Panel.CUT_TYPES():
            self.Qcut_type.addItem(cut_type)
        self.Qcut_type.setEnabled(False)
        self.Qcut_type.currentIndexChanged.connect(self.cut_type_changed)

        self.tool_layout.setWidget(0, text_field, QtGui.QLabel("cut type"))
        self.tool_layout.setWidget(0, input_field, self.Qcut_type)

        self.QPointPos = QtGui.QDoubleSpinBox()
        self.QPointPos.setMinimum(0)
        self.QPointPos.setMaximum(1)
        self.QPointPos.setSingleStep(0.01)
        self.QPointPos.setDecimals(5)
        self.QPointPos.valueChanged.connect(self.point_pos_changed)

        self.tool_layout.setWidget(1, text_field, QtGui.QLabel("point position"))
        self.tool_layout.setWidget(1, input_field, self.QPointPos)

        # event handlers
        self.Qtoggle_side.clicked.connect(self.toggle_side)

        # Design Path section
        self.Qadd_line = QtGui.QPushButton("+ Line")
        self.Qadd_line.setToolTip("Add a straight line path")
        self.Qadd_polyline = QtGui.QPushButton("+ Polyline")
        self.Qadd_polyline.setToolTip("Add a multi-segment straight line path")
        self.Qadd_bezier = QtGui.QPushButton("+ Bezier")
        self.Qadd_bezier.setToolTip("Add a cubic Bezier curve path")
        
        path_buttons = QtGui.QHBoxLayout()
        path_buttons.addWidget(self.Qadd_line)
        path_buttons.addWidget(self.Qadd_polyline)
        path_buttons.addWidget(self.Qadd_bezier)
        path_widget = QtGui.QWidget()
        path_widget.setLayout(path_buttons)
        self.tool_layout.setWidget(2, text_field, QtGui.QLabel("add path"))
        self.tool_layout.setWidget(2, input_field, path_widget)
        self.Qadd_line.clicked.connect(self.add_line_path)
        self.Qadd_polyline.clicked.connect(self.add_polyline_path)
        self.Qadd_bezier.clicked.connect(self.add_bezier_path)
        
        # Add/Remove Point for Polyline
        self.Qadd_point = QtGui.QPushButton("+ Pt")
        self.Qadd_point.setToolTip("Add a point to the end of the selected path (Polyline mode)")
        self.Qremove_point = QtGui.QPushButton("- Pt")
        self.Qremove_point.setToolTip("Remove the last point from the selected path (Polyline mode)")
        point_buttons = QtGui.QHBoxLayout()
        point_buttons.addWidget(self.Qadd_point)
        point_buttons.addWidget(self.Qremove_point)
        point_widget = QtGui.QWidget()
        point_widget.setLayout(point_buttons)
        self.tool_layout.setWidget(3, text_field, QtGui.QLabel("modify path"))
        self.tool_layout.setWidget(3, input_field, point_widget)
        self.Qadd_point.clicked.connect(self.add_point_to_path)
        self.Qremove_point.clicked.connect(self.remove_point_from_path)
        
        self._update_point_buttons_state()

        # Path cut type
        self.Qpath_cut_type = QtGui.QComboBox(self.tool_widget)
        for _, cut_type in Panel.CUT_TYPES():
            self.Qpath_cut_type.addItem(cut_type)
        self.tool_layout.setWidget(4, text_field, QtGui.QLabel("path cut type"))
        self.tool_layout.setWidget(4, input_field, self.Qpath_cut_type)
        self.Qpath_cut_type.currentIndexChanged.connect(self.path_cut_type_changed)

        # Add cuts at percentage - quick tool
        self.Qpercentage = QtGui.QDoubleSpinBox()
        self.Qpercentage.setMinimum(0)
        self.Qpercentage.setMaximum(100)
        self.Qpercentage.setValue(20)
        self.Qpercentage.setSuffix("%")
        self.Qadd_at_percent = QtGui.QPushButton("Add")
        self.Qadd_at_percent.setToolTip("Add cut line at this chord percentage across all ribs")
        percent_layout = QtGui.QHBoxLayout()
        percent_layout.addWidget(self.Qpercentage)
        percent_layout.addWidget(self.Qadd_at_percent)
        percent_widget = QtGui.QWidget()
        percent_widget.setLayout(percent_layout)
        self.tool_layout.setWidget(5, text_field, QtGui.QLabel("add at %"))
        self.tool_layout.setWidget(5, input_field, percent_widget)
        self.Qadd_at_percent.clicked.connect(self.add_cuts_at_percentage)

        # Apply and Delete buttons
        self.Qapply_paths = QtGui.QPushButton("Apply Paths")
        self.Qapply_paths.setToolTip("Project all paths onto panel cuts")
        self.Qdelete_path = QtGui.QPushButton("Delete Path")
        self.Qdelete_path.setToolTip("Delete selected path")
        action_buttons = QtGui.QHBoxLayout()
        action_buttons.addWidget(self.Qapply_paths)
        action_buttons.addWidget(self.Qdelete_path)
        action_widget = QtGui.QWidget()
        action_widget.setLayout(action_buttons)
        self.tool_layout.setWidget(6, text_field, QtGui.QLabel("actions"))
        self.tool_layout.setWidget(6, input_field, action_widget)
        self.Qapply_paths.clicked.connect(self.apply_paths_to_cuts)
        self.Qdelete_path.clicked.connect(self.delete_selected_path)

        # Select Connected and Delete Selected section
        self.Qselect_connected = QtGui.QPushButton("Select Connected")
        self.Qselect_connected.setToolTip("Select all points and lines connected to current selection")
        self.Qdelete_selected = QtGui.QPushButton("Delete Selected")
        self.Qdelete_selected.setToolTip("Delete all selected points and lines")
        selection_buttons = QtGui.QHBoxLayout()
        selection_buttons.addWidget(self.Qselect_connected)
        selection_buttons.addWidget(self.Qdelete_selected)
        selection_widget = QtGui.QWidget()
        selection_widget.setLayout(selection_buttons)
        self.tool_layout.setWidget(7, text_field, QtGui.QLabel("selection"))
        self.tool_layout.setWidget(7, input_field, selection_widget)
        self.Qselect_connected.clicked.connect(self.select_connected)
        self.Qdelete_selected.clicked.connect(self.delete_selected_cuts)

        # Copy a cut onto the other skin (extrados <-> intrados) at the same
        # chord %. Keeps the originals and adds a mirror-surface copy: copies
        # the selected design path (+ its cuts) if a path is selected, else the
        # selected cut lines/points.
        self.Qflip_surface = QtGui.QPushButton("⧉ Copy to other skin (extrados/intrados)")
        self.Qflip_surface.setToolTip(
            "Duplicate the selected design path (or selected cuts) onto the "
            "other skin at the same chord %. Originals are kept; the copy lands "
            "on the other surface (toggle the side view to see it).")
        self.tool_layout.setWidget(9, text_field, QtGui.QLabel("copy to skin"))
        self.tool_layout.setWidget(9, input_field, self.Qflip_surface)
        self.Qflip_surface.clicked.connect(self.copy_selection_to_other_surface)

        # Full-span (asymmetric decoupe) toggle: when checked the canvas shows
        # BOTH half-wings glued at the centre so cut paths can be drawn across
        # the whole span; each cut's wing is derived from its planform side.
        self.Qfull_span = QtGui.QCheckBox("Asymmetric (full span)")
        self.Qfull_span.setToolTip(
            "Show both wings and draw cut paths across the whole span. "
            "Left/right cuts that stay identical are saved as symmetric.")
        self.Qfull_span.setChecked(self.full_span)
        self.tool_layout.setWidget(8, text_field, QtGui.QLabel("decoupe mode"))
        self.tool_layout.setWidget(8, input_field, self.Qfull_span)
        self.Qfull_span.stateChanged.connect(self.toggle_full_span)

    def toggle_full_span(self):
        """Switch between half-wing and full-span editing, rebuilding the
        canvas and reloading the cuts for the new mode."""
        new_state = bool(self.Qfull_span.isChecked())
        if new_state == self.full_span:
            return
        self.full_span = new_state
        self._rebuild_canvas()

    def _rebuild_canvas(self):
        """Tear down the cut/shape visuals and rebuild them for the current
        mode (half-wing or full-span).  Cuts are re-read from the parametric
        glider, so any unsaved edits made before toggling are dropped."""
        # remove current cut interaction separator
        self.event_separator.select_object(None)
        self.event_separator.unregister()
        self.task_separator.removeChild(self.event_separator)

        # remove current shape lines
        for line in getattr(self, "_shape_lines", []):
            try:
                self.shape.removeChild(line)
            except Exception:
                pass
        self._shape_lines = []

        # rebuild planform + reload cuts for the new mode
        self.update_shape_display()
        self.draw_shape()
        CutLine.cuts_to_lines(self.parametric_glider, full_span=self.full_span)

        # repopulate the interaction separator for the visible surface side
        self.event_separator = InteractionSeparator(self.rm)
        self.event_separator.selection_changed = self.selection_changed
        if self.side == "upper":
            self.event_separator += list(CutLine.upper_point_set)
            self.event_separator += CutLine.upper_line_list
        else:
            self.event_separator += list(CutLine.lower_point_set)
            self.event_separator += CutLine.lower_line_list
        self.task_separator += [self.event_separator]
        self.event_separator.register()
        self._refresh_path_visuals()

    # ==================== Surface transfer (extrados <-> intrados) ============

    def copy_selection_to_other_surface(self):
        """Duplicate the selection onto the other skin at the same chord %.

        Originals are kept.  If a design path is selected, the path is cloned to
        the other surface together with its cuts; otherwise the selected cut
        lines (and the lines of selected points) are copied.  The copies land on
        the other surface (negated rib_pos), so they appear when that side is
        shown.
        """
        if self._selected_path is not None:
            self._copy_path_to_other_surface(self._selected_path)
            return
        lines = []
        for el in self.event_separator.selected_objects:
            if isinstance(el, CutLine):
                lines.append(el)
            elif isinstance(el, CutPoint):
                lines.extend(el.lines)
        lines = list(dict.fromkeys(lines))  # de-dup, keep order
        if not lines:
            return
        self._duplicate_lines_to_other_surface(lines, new_path_id=None)

    def _copy_path_to_other_surface(self, path):
        other = "lower" if path.side == "upper" else "upper"
        data = path.to_dict()
        new_id = f"path_{self._path_counter}"
        self._path_counter += 1
        data["id"] = new_id
        data["side"] = other
        new_path = DesignPath.from_dict(data)
        new_path._applied = True  # its cuts are created below
        self.design_paths.append(new_path)

        lines = [
            ln for ln in CutLine.upper_line_list + CutLine.lower_line_list
            if getattr(ln, "_path_id", None) == path.path_id
        ]
        self._duplicate_lines_to_other_surface(lines, new_path_id=new_id)

    def _copy_point_flipped(self, p):
        """A new CutPoint at the same span/chord position but on the other skin
        (negated rib_pos)."""
        has_center = self.parametric_glider.shape.has_center_cell
        return CutPoint(
            p.rib_nr + has_center, -p.rib_pos, self.parametric_glider,
            x_value=p.x_value, min_y=p.min, max_y=p.max,
            signed_rib=getattr(p, "signed_rib", None),
            wing=getattr(p, "wing", "both"),
        )

    def _duplicate_lines_to_other_surface(self, lines, new_path_id):
        for ln in lines:
            np1 = self._copy_point_flipped(ln.point1)
            np2 = self._copy_point_flipped(ln.point2)
            new_line = CutLine(np1, np2, ln.cut_type,
                               wing=getattr(ln, "_wing", "both"))
            if new_path_id is not None:
                new_line._path_id = new_path_id
            elif hasattr(ln, "_path_id"):
                new_line._path_id = ln._path_id
            new_line.replace_points_by_set()
            new_line.update_Line()
            new_line.setup_visuals()
        self._repartition_and_reload()

    def _repartition_and_reload(self):
        """Re-sort every cut line into the upper/lower sets by its (possibly
        just-flipped) surface, then rebuild the interaction separator for the
        currently visible side."""
        all_lines = CutLine.upper_line_list + CutLine.lower_line_list
        CutLine.upper_line_list = []
        CutLine.lower_line_list = []
        CutLine.upper_point_set = set()
        CutLine.lower_point_set = set()
        for ln in all_lines:
            if ln.is_upper:
                CutLine.upper_line_list.append(ln)
                CutLine.upper_point_set.add(ln.point1)
                CutLine.upper_point_set.add(ln.point2)
            else:
                CutLine.lower_line_list.append(ln)
                CutLine.lower_point_set.add(ln.point1)
                CutLine.lower_point_set.add(ln.point2)

        # rebuild the event separator for the current side (no side toggle)
        self.event_separator.select_object(None)
        self.event_separator.unregister()
        self.task_separator.removeChild(self.event_separator)
        self.event_separator = InteractionSeparator(self.rm)
        self.event_separator.selection_changed = self.selection_changed
        if self.side == "upper":
            self.event_separator += list(CutLine.upper_point_set)
            self.event_separator += CutLine.upper_line_list
        else:
            self.event_separator += list(CutLine.lower_point_set)
            self.event_separator += CutLine.lower_line_list
        self.task_separator += [self.event_separator]
        self.event_separator.register()
        self._refresh_path_visuals()

    def setup_pivy(self):
        """set up the scene"""
        self.shape = coin.SoSeparator()
        self.task_separator += [self.shape]
        
        # Separator for design paths (Bezier curves)
        self.path_separator = InteractionSeparator(self.rm)
        self.path_separator.selection_changed = self.path_selection_changed
        self.path_separator.register()  # Register for marker interaction
        self.shape += [self.path_separator]
        
        # Separator for add_neighbour temp markers  
        self.add_separator = InteractionSeparator(self.rm)
        self.shape += [self.add_separator]
        
        self.update_shape_display()  # Initialize shape properties based on mode
        self.draw_shape()
        self.event_separator = InteractionSeparator(self.rm)
        self.event_separator.selection_changed = self.selection_changed
        self.event_separator.register()
        self.toggle_side()
        self.add_cb = self.view.addEventCallbackPivy(
            coin.SoKeyboardEvent.getClassTypeId(), self.add_geo
        )

    def update_shape_display(self):
        """Rebuild the planform canvas.

        Half-wing mode: right half + synthetic centre rib at x=0 (legacy).
        Full-span mode: both real half-wings mirrored about x=0 (no synthetic
        centre rib); ``self._rib_meta[list_idx]`` carries the signed rib index
        so cut points know which physical wing/cell they belong to.
        """
        _shape = self.parametric_glider.shape.get_half_shape()
        has_center = self.parametric_glider.shape.has_center_cell

        if getattr(self, "full_span", False):
            # Real half ribs (positive x), then their mirror (negative x).
            half_front = list(_shape.front)   # [x0..x_tip]
            half_back = list(_shape.back)
            n = len(half_front)
            front_pts, back_pts, meta = [], [], []
            # left wing: -x_tip .. -x0  (signed rib -n .. -1)
            for i in range(n - 1, -1, -1):
                front_pts.append([-half_front[i][0], half_front[i][1]])
                back_pts.append([-half_back[i][0], half_back[i][1]])
                meta.append(-(i + 1))
            # right wing: x0 .. x_tip  (signed rib +1 .. +n)
            for i in range(n):
                front_pts.append([half_front[i][0], half_front[i][1]])
                back_pts.append([half_back[i][0], half_back[i][1]])
                meta.append(i + 1)
            self._rib_meta = meta
        else:
            front_pts = list(_shape.front)
            back_pts = list(_shape.back)
            # For odd cell count gliders, add the center rib at x=0
            if has_center:
                first_front = _shape.front[0]
                first_back = _shape.back[0]
                front_pts = [[0, first_front[1]]] + front_pts
                back_pts = [[0, first_back[1]]] + back_pts
            self._rib_meta = None

        self.front = vector3D(front_pts, z=-0.01)
        self.back = vector3D(back_pts, z=-0.01)
        self.ribs = list(zip(self.front, self.back))

        # Store rib bounds for use in other methods
        self._rib_front_pts = front_pts
        self._rib_back_pts = back_pts

        # Extract x_values from front coordinates
        self.x_values = [pt[0] for pt in front_pts]
    
    def _get_rib_bounds(self, list_idx):
        """Get front and back y coordinates for a rib at the given list index.
        
        Args:
            list_idx: Index in self.x_values/self.ribs (0 = center for odd cells)
            
        Returns:
            (front_y, back_y) - y coordinates of front and back of rib
        """
        front_y = self._rib_front_pts[list_idx][1]
        back_y = self._rib_back_pts[list_idx][1]
        return front_y, back_y
    
    def _list_idx_to_shape_idx(self, list_idx):
        """Convert list index (0-based including center) to shape index.
        
        For odd cells: list_idx=0 is center (no shape equivalent), list_idx=1 maps to shape[0], etc.
        For even cells: list_idx=i maps to shape[i]
        
        Returns shape index, or None if this is the center rib.
        """
        has_center = self.parametric_glider.shape.has_center_cell
        if has_center:
            if list_idx == 0:
                return None  # Center rib has no shape index
            return list_idx - 1
        return list_idx

    # ==================== Design Path Management ====================
    
    def _load_design_paths(self):
        """Load design paths from parametric_glider.elements."""
        paths_data = self.parametric_glider.elements.get("design_paths", [])
        for data in paths_data:
            path = DesignPath.from_dict(data)
            self.design_paths.append(path)
            self._path_counter = max(self._path_counter, 
                                     int(data.get("id", "path_0").split("_")[-1]) + 1)
    
    def _save_design_paths(self):
        """Save design paths to parametric_glider.elements."""
        self.parametric_glider.elements["design_paths"] = [
            path.to_dict() for path in self.design_paths
        ]
    
    def add_line_path(self):
        """Add a new straight line path."""
        # Create line across current visible shape
        if self.x_values and len(self.x_values) >= 2:
            x_start = self.x_values[0]
            x_end = self.x_values[-1]
            # Default position at 20% chord
            y_start = self.parametric_glider.shape[0, 0.2][1]
            y_end = self.parametric_glider.shape[-1, 0.2][1]
        else:
            x_start, x_end = 0, 5
            y_start, y_end = 0, 0
        
        path_id = f"path_{self._path_counter}"
        self._path_counter += 1
        
        cut_type = self.Qpath_cut_type.currentText()
        path = LinePath(
            path_id,
            cut_type,
            self.side,
            [[x_start, y_start], [x_end, y_end]]
        )
        self.design_paths.append(path)
        self._refresh_path_visuals()
        self._select_path(path)
        
    def add_polyline_path(self):
        """Add a new polyline path."""
        # Create polyline across current visible shape
        if self.x_values and len(self.x_values) >= 2:
            x_start = self.x_values[0]
            x_end = self.x_values[-1]
            x_mid = (x_start + x_end) / 2.0
            # Default position at 20% chord
            y_start = self.parametric_glider.shape[0, 0.2][1]
            y_end = self.parametric_glider.shape[-1, 0.2][1]
            y_mid = (y_start + y_end) / 2.0
            
            # Create a 3-point polyline by default
            pts = [[x_start, y_start], [x_mid, y_mid], [x_end, y_end]]
        else:
            pts = [[0, 0], [2.5, 0], [5, 0]]
            
        path_id = f"path_{self._path_counter}"
        self._path_counter += 1
        
        cut_type = self.Qpath_cut_type.currentText()
        path = PolylinePath(
            path_id,
            cut_type,
            self.side,
            pts
        )
        self.design_paths.append(path)
        self._refresh_path_visuals()
        self._select_path(path)
    
    def add_point_to_path(self):
        """Add a point to the end of the selected path."""
        if not self._selected_path or len(self._selected_path.control_points) == 0:
            return
            
        pts = self._selected_path.control_points
        if len(pts) >= 2:
            # Extrapolate from last two points
            p1 = pts[-2]
            p2 = pts[-1]
            new_pt = [p2[0] + (p2[0] - p1[0]) * 0.5, p2[1] + (p2[1] - p1[1]) * 0.5]
        else:
            # Just copy the last point with an offset
            p1 = pts[-1]
            new_pt = [p1[0] + 0.5, p1[1]]
            
        self._selected_path.control_points.append(new_pt)
        # Re-create visuals
        self._refresh_path_visuals()
        self._selected_path.select()
        self._update_point_buttons_state()
        
    def remove_point_from_path(self):
        """Remove the last point from the selected path."""
        if not self._selected_path:
            return
            
        # For Polyline, keep at least 2 points
        min_pts = 2
        # For Bezier, it's more complex, requiring specific amount of points, but we'll allow it generally,
        # but really this is meant for Polyline which is why we enforce 2 point minimum
        if isinstance(self._selected_path, BezierPath):
            min_pts = 4 # Keep at least 4 for Bezier
            
        if len(self._selected_path.control_points) > min_pts:
            self._selected_path.control_points.pop()
            # Re-create visuals
            self._refresh_path_visuals()
            self._selected_path.select()
            self._update_point_buttons_state()
            
    def _update_point_buttons_state(self):
        """Update enabled state of add/remove point buttons."""
        has_path = self._selected_path is not None
        can_remove = False
        
        if has_path:
            pts_count = len(self._selected_path.control_points)
            if isinstance(self._selected_path, PolylinePath):
                can_remove = pts_count > 2
            elif isinstance(self._selected_path, BezierPath):
                can_remove = pts_count > 4
                
        self.Qadd_point.setEnabled(has_path)
        self.Qremove_point.setEnabled(can_remove)
    
    def add_bezier_path(self):
        """Add a new cubic Bezier path."""
        # Create Bezier across current visible shape
        if self.x_values and len(self.x_values) >= 2:
            x_start = self.x_values[0]
            x_end = self.x_values[-1]
            x_mid = (x_start + x_end) / 2
            # Default position at 20% chord
            y_start = self.parametric_glider.shape[0, 0.2][1]
            y_end = self.parametric_glider.shape[-1, 0.2][1]
            y_mid = (y_start + y_end) / 2
        else:
            x_start, x_end, x_mid = 0, 5, 2.5
            y_start, y_end, y_mid = 0, 0, 0
        
        path_id = f"path_{self._path_counter}"
        self._path_counter += 1
        
        cut_type = self.Qpath_cut_type.currentText()
        # Create cubic Bezier with 4 control points
        path = BezierPath(
            path_id,
            cut_type,
            self.side,
            [
                [x_start, y_start],  # P0 - start
                [x_start + (x_end - x_start) * 0.33, y_start],  # P1 - control 1
                [x_start + (x_end - x_start) * 0.66, y_end],    # P2 - control 2
                [x_end, y_end]       # P3 - end
            ]
        )
        self.design_paths.append(path)
        self._refresh_path_visuals()
        self._select_path(path)
    
    def _select_path(self, path):
        """Select a design path."""
        if self._selected_path:
            self._selected_path.unselect()
        self._selected_path = path
        if path:
            path.select()
            # Update UI to show path's cut type
            idx = self.Qpath_cut_type.findText(path.cut_type)
            if idx >= 0:
                self.Qpath_cut_type.blockSignals(True)
                self.Qpath_cut_type.setCurrentIndex(idx)
                self.Qpath_cut_type.blockSignals(False)
        self._update_point_buttons_state()
    
    def path_cut_type_changed(self):
        """Handle path cut type change.

        Applies to the currently selected design path, and also to any
        CutLines selected in the event separator, so this dropdown can be
        used interchangeably with the "cut type" combobox.
        """
        text = self.Qpath_cut_type.currentText()
        if self._selected_path:
            self._selected_path.cut_type = text
        for element in self.event_separator.selected_objects:
            if isinstance(element, CutLine):
                element.cut_type = text
        # Mirror into the other combobox.
        cut_index = self.Qcut_type.findText(text)
        if cut_index is not None and cut_index >= 0:
            self.Qcut_type.blockSignals(True)
            self.Qcut_type.setCurrentIndex(cut_index)
            self.Qcut_type.blockSignals(False)
    
    def path_selection_changed(self):
        """Handle selection change in path separator - select the path of clicked marker."""
        for obj in self.path_separator.selected_objects:
            if hasattr(obj, '_path') and obj._path in self.design_paths:
                self._select_path(obj._path)
                break
    
    def delete_selected_path(self):
        """Delete the currently selected path."""
        if self._selected_path:
            # Remove from list
            self.design_paths.remove(self._selected_path)
            self._selected_path = None
            
            # Refresh all path visuals - clear and recreate
            self._refresh_path_visuals()
    
    def _refresh_path_visuals(self):
        """Clear and recreate all path visuals."""
        # Unregister and remove old separator to ensure a completely clean state
        if hasattr(self, 'path_separator') and self.path_separator is not None:
            self.path_separator.select_object(None)
            self.path_separator.unregister()
            if self.path_separator in self.shape:
                self.shape.removeChild(self.path_separator)
            del self.path_separator
            
        self.path_separator = InteractionSeparator(self.rm)
        self.path_separator.selection_changed = self.path_selection_changed
        self.shape += [self.path_separator]
        
        # Recreate visuals only for paths matching current side
        for path in self.design_paths:
            path.curve_line = None
            path.markers = []
            if path.side == self.side:
                path.setup_visuals(self.path_separator)
                
        # Register separator *after* all event nodes have been added
        self.path_separator.register()
    
    def apply_paths_to_cuts(self):
        """Project all design paths onto panel cuts.
        
        Each path is projected at most once per editing session.  A path
        whose ``_applied`` flag is True is skipped to avoid creating
        duplicate CutPoints.  The flag is reset automatically whenever the
        user drags one of the path's control points, so modified paths can
        be re-applied cleanly.
        """
        has_center = self.parametric_glider.shape.has_center_cell
        
        # Build rib bounds from stored front/back points
        rib_bounds = list(zip(
            [pt[1] for pt in self._rib_front_pts],
            [pt[1] for pt in self._rib_back_pts]
        ))
        
        applied_any = False
        for path in self.design_paths:
            if path.side != self.side:
                continue  # Only apply paths for current side

            if path._applied:
                continue  # Already projected - skip to avoid duplicates
            
            # Get intersections with rib bounds included for proper center rib handling
            intersections = path.get_rib_intersections(
                self.x_values, 
                rib_bounds=rib_bounds
            )
            
            if len(intersections) < 2:
                continue
            
            # For odd cell count, force the center rib to be perpendicular/symmetric
            # by using the same relative chord position as the first visible rib
            if has_center and len(intersections) > 1:
                # Find if index 0 (center) and index 1 (first visible) are in intersections
                center_found = any(idx == 0 for idx, _ in intersections)
                first_visible_found = any(idx == 1 for idx, _ in intersections)
                
                if center_found and first_visible_found:
                    # Get the first visible rib's y position and compute relative chord position
                    first_visible_y = None
                    for idx, y in intersections:
                        if idx == 1:
                            first_visible_y = y
                            break
                    
                    if first_visible_y is not None:
                        # Compute relative position on first visible rib
                        fv_front, fv_back = self._get_rib_bounds(1)
                        fv_chord = fv_front - fv_back
                        
                        if abs(fv_chord) > 0.001:
                            rel_pos = (fv_front - first_visible_y) / fv_chord
                            
                            # Apply same relative position to center rib
                            c_front, c_back = self._get_rib_bounds(0)
                            c_chord = c_front - c_back
                            center_y_perpendicular = c_front - rel_pos * c_chord
                            
                            # Replace center intersection with perpendicular position
                            intersections = [
                                (0, center_y_perpendicular) if idx == 0 else (idx, y)
                                for idx, y in intersections
                            ]
            
            # Create CutPoints and CutLines from intersections
            cut_points = []
            
            for list_idx, y_pos in intersections:
                try:
                    # Convert list_idx to the correct rib index for CutPoint
                    # list_idx 0 = center for odd cells, needs special handling
                    cp = self._create_cut_point_at_list_idx(list_idx, y_pos)
                    cut_points.append((list_idx, cp))
                except (IndexError, TypeError) as e:
                    print(f"Skipping rib {list_idx}: {e}")
                    continue
            
            # Create lines between consecutive points
            path_cut_lines_created = 0
            for (i1, cp1), (i2, cp2) in zip(cut_points[:-1], cut_points[1:]):
                if abs(i1 - i2) == 1:  # Adjacent ribs
                    if self.side == "upper":
                        CutLine.upper_point_set.add(cp1)
                        CutLine.upper_point_set.add(cp2)
                    else:
                        CutLine.lower_point_set.add(cp1)
                        CutLine.lower_point_set.add(cp2)
                    
                    cut_line = CutLine(cp1, cp2, path.cut_type, wing=self.wing)
                    cut_line._path_id = path.path_id  # link cut -> path (surface flip)
                    cut_line.replace_points_by_set()
                    cut_line.update_Line()
                    cut_line.setup_visuals()
                    self.event_separator += [cp1, cp2, cut_line]
                    path_cut_lines_created += 1
            
            if path_cut_lines_created > 0:
                # Mark as applied only when at least one CutLine was created
                path._applied = True
                applied_any = True
        
        if applied_any:
            self.event_separator.color_selected()
    
    def _create_cut_point_at_list_idx(self, list_idx, y_pos):
        """Create a CutPoint for a rib at the given list index.
        
        Handles the conversion from list_idx (which includes center for odd cells)
        to the proper shape index used by CutPoint.
        
        Note on zero-chord wingtip ribs
        ---------------------------------
        The wingtip rib often has chord ≈ 0 (pointed tip). In that case we
        cannot compute a meaningful normalised chord position, but we MUST
        preserve the side sign so that ``CutLine.is_upper`` (which tests
        ``rib_pos < 0``) can correctly route the line to the upper or lower
        set.  We therefore assign a tiny symbolic value (±1e-4) instead of 0.
        """
        has_center = self.parametric_glider.shape.has_center_cell
        
        # Get x coordinate and bounds for this rib
        x_value = self.x_values[list_idx]
        front_y, back_y = self._get_rib_bounds(list_idx)
        chord = abs(front_y - back_y)
        
        upper = self.side == "upper"
        if chord < 0.001:
            # Zero-chord (pointed) wingtip: preserve side sign so that
            # CutLine.is_upper correctly categorises the resulting line.
            rib_pos = -1e-4 if upper else 1e-4
        else:
            # rib_pos is negative for upper (extrados), positive for lower (intrados).
            # Normalised distance from leading edge, signed by surface side.
            rib_pos = -(upper * 2.0 - 1.0) * abs(y_pos - front_y) / chord
            # A cut snapped exactly onto the leading edge has rib_pos == 0, which
            # would lose the surface side (is_upper tests rib_pos <= 0). Keep a
            # tiny signed value so the cut stays on its own surface.
            if abs(rib_pos) < 1e-4:
                rib_pos = -1e-4 if upper else 1e-4
        
        # Full-span mode: tag the point with its signed rib / wing (derived
        # from the planform side) so cut lines resolve to the right cell+wing.
        if getattr(self, "full_span", False) and self._rib_meta is not None:
            signed = self._rib_meta[list_idx]
            wing = "left" if signed < 0 else "right"
            return CutPoint(
                abs(signed) + has_center, rib_pos, self.parametric_glider,
                x_value=x_value, min_y=front_y, max_y=back_y,
                signed_rib=signed, wing=wing,
            )

        # CutPoint expects rib_nr to be: list_idx + has_center_cell (then subtracts has_center_cell)
        # After subtraction, rib_nr becomes list_idx, which is correct
        rib_nr_for_cutpoint = list_idx + has_center

        # Pass x_value, min_y, max_y explicitly to handle center rib correctly
        return CutPoint(
            rib_nr_for_cutpoint, rib_pos, self.parametric_glider,
            x_value=x_value,
            min_y=front_y,
            max_y=back_y
        )

    def add_cuts_at_percentage(self):
        """Add cut points and lines at a given chord percentage across all ribs."""
        percentage = self.Qpercentage.value() / 100.0  # Convert to 0-1 range
        cut_type = self.Qpath_cut_type.currentText()
        
        cut_points = []
        num_ribs = len(self.x_values)
        
        # Iterate through all ribs (x_values now includes center for odd cells)
        for list_idx in range(num_ribs):
            try:
                # Get rib bounds and compute y position from percentage
                front_y, back_y = self._get_rib_bounds(list_idx)
                chord = front_y - back_y
                y_pos = front_y - percentage * chord
                
                # Create cut point using the helper
                cp = self._create_cut_point_at_list_idx(list_idx, y_pos)
                cut_points.append((list_idx, cp))
            except (IndexError, TypeError) as e:
                print(f"Skipping rib {list_idx}: {e}")
                continue
        
        # Create lines between consecutive points
        for (i1, cp1), (i2, cp2) in zip(cut_points[:-1], cut_points[1:]):
            if abs(i1 - i2) == 1:  # Adjacent ribs
                if self.side == "upper":
                    CutLine.upper_point_set.add(cp1)
                    CutLine.upper_point_set.add(cp2)
                else:
                    CutLine.lower_point_set.add(cp1)
                    CutLine.lower_point_set.add(cp2)
                
                cut_line = CutLine(cp1, cp2, cut_type, wing=self.wing)
                cut_line.replace_points_by_set()
                cut_line.update_Line()
                cut_line.setup_visuals()
                self.event_separator += [cp1, cp2, cut_line]

        self.event_separator.color_selected()

    def select_connected(self):
        """Select all points and lines connected to the current selection.
        
        Uses graph traversal to follow connections through CutLines.
        """
        if not self.event_separator.selected_objects:
            return
        
        # Collect initial points from selection
        visited_points = set()
        visited_lines = set()
        to_visit = []
        
        for elem in self.event_separator.selected_objects:
            if isinstance(elem, CutPoint):
                to_visit.append(elem)
            elif isinstance(elem, CutLine):
                to_visit.append(elem.point1)
                to_visit.append(elem.point2)
        
        # Graph traversal - follow lines to find all connected points
        while to_visit:
            point = to_visit.pop()
            if point in visited_points:
                continue
            visited_points.add(point)
            
            # Find all lines connected to this point
            for line in point.lines:
                if line not in visited_lines:
                    visited_lines.add(line)
                    # Add the other endpoint to visit
                    other = line.point2 if line.point1 == point else line.point1
                    if other not in visited_points:
                        to_visit.append(other)
        
        # Select all connected elements
        for point in visited_points:
            if point not in self.event_separator.selected_objects:
                self.event_separator.select_object(point, multi=True)
        for line in visited_lines:
            if line not in self.event_separator.selected_objects:
                self.event_separator.select_object(line, multi=True)

    def delete_selected_cuts(self):
        """Delete the selected CutLines / CutPoints without cascading
        through shared endpoints.

        CutPoints at the same (rib_nr, rib_pos) are intentionally shared
        between adjacent cells (that is what makes ``select_connected``
        traverse a whole chord-percentage chain). Pivy's native
        ``remove_selected`` propagates ``_delete`` through those shared
        points via ``CutLine.check_dependency``, which wipes every cut
        that touches the shared endpoint - not just the one the user
        clicked.

        Here we instead build the explicit set of CutLines to remove:
        - any selected CutLine,
        - plus every CutLine incident to a selected CutPoint (a point
          selection still means "remove the cut(s) ending here").
        Then we detach those lines from their endpoints before flagging
        them, so neighbouring lines keep their shared point alive. A
        point is only removed once no surviving line references it.
        """
        sep = self.event_separator
        selected = list(sep.selected_objects)
        if not selected:
            return

        lines_to_delete = set()
        points_explicit = set()
        for el in selected:
            if isinstance(el, CutLine):
                lines_to_delete.add(el)
            elif isinstance(el, CutPoint):
                points_explicit.add(el)

        for pt in points_explicit:
            for ln in list(pt.lines):
                lines_to_delete.add(ln)

        for ln in lines_to_delete:
            for pt in (ln.point1, ln.point2):
                if ln in pt.lines:
                    pt.lines.remove(ln)
            if ln.is_upper:
                if ln in CutLine.upper_line_list:
                    CutLine.upper_line_list.remove(ln)
            else:
                if ln in CutLine.lower_line_list:
                    CutLine.lower_line_list.remove(ln)
            ln._delete = True

        points_to_delete = set(points_explicit)
        for ln in lines_to_delete:
            for pt in (ln.point1, ln.point2):
                if not pt.lines:
                    points_to_delete.add(pt)

        for pt in points_to_delete:
            CutLine.upper_point_set.discard(pt)
            CutLine.lower_point_set.discard(pt)
            pt._delete = True

        to_remove = [o for o in sep.dynamic_objects + sep.static_objects
                     if getattr(o, "_delete", False)]
        sep.selected_objects = []
        sep.over_object = None
        for obj in to_remove:
            if obj in sep.dynamic_objects:
                sep.dynamic_objects.remove(obj)
            elif obj in sep.static_objects:
                sep.static_objects.remove(obj)
            try:
                sep.objects.removeChild(obj)
            except Exception:
                pass
        sep.selection_changed()

    def selection_changed(self):
        points = set()
        lines = []
        for element in self.event_separator.selected_objects:
            if isinstance(element, CutPoint):
                points.add(element)
            elif isinstance(element, CutLine):
                lines.append(element)
                points.add(element.point1)
                points.add(element.point2)

        self.Qcut_type.setEnabled(bool(lines))
        self.QPointPos.setEnabled(bool(points))
        if lines:
            self.set_cut_type(lines[0].cut_type)
        if points:
            self.QPointPos.blockSignals(True)
            self.QPointPos.setValue(
                abs(list(points)[0].rib_pos)
            )  # maybe summing over all points?
            self.QPointPos.blockSignals(False)

    def set_cut_type(self, text):
        # Keep both dropdowns in sync with the currently selected cut so the
        # user does not have to guess which combobox reflects the real type.
        index = self.Qcut_type.findText(text)
        if index is not None and index >= 0:
            self.Qcut_type.blockSignals(True)
            self.Qcut_type.setCurrentIndex(index)
            self.Qcut_type.blockSignals(False)
        path_index = self.Qpath_cut_type.findText(text)
        if path_index is not None and path_index >= 0:
            self.Qpath_cut_type.blockSignals(True)
            self.Qpath_cut_type.setCurrentIndex(path_index)
            self.Qpath_cut_type.blockSignals(False)

    def cut_type_changed(self):
        text = self.Qcut_type.currentText()
        for element in self.event_separator.selected_objects:
            if isinstance(element, CutLine):
                element.cut_type = text
        # Mirror the change into the path dropdown so both stay consistent.
        path_index = self.Qpath_cut_type.findText(text)
        if path_index is not None and path_index >= 0:
            self.Qpath_cut_type.blockSignals(True)
            self.Qpath_cut_type.setCurrentIndex(path_index)
            self.Qpath_cut_type.blockSignals(False)

    def point_pos_changed(self):
        points = set()
        lines = set()
        sign = 2.0 * (self.side != "upper") - 1.0
        for element in self.event_separator.selected_objects:
            if isinstance(element, CutPoint):
                points.add(element)
            elif isinstance(element, CutLine):
                lines.add(element)
                points.add(element.point1)
                points.add(element.point2)
        for point in points:
            point.rib_pos = self.QPointPos.value() * sign
            point.update_position()
            for line in point.lines:
                lines.add(line)
        for line in lines:
            line.update_Line()

    def draw_shape(self):
        """draws the shape of the glider"""
        l1 = Line(self.front)
        l2 = Line(self.back)
        l3 = Line([self.back[0], self.front[0]])
        l4 = Line([self.back[-1], self.front[-1]])
        l_ribs = list(map(Line, self.ribs))
        lines = [l1, l2, l3, l4] + l_ribs
        for l in lines:
            l.color.diffuseColor = (0.2, 0.2, 0.2)
        self._shape_lines = lines  # Track lines for redraw
        self.shape += lines

    def redraw_shape(self):
        """Clear and redraw the shape (used when switching modes)"""
        # Remove existing shape lines from the separator
        for line in self._shape_lines:
            try:
                self.shape.removeChild(line)
            except:
                pass  # Line may not be part of shape anymore
        self._shape_lines = []
        # Draw new shape
        self.draw_shape()

    def toggle_side(self):
        self.event_separator.select_object(None)
        self.event_separator.unregister()
        self.task_separator.removeChild(self.event_separator)
        del self.event_separator
        self.event_separator = InteractionSeparator(self.rm)
        self.event_separator.selection_changed = self.selection_changed
        if self.side == "upper":
            self.side = "lower"
            self.Qtoggle_side.setText("show upper side")
            self.event_separator += list(CutLine.lower_point_set)
            self.event_separator += CutLine.lower_line_list
        elif self.side == "lower":
            self.side = "upper"
            self.Qtoggle_side.setText("show lower side")
            self.event_separator += list(CutLine.upper_point_set)
            self.event_separator += CutLine.upper_line_list
        self.task_separator += [self.event_separator]
        self.event_separator.register()
        
        # Update path visibility for new side
        self._refresh_path_visuals()
        self._selected_path = None

    def add_geo(self, event_callback):
        """this function provides some interaction functionality to create points and lines
        press I to start the mode. if a point is selected, the left and right rib will offer the possibility to add
        a point + line, if no point is selected, it's possible to add a point to any rib.
        Press I again while in add-mode to cancel."""
        event = event_callback.getEvent()
        if event.getKey() != ord("i"):
            return
        # Key released - ignore
        if event.getState() != 0:
            return

        # Allow pressing I again to cancel an active add_mode
        if self._add_mode:
            self._cancel_add_mode()
            return

        self._add_mode = True
        # first we check if nothing is selected:
        select_obj = self.event_separator.selected_objects
        num_of_obj = len(select_obj)
        action = None
        self._add_event = None
        self._close_event = None

        # insert a point
        if num_of_obj == 0:
            self._add_event = self.view.addEventCallbackPivy(
                coin.SoLocation2Event.getClassTypeId(), self.add_point
            )
            action = self.add_point

        # insert a line + point
        elif num_of_obj == 1 and isinstance(select_obj[0], CutPoint):
            self._add_event = self.view.addEventCallbackPivy(
                coin.SoLocation2Event.getClassTypeId(), self.add_neighbour
            )
            self.add_neighbour(event_callback)
            action = self.add_neighbour

        # join two points with a line
        elif num_of_obj == 2 and all(isinstance(el, CutPoint) for el in select_obj):
            cut_point_1 = select_obj[0]
            cut_point_2 = select_obj[1]
            if abs(cut_point_1.rib_nr - cut_point_2.rib_nr) == 1:
                cut_line = CutLine(cut_point_1, cut_point_2, "folded")
                cut_line.replace_points_by_set()
                cut_line.update_Line()
                cut_line.setup_visuals()
                self.event_separator += [cut_line]
            # No async action needed - fall through to direct reset below

        def remove_cb(event_callback=None):
            if event_callback:
                event = event_callback.getEvent()
                if not event.getButton() == coin.SoMouseButtonEvent.BUTTON1:
                    return
            self._finalize_add_mode(action)

        if action in [self.add_neighbour, self.add_point]:
            # these functions need an extra callback for closing on mouse button press
            self._close_event = self.view.addEventCallbackPivy(
                coin.SoMouseButtonEvent.getClassTypeId(), remove_cb
            )
        else:
            # add line (or no-op): call remove_callback directly
            self._finalize_add_mode(action)

    def _cancel_add_mode(self):
        """Cancel an in-progress add_mode, cleaning up callbacks."""
        if hasattr(self, '_add_event') and self._add_event:
            try:
                self.view.removeEventCallbackPivy(
                    coin.SoLocation2Event.getClassTypeId(), self._add_event
                )
            except Exception:
                pass
            self._add_event = None
        if hasattr(self, '_close_event') and self._close_event:
            try:
                self.view.removeEventCallbackPivy(
                    coin.SoMouseButtonEvent.getClassTypeId(), self._close_event
                )
            except Exception:
                pass
            self._close_event = None
        self._add_mode = False
        self.add_separator.removeAllChildren()

    def _finalize_add_mode(self, action):
        """Finalize the add_mode: clean up callbacks, commit geometry."""
        # Remove callbacks
        if hasattr(self, '_add_event') and self._add_event:
            try:
                self.view.removeEventCallbackPivy(
                    coin.SoLocation2Event.getClassTypeId(), self._add_event
                )
            except Exception:
                pass
            self._add_event = None
        if hasattr(self, '_close_event') and self._close_event:
            try:
                self.view.removeEventCallbackPivy(
                    coin.SoMouseButtonEvent.getClassTypeId(), self._close_event
                )
            except Exception:
                pass
            self._close_event = None

        try:
            if action == self.add_neighbour:
                static = self.add_separator.static_objects
                if len(static) >= 2 and isinstance(static[0], Marker) and isinstance(static[1], Line):
                    marker = static[0]
                    line = static[1]
                    cut_point_1 = self._create_cut_point_at_list_idx(
                        marker.rib_nr,
                        marker.points[0][1]
                    )
                    cut_point_2 = line.active_point
                    cut_line = CutLine(cut_point_1, cut_point_2, "folded")
                    cut_line.replace_points_by_set()
                    cut_line.update_Line()
                    cut_line.setup_visuals()
                    self.event_separator += [cut_point_1, cut_line]
                    self.event_separator.select_object(cut_point_1)

            elif action == self.add_point:
                static = self.add_separator.static_objects
                if len(static) >= 1 and isinstance(static[0], Marker):
                    marker = static[0]
                    cut_point = self._create_cut_point_at_list_idx(
                        marker.rib_nr,
                        marker.points[0][1]
                    )
                    self.event_separator += [cut_point]
                    self.event_separator.select_object(cut_point)
        except Exception as e:
            print(f"Design Tool: error finalizing add_mode: {e}")
        finally:
            self._add_mode = False
            self.add_separator.removeAllChildren()

    def add_point(self, event_callback=None):
        event = event_callback.getEvent()
        # first get the closest rib
        pos = event.getPosition()
        pos = list(self.view.getPoint(*pos))
        pos[2] = 0.0

        # Find the closest rib by x distance
        best_diff = None
        index = 0
        for i, value in enumerate(self.x_values):
            diff = abs(pos[0] - value)
            if best_diff is None or diff < best_diff:
                best_diff = diff
                index = i
        
        # Use stored rib bounds (works correctly for center rib)
        x = self.x_values[index]
        front_y, back_y = self._get_rib_bounds(index)
        min_y = min(front_y, back_y)
        max_y = max(front_y, back_y)
        
        pos[0] = x
        # Snap clicks at or just beyond an edge onto the leading/trailing edge
        # so a cut can be placed right at the edge and split a panel fully.
        tol = EDGE_SNAP_CHORD_FACTOR * (max_y - min_y)
        if min_y - tol <= pos[1] <= max_y + tol:
            pos[1] = min(max(pos[1], min_y), max_y)
            if len(self.add_separator.static_objects) == 0:
                self.add_separator.removeAllChildren()
                marker = Marker([pos])
                marker.rib_nr = index  # This is the list index
                self.add_separator += [marker]
            else:
                marker = self.add_separator.static_objects[0]
                marker.points = [pos]
                marker.rib_nr = index
        else:
            self.add_separator.removeAllChildren()

    def add_neighbour(self, event_callback=None):
        event = event_callback.getEvent()
        select_obj = self.event_separator.selected_objects[0]
        rib_nr = select_obj.rib_nr  # This is the list index
        
        # Get bounds for left neighbour (rib_nr - 1)
        x1, min1, max1 = None, None, None
        if rib_nr > 0:
            try:
                x1 = self.x_values[rib_nr - 1]
                front_y, back_y = self._get_rib_bounds(rib_nr - 1)
                min1 = min(front_y, back_y)
                max1 = max(front_y, back_y)
            except (IndexError, KeyError):
                pass
        
        # Get bounds for right neighbour (rib_nr + 1)
        x2, min2, max2 = None, None, None
        if rib_nr < len(self.x_values) - 1:
            try:
                x2 = self.x_values[rib_nr + 1]
                front_y, back_y = self._get_rib_bounds(rib_nr + 1)
                min2 = min(front_y, back_y)
                max2 = max(front_y, back_y)
            except (IndexError, KeyError):
                pass
        
        show_point = False
        pos = event.getPosition()
        pos = list(self.view.getPoint(*pos))
        pos[2] = 0

        # FIX: Use explicit None checks instead of truthiness tests.
        # `if not x2` was True when x2==0.0 (center rib at x=0), causing
        # the left neighbour to always be preferred near the center.
        left_available = x1 is not None
        right_available = x2 is not None
        
        # Decide which side the cursor is closer to
        choose_left = False
        if left_available and right_available:
            choose_left = abs(pos[0] - x1) <= abs(pos[0] - x2)
        elif left_available:
            choose_left = True
        # else: only right available, choose_left stays False

        if choose_left and left_available:
            pos[0] = x1
            tol1 = EDGE_SNAP_CHORD_FACTOR * (max1 - min1)
            if min1 - tol1 <= pos[1] <= max1 + tol1:
                pos[1] = min(max(pos[1], min1), max1)
                new_rib_nr = rib_nr - 1
                show_point = True
        elif right_available:
            pos[0] = x2
            tol2 = EDGE_SNAP_CHORD_FACTOR * (max2 - min2)
            if min2 - tol2 <= pos[1] <= max2 + tol2:
                pos[1] = min(max(pos[1], min2), max2)
                new_rib_nr = rib_nr + 1
                show_point = True
            
        if show_point:
            if not self.add_separator.static_objects:
                self.add_separator.removeAllChildren()
                marker = Marker([pos])
                marker.rib_nr = new_rib_nr
                line = Line([list(select_obj.points[0]), pos])
                line.active_point = select_obj
                self.add_separator += [marker, line]
            else:
                marker = self.add_separator.static_objects[0]
                line = self.add_separator.static_objects[1]
                marker.points = [pos]
                marker.rib_nr = new_rib_nr
                line.points = [list(select_obj.points[0]), pos]
        else:
            self.add_separator.removeAllChildren()

    def accept(self):
        self.event_separator.unregister()
        self.view.removeEventCallbackPivy(
            coin.SoKeyboardEvent.getClassTypeId(), self.add_cb
        )
        # Save design paths
        self._save_design_paths()
        # Get cuts and handle symmetric mirroring if needed
        cuts = CutLine.get_cut_dict()
        self.parametric_glider.elements["cuts"] = cuts
        super().accept()
        self.update_view_glider()

    def reject(self):
        self.event_separator.unregister()
        self.view.removeEventCallbackPivy(
            coin.SoKeyboardEvent.getClassTypeId(), self.add_cb
        )
        super().reject()


class CutPoint(Marker):
    def __init__(self, rib_nr, rib_pos, parametric_glider=None, x_value=None,
                 min_y=None, max_y=None, signed_rib=None, wing="both"):
        super().__init__([[0, 0, 0]], True)
        self.marker.markerIndex = coin.SoMarkerSet.CROSS_7_7
        self.parametric_glider = parametric_glider
        self.rib_nr = rib_nr - parametric_glider.shape.has_center_cell
        self.rib_pos = rib_pos
        # Full-span (asymmetric) canvas: signed rib index (>0 right, <0 left)
        # and the physical wing.  None in half-wing mode.
        self.signed_rib = signed_rib
        self.wing = wing
        self.lines = []
        
        # Flag to indicate if bounds were explicitly provided (for center rib)
        self._has_explicit_bounds = x_value is not None
        
        # Use provided values if available, otherwise compute from shape
        if self._has_explicit_bounds:
            self.x_value = x_value
            self.min = min_y if min_y is not None else 0
            self.max = max_y if max_y is not None else 0
            # Compute 2D point directly
            chord = self.min - self.max
            y = self.min - abs(rib_pos) * chord
            point = [self.x_value, y, 0]
        else:
            point = self._get_2D_from_shape()
            self.x_value = point[0]
            self.max = self._get_shape_point(1.0)[1]
            self.min = self._get_shape_point(0.0)[1]
        
        self.points = [point]
        self.on_drag_release.append(self.get_rib_pos)

    def update_position(self):
        self.points = [self.get_2D()]

    def _get_shape_point(self, pos_fraction):
        """Helper to correctly map self.rib_nr to the parametric shape's internal indexing.
        
        When has_center_cell is True, self.rib_nr=0 represents the center rib (x=0).
        However, the parametric shape's __getitem__ does not have a native index for x=0.
        shape[0] returns the first physical half-rib (x=x1).
        This method correctly translates self.rib_nr to the shape's point.
        """
        has_center = self.parametric_glider.shape.has_center_cell
        try:
            if has_center:
                if self.rib_nr == 0:
                    # Center rib (x=0). Y coordinates are the same as the first half-rib.
                    pt = list(self.parametric_glider.shape[0, pos_fraction])
                    pt[0] = 0.0  # Force x=0
                    return pt
                else:
                    # 1st half-rib (self.rib_nr=1) is at shape[0], etc.
                    return list(self.parametric_glider.shape[self.rib_nr - 1, pos_fraction])
            else:
                return list(self.parametric_glider.shape[self.rib_nr, pos_fraction])
        except IndexError:
            raise IndexError(f"index {self.rib_nr} out of range")

    def _get_2D_from_shape(self):
        """Get 2D position from shape indexing."""
        pt = self._get_shape_point(abs(self.rib_pos))
        return pt + [0]

    def get_2D(self):
        """Get 2D position, using stored bounds if available."""
        if self._has_explicit_bounds:
            # Use stored bounds to compute position
            chord = self.min - self.max
            y = self.min - abs(self.rib_pos) * chord
            return [self.x_value, y, 0]
        else:
            return self._get_2D_from_shape()

    def get_rib_pos(self):
        """Update rib_pos from current y position after drag.
        
        Convention: rib_pos < 0 for upper surface, > 0 for lower surface.
        Computed as signed distance from LE (min) normalised by chord.
        """
        if self._has_explicit_bounds:
            le = self.min   # LE has the larger y value (front)
            te = self.max   # TE has the smaller y value (back)
        else:
            le = self.parametric_glider.shape[self.rib_nr, 0][1]
            te = self.parametric_glider.shape[self.rib_nr, 1][1]
        chord = le - te
        if abs(chord) < 1e-6:
            return self.rib_pos
        # Preserve the sign of the original rib_pos (upper=negative, lower=positive)
        sign = -1 if self.rib_pos < 0 else 1
        # Distance from LE, normalised by chord - always positive
        norm_dist = (le - self.pos[1]) / chord
        norm_dist = max(0.0, min(1.0, norm_dist))  # Clamp to valid range
        self.rib_pos = round(sign * norm_dist, 3)
        return self.rib_pos

    def __eq__(self, other):
        if isinstance(other, CutPoint):
            # In full-span mode a left and a right point can share rib_nr and
            # rib_pos; the signed rib keeps them distinct so replace_by_set()
            # never welds a left point onto its right mirror.
            if getattr(self, "signed_rib", None) != getattr(other, "signed_rib", None):
                return False
            if self.rib_nr == other.rib_nr:
                if round(self.rib_pos, 3) == round(other.rib_pos, 3):
                    return True
        return False

    def __hash__(self):
        return hash(self.rib_nr) ^ hash(round(self.rib_pos, 2))

    def replace_by_set(self, point_set):
        for point in point_set:
            if point == self:
                return point
        return False

    @property
    def pos(self):
        return [self.x_value, self.points[0][1], 0]

    @pos.setter
    def pos(self, pos):
        # FIX: self.points expects a list-of-points, not a single point
        self.points = [[self.x_value, pos[1], 0]]

    def drag(self, mouse_coords, fact=1.0):
        if self.enabled:
            pts = self.points
            for i, pt in enumerate(pts):
                pt[0] = self._tmp_points[i][0]
                y = mouse_coords[1] * fact + self._tmp_points[i][1]
                if y > self.min:
                    pt[1] = self.min
                elif y < self.max:
                    pt[1] = self.max
                else:
                    pt[1] = y
                pt[2] = self._tmp_points[i][2]
            self.points = pts
            for i in self.on_drag:
                i()

    @classmethod
    def from_position_and_rib(cls, rib_nr, y_pos, upper, parametric_glider):
        i = 0
        max_val = parametric_glider.shape[rib_nr, 1.0][1]
        min_val = parametric_glider.shape[rib_nr, 0.0][1]
        rib_pos = -(upper * 2.0 - 1.0) * abs(y_pos - min_val) / abs(max_val - min_val)
        return cls(
            rib_nr + parametric_glider.shape.has_center_cell, rib_pos, parametric_glider
        )


class CutLine(Line):
    upper_point_set = set()
    lower_point_set = set()
    upper_line_list = []
    lower_line_list = []

    def __init__(self, point1, point2, cut_type, wing="both"):
        super().__init__([point1.get_2D(), point2.get_2D()], dynamic=True)
        self.drawstyle.lineWidth = 1.5
        self.point1 = point1
        self.point2 = point2
        self.parametric_glider = self.point1.parametric_glider
        self.cut_type = cut_type
        # Fallback wing (half-wing mode). In full-span mode the wing is derived
        # from the endpoints' signed ribs via the ``wing`` property below.
        self._wing = wing
        if self.is_upper:
            CutLine.upper_point_set.add(point1)
            CutLine.upper_point_set.add(point2)
            CutLine.upper_line_list.append(self)
        else:
            CutLine.lower_point_set.add(point1)
            CutLine.lower_point_set.add(point2)
            CutLine.lower_line_list.append(self)

    def setup_visuals(self):
        self.point1.lines.append(self)
        self.point2.lines.append(self)
        self.point1.on_drag.append(self.update_Line)
        self.point2.on_drag.append(self.update_Line)

    def replace_points_by_set(self):
        # this has to be done once for every line (before the parent Line is initialized)
        if self.is_upper:
            self.point1 = self.point1.replace_by_set(CutLine.upper_point_set)
            self.point2 = self.point2.replace_by_set(CutLine.upper_point_set)
        else:
            self.point1 = self.point1.replace_by_set(CutLine.lower_point_set)
            self.point2 = self.point2.replace_by_set(CutLine.lower_point_set)

    def update_Line(self):
        self.points = [self.point1.pos, self.point2.pos]

    @property
    def is_upper(self):
        """True when this cut is on the upper (extrados) surface.
        
        Convention: rib_pos < 0  → upper surface.
                    rib_pos > 0  → lower surface.
                    rib_pos == 0 → zero-chord wingtip (treated as upper since
                                   both surfaces converge to the same point).
        We use ``<= 0`` so that wingtip CutPoints (rib_pos = ±1e-4 by
        convention, or exactly 0 when loaded from an old file) are included
        in the upper set and remain visible when viewing the upper side.
        """
        return self.point1.rib_pos <= 0 and self.point2.rib_pos <= 0

    def drag(self, mouse_coords, fact=1.0):
        self.point1.drag(mouse_coords, fact)
        self.point2.drag(mouse_coords, fact)

    @property
    def drag_objects(self):
        return [self.point1, self.point2]

    @property
    def points(self):
        return self.data.point.getValues()

    @points.setter
    def points(self, points):
        p = [[pi[0], pi[1], pi[2] - 0.001] for pi in points]
        self.data.point.setValue(0, 0, 0)
        self.data.point.setValues(0, len(p), p)

    @classmethod
    def _fs_cutpoint(cls, parametric_glider, signed_rib, rib_pos):
        """Build a CutPoint at a signed rib (mirrored to -x on the left wing)."""
        has_center = parametric_glider.shape.has_center_cell
        k = abs(signed_rib)
        # rib_nr arg chosen so _get_shape_point returns shape[k-1] (= x_{k-1}).
        arg = (k + has_center) if has_center else (k - 1)
        ref = CutPoint(arg, rib_pos, parametric_glider)   # positive-side coords
        x, mn, mx = ref.x_value, ref.min, ref.max
        wing = "right"
        if signed_rib < 0:
            x = -x
            wing = "left"
        return CutPoint(arg, rib_pos, parametric_glider, x_value=x,
                        min_y=mn, max_y=mx, signed_rib=signed_rib, wing=wing)

    @classmethod
    def cuts_to_lines(cls, parametric_glider, full_span=False):
        """Convert stored cuts into visual CutLine objects.

        Half-wing mode (default): only the right/centre half is shown; a cut's
        ``cells`` are 0-based with 0 = centre cell for odd gliders.
        Full-span mode: both wings are drawn.  A symmetric ("both") cut is
        exploded onto both wings so it can be edited independently; a
        side-tagged cut is drawn only on its wing.  See
        openglider.glider.parametric.asymmetric.
        """
        from openglider.glider.parametric.asymmetric import (
            expand_cut_placements,
            signed_ribs_for,
        )

        CutLine.upper_point_set = set()
        CutLine.lower_point_set = set()
        CutLine.upper_line_list = []
        CutLine.lower_line_list = []
        has_center = parametric_glider.shape.has_center_cell
        for cut in parametric_glider.elements.get("cuts", []):
            if full_span:
                for cell, side in expand_cut_placements(cut, has_center):
                    inner_s, outer_s = signed_ribs_for(cell, side, has_center)
                    try:
                        CutLine(
                            cls._fs_cutpoint(parametric_glider, inner_s, cut["left"]),
                            cls._fs_cutpoint(parametric_glider, outer_s, cut["right"]),
                            cut["type"],
                        )
                    except (TypeError, IndexError) as e:
                        print(f"Design Tool: skipping cut cell {cell}/{side}: {e}")
                continue
            for cell_nr in cut["cells"]:
                if cell_nr < 0:
                    continue  # legacy negative index: half-wing mode ignores it
                try:
                    CutLine(
                        CutPoint(cell_nr + has_center, cut["left"], parametric_glider),
                        CutPoint(cell_nr + 1 + has_center, cut["right"], parametric_glider),
                        cut["type"],
                        wing=cut.get("side", "both"),
                    )
                except (TypeError, IndexError) as e:
                    print(f"Design Tool: skipping cut cell {cell_nr}: {e}")
        for l in cls.upper_line_list:
            l.replace_points_by_set()
            l.setup_visuals()
        for l in cls.lower_line_list:
            l.replace_points_by_set()
            l.setup_visuals()

    def _fs_endpoints(self):
        """Full-span endpoints resolved from signed ribs.

        Returns (inner_point, outer_point, cell_nr, wing) or None in half-wing
        mode.  "inner" is always the centre-side rib so that get_dict()'s
        "left" is the centre-side chord position (matching the engine, which
        mirrors the left wing).  The single cross-centre line (a left and a
        right rib) is the centre cell: cell 0, always symmetric.
        """
        r1 = getattr(self.point1, "signed_rib", None)
        r2 = getattr(self.point2, "signed_rib", None)
        if r1 is None or r2 is None:
            return None
        from openglider.glider.parametric.asymmetric import resolve_signed_pair

        has_center = self.parametric_glider.shape.has_center_cell
        inner_is_r1, cell, wing = resolve_signed_pair(r1, r2, has_center)
        inner, outer = (self.point1, self.point2) if inner_is_r1 else (
            self.point2, self.point1)
        return inner, outer, cell, wing

    @property
    def wing(self):
        fs = self._fs_endpoints()
        if fs is not None:
            return fs[3]
        return getattr(self, "_wing", "both")

    @wing.setter
    def wing(self, value):
        self._wing = value

    @property
    def cell_nr(self):
        """Cell index as stored in the cuts dict (matches get_panels cell_no).

        Half-wing mode: cell_nr == inner_point.rib_nr (0 = centre cell for odd
        cell counts).  Full-span mode: derived from the signed ribs.
        """
        fs = self._fs_endpoints()
        if fs is not None:
            return fs[2]
        return self.get_point(inner=True).rib_nr

    def get_point(self, inner=True):
        if (self.point1.rib_nr < self.point2.rib_nr) == inner:
            return self.point1
        else:
            return self.point2

    def get_dict(self):
        fs = self._fs_endpoints()
        if fs is not None:
            inner, outer, cell, wing = fs
        else:
            inner = self.get_point(inner=True)
            outer = self.get_point(inner=False)
            cell = self.cell_nr
            wing = getattr(self, "_wing", "both")
        # "side" always carried here so finalize_cut_lines() never merges cuts
        # from different wings; the neutral "both" marker is stripped there so
        # symmetric files stay byte-for-byte unchanged.
        return {
            "cells": [cell],
            "left": inner.rib_pos,
            "right": outer.rib_pos,
            "type": self.cut_type,
            "side": wing,
        }

    @classmethod
    def get_cut_dict(cls):
        # Delegate merge/collapse/strip to the headless-tested helper so the
        # same rules apply in symmetric and full-span (asymmetric) modes:
        #  - a left+right pair with identical geometry collapses to symmetric,
        #  - contiguous cells merge, and the neutral "both" marker is stripped.
        from openglider.glider.parametric.asymmetric import finalize_cut_lines

        line_dicts = [
            line.get_dict() for line in cls.upper_line_list + cls.lower_line_list
        ]
        return finalize_cut_lines(line_dicts)

    def check_dependency(self):
        if (not self._delete) and (self.point1._delete or self.point2._delete):
            self.delete()

    def delete(self):
        # Check if in list before removing (may already be removed)
        if self.is_upper:
            if self in CutLine.upper_line_list:
                CutLine.upper_line_list.remove(self)
        else:
            if self in CutLine.lower_line_list:
                CutLine.lower_line_list.remove(self)
        super().delete()
