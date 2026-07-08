"""
Unified Aerodynamic Analysis Tool for OpenGlider

This module provides a single tool for aerodynamic analysis combining:
- Settings: Mesh and flight parameters
- Cp View: 3D pressure coefficient visualization with streamlines
- Polars: Flight performance curves (speed polar, sink rate, drag polar)
"""


import logging

import FreeCADGui as Gui
import matplotlib.pyplot as plt
import numpy as np
from pivy.graphics import COLORS, InteractionSeparator, Line, Marker, coin
from PySide import QtCore, QtGui

from openglider.glider.in_out.export_3d import parabem_Panels

from .tools import BaseTool


def rho_isa(altitude_m, temp_sea_level_C=15.0):
    """
    Calculate air density using International Standard Atmosphere (ISA) model.
    
    Args:
        altitude_m: Altitude in meters
        temp_sea_level_C: Sea level temperature in Celsius (default 15°C = ISA standard)
    
    Returns:
        Air density in kg/m³
    """
    T0 = temp_sea_level_C + 273.15  # Convert to Kelvin
    L = 0.0065  # Temperature lapse rate K/m
    T = T0 - L * altitude_m
    if T < 216.65:  # Tropopause limit
        T = 216.65
    return 1.225 * (T / 288.15) ** 4.2559


class AerodynamicTool(BaseTool):
    """
    Unified aerodynamic analysis tool with tabbed interface.
    
    Combines mesh settings, 3D Cp visualization, and polar analysis
    into a single coherent workflow.
    """
    widget_name = "Aerodynamic Analysis"
    hide = True
    
    # Check parabem availability
    try:
        parabem = __import__("parabem")
        pan3d = __import__("parabem.pan3d", globals(), locals(), ["abc"])
        parabem_utils = __import__("parabem.utils", globals(), locals(), ["abc"])
    except ImportError:
        parabem = None
    
    # Check scipy availability
    try:
        from scipy.optimize import newton_krylov
        scipy_available = True
    except ImportError:
        scipy_available = False

    def __init__(self, obj):
        super().__init__(obj)
        
        if not self.parabem:
            self.QWarning = QtGui.QLabel(
                "<b style='color: red'>parabem not installed</b><br>"
                "Install with: <code>pip install parabem</code>"
            )
            self.layout.addWidget(self.QWarning)
        else:
            # Shared state
            self.case = None
            self._panels = None
            self._vertices = None
            self._trailing_edges = None
            self._analysis_complete = False
            
            # Polars data
            self.cL = np.array([])
            self.cDi = np.array([])
            self.cPi = np.array([])
            self.alpha = np.array([])
            
            # 3D visualization
            self.cpc = InteractionSeparator(self.rm)
            self.stream = coin.SoSeparator()
            self.glider_result = coin.SoSeparator()
            self.marker = Marker([[0, 0, 0]], dynamic=True)
            
            self.setup_widget()
            self.setup_pivy()

    def setup_widget(self):
        """Create the tabbed interface."""
        # Create main tab widget
        self.tabs = QtGui.QTabWidget()
        self.layout.addWidget(self.tabs)
        
        # Create tabs
        self.settings_tab = QtGui.QWidget()
        self.cpview_tab = QtGui.QWidget()
        self.polars_tab = QtGui.QWidget()
        
        self.tabs.addTab(self.settings_tab, "⚙ Settings")
        self.tabs.addTab(self.cpview_tab, "🎨 Cp View")
        self.tabs.addTab(self.polars_tab, "📊 Polars")
        
        # Setup each tab
        self._setup_settings_tab()
        self._setup_cpview_tab()
        self._setup_polars_tab()
        
        # Run button at the bottom (outside tabs)
        self.Qrun = QtGui.QPushButton("▶ Run Analysis")
        self.Qrun.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }")
        self.layout.addWidget(self.Qrun)
        
        # Status label
        self.Qstatus = QtGui.QLabel("Status: Ready")
        self.layout.addWidget(self.Qstatus)
        
        # Connect run button
        self.Qrun.clicked.connect(self.run_analysis)
        
        # Initialize displays
        self.update_rho_display()
        self.update_drag_display()

    def _setup_settings_tab(self):
        """Setup the Settings tab with mesh and flight parameters."""
        layout = QtGui.QFormLayout(self.settings_tab)
        
        # === Mesh Settings Group ===
        mesh_group = QtGui.QGroupBox("Mesh Settings")
        mesh_layout = QtGui.QFormLayout(mesh_group)
        
        self.Qprofile_points = QtGui.QSpinBox()
        self.Qprofile_points.setRange(10, 50)
        self.Qprofile_points.setValue(20)
        mesh_layout.addRow("Profile points:", self.Qprofile_points)
        
        self.Qmidribs = QtGui.QSpinBox()
        self.Qmidribs.setRange(0, 5)
        self.Qmidribs.setValue(0)
        mesh_layout.addRow("Midribs per cell:", self.Qmidribs)
        
        self.Qsymmetric = QtGui.QCheckBox()
        self.Qsymmetric.setChecked(True)
        self.Qsymmetric.setToolTip(
            "When checked: analyze only half wing (faster, symmetric assumption).\n"
            "When unchecked: analyze full wing (slower, more accurate).\n\n"
            "⚠️ Click 'Run Analysis' after changing this setting!"
        )
        mesh_layout.addRow("Half model (symmetric):", self.Qsymmetric)
        
        self.Qmean_profile = QtGui.QCheckBox()
        self.Qmean_profile.setChecked(True)
        mesh_layout.addRow("Average profiles:", self.Qmean_profile)
        
        layout.addRow(mesh_group)
        
        # === Solver Settings Group ===
        solver_group = QtGui.QGroupBox("Solver Settings")
        solver_layout = QtGui.QFormLayout(solver_group)
        
        self.Qfarfield = QtGui.QDoubleSpinBox()
        self.Qfarfield.setRange(5, 50)
        self.Qfarfield.setValue(10)
        self.Qfarfield.setToolTip(
            "Farfield radius in wing spans.\n"
            "Defines the outer boundary of the computational domain.\n"
            "Larger values = more accurate but slower.\n"
            "Recommended: 10-20 for accuracy, 5-10 for speed."
        )
        solver_layout.addRow("Farfield radius [spans]:", self.Qfarfield)
        
        layout.addRow(solver_group)
        
        # === Flight Conditions Group ===
        flight_group = QtGui.QGroupBox("Flight Conditions")
        flight_layout = QtGui.QFormLayout(flight_group)
        
        self.Qweight = QtGui.QDoubleSpinBox()
        self.Qweight.setRange(10, 300)
        self.Qweight.setValue(90)
        flight_layout.addRow("Total weight [kg]:", self.Qweight)
        
        self.Qaltitude = QtGui.QDoubleSpinBox()
        self.Qaltitude.setRange(0, 5000)
        self.Qaltitude.setValue(500)
        self.Qaltitude.setSingleStep(100)
        self.Qaltitude.valueChanged.connect(self.update_rho_display)
        flight_layout.addRow("Altitude [m]:", self.Qaltitude)
        
        self.Qrho_display = QtGui.QLabel()
        flight_layout.addRow("Air density ρ [kg/m³]:", self.Qrho_display)
        
        layout.addRow(flight_group)
        
        # === Drag Model Group ===
        drag_group = QtGui.QGroupBox("Drag Model (auto-calculated)")
        drag_layout = QtGui.QFormLayout(drag_group)
        
        self.Qpilot_drag = QtGui.QDoubleSpinBox()
        self.Qpilot_drag.setRange(0, 1.0)
        self.Qpilot_drag.setValue(0.5)
        self.Qpilot_drag.setSingleStep(0.05)
        self.Qpilot_drag.setDecimals(3)
        self.Qpilot_drag.setToolTip(
            "Pilot equivalent flat plate area (Cd × S).\n"
            "Typical values: 0.4-0.6 m² for seated pilot."
        )
        drag_layout.addRow("Pilot drag (Cd·S) [m²]:", self.Qpilot_drag)
        
        # Auto-calculated line drag display
        self.Qline_drag_display = QtGui.QLabel()
        self.Qline_drag_display.setToolTip(
            "Line drag automatically calculated from the\n"
            "glider's line set (diameter, length, Cd)."
        )
        drag_layout.addRow("Line drag (auto):", self.Qline_drag_display)
        
        # Profile drag with xfoil install button
        profile_drag_widget = QtGui.QWidget()
        profile_drag_layout = QtGui.QHBoxLayout(profile_drag_widget)
        profile_drag_layout.setContentsMargins(0, 0, 0, 0)
        
        self.Qprofile_drag_display = QtGui.QLabel()
        self.Qprofile_drag_display.setToolTip(
            "Profile drag calculated by Xfoil (if installed)\n"
            "or estimated from profile thickness."
        )
        profile_drag_layout.addWidget(self.Qprofile_drag_display)
        
        # Install xfoil button (hidden if xfoil binary or package is available)
        self.Qinstall_xfoil = QtGui.QPushButton("ℹ️ Install")
        self.Qinstall_xfoil.setToolTip("Show how to install xfoil")
        self.Qinstall_xfoil.setMaximumWidth(80)
        self.Qinstall_xfoil.clicked.connect(self._show_xfoil_install_help)
        profile_drag_layout.addWidget(self.Qinstall_xfoil)
        
        # Check xfoil button
        self.Qcheck_xfoil = QtGui.QPushButton("🔍 Check")
        self.Qcheck_xfoil.setToolTip("Check if xfoil is installed and working")
        self.Qcheck_xfoil.setMaximumWidth(80)
        self.Qcheck_xfoil.clicked.connect(self._check_xfoil_status)
        profile_drag_layout.addWidget(self.Qcheck_xfoil)
        
        # Check if xfoil is available (package or binary) and update UI
        self._update_xfoil_status_ui()
        
        drag_layout.addRow("Profile drag Cd0:", profile_drag_widget)
        
        layout.addRow(drag_group)
        
        # === Alpha Range Group ===
        alpha_group = QtGui.QGroupBox("Angle of Attack Range")
        alpha_layout = QtGui.QFormLayout(alpha_group)
        
        self.Qalpha_min = QtGui.QDoubleSpinBox()
        self.Qalpha_min.setRange(-10, 20)
        self.Qalpha_min.setValue(2)
        alpha_layout.addRow("Alpha min [°]:", self.Qalpha_min)
        
        self.Qalpha_max = QtGui.QDoubleSpinBox()
        self.Qalpha_max.setRange(10, 45)
        self.Qalpha_max.setValue(30)
        alpha_layout.addRow("Alpha max [°]:", self.Qalpha_max)
        
        self.Qalpha_steps = QtGui.QSpinBox()
        self.Qalpha_steps.setRange(5, 100)
        self.Qalpha_steps.setValue(30)
        alpha_layout.addRow("Number of steps:", self.Qalpha_steps)
        
        layout.addRow(alpha_group)

    def _setup_cpview_tab(self):
        """Setup the Cp View tab for 3D visualization."""
        layout = QtGui.QFormLayout(self.cpview_tab)
        
        # === Colormap Settings ===
        color_group = QtGui.QGroupBox("Pressure Visualization")
        color_layout = QtGui.QFormLayout(color_group)
        
        self.Qcolormap = QtGui.QComboBox()
        self.Qcolormap.addItems(["Red-Yellow-Blue", "Blue-White-Red", "Rainbow"])
        self.Qcolormap.currentIndexChanged.connect(self.show_glider)
        color_layout.addRow("Colormap:", self.Qcolormap)
        
        self.Qmin_val = QtGui.QDoubleSpinBox()
        self.Qmin_val.setRange(-10, 3)
        self.Qmin_val.setValue(-3)
        self.Qmin_val.setSingleStep(0.1)
        self.Qmin_val.valueChanged.connect(self.show_glider)
        color_layout.addRow("Cp min:", self.Qmin_val)
        
        self.Qmax_val = QtGui.QDoubleSpinBox()
        self.Qmax_val.setRange(0, 10)
        self.Qmax_val.setValue(1)
        self.Qmax_val.setSingleStep(0.1)
        self.Qmax_val.valueChanged.connect(self.show_glider)
        color_layout.addRow("Cp max:", self.Qmax_val)
        
        layout.addRow(color_group)
        
        # === Streamline Settings ===
        stream_group = QtGui.QGroupBox("Streamlines")
        stream_layout = QtGui.QFormLayout(stream_group)
        
        self.Qstream_points = QtGui.QSpinBox()
        self.Qstream_points.setRange(1, 30)
        self.Qstream_points.setValue(3)
        self.Qstream_points.valueChanged.connect(self.update_stream)
        stream_layout.addRow("Number of streams:", self.Qstream_points)
        
        self.Qstream_radius = QtGui.QDoubleSpinBox()
        self.Qstream_radius.setRange(0, 2)
        self.Qstream_radius.setValue(0.1)
        self.Qstream_radius.setSingleStep(0.1)
        self.Qstream_radius.valueChanged.connect(self.update_stream)
        stream_layout.addRow("Stream radius [m]:", self.Qstream_radius)
        
        self.Qstream_num = QtGui.QSpinBox()
        self.Qstream_num.setRange(5, 300)
        self.Qstream_num.setValue(70)
        self.Qstream_num.valueChanged.connect(self.update_stream)
        stream_layout.addRow("Points per streamline:", self.Qstream_num)
        
        self.Qstream_interval = QtGui.QDoubleSpinBox()
        self.Qstream_interval.setRange(0.00001, 1.0)
        self.Qstream_interval.setValue(0.02)
        self.Qstream_interval.setSingleStep(0.01)
        self.Qstream_interval.valueChanged.connect(self.update_stream)
        stream_layout.addRow("Stream interval [s]:", self.Qstream_interval)
        
        layout.addRow(stream_group)
        
        # === Animation Controls ===
        anim_group = QtGui.QGroupBox("Alpha Animation")
        anim_layout = QtGui.QFormLayout(anim_group)
        
        # Alpha slider
        self.Qalpha_slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.Qalpha_slider.setRange(0, 100)
        self.Qalpha_slider.setValue(50)
        self.Qalpha_slider.valueChanged.connect(self._on_alpha_slider_changed)
        anim_layout.addRow("Alpha:", self.Qalpha_slider)
        
        # Alpha display
        self.Qalpha_display = QtGui.QLabel("--°")
        anim_layout.addRow("Current angle:", self.Qalpha_display)
        
        # Animation buttons
        anim_btn_widget = QtGui.QWidget()
        anim_btn_layout = QtGui.QHBoxLayout(anim_btn_widget)
        anim_btn_layout.setContentsMargins(0, 0, 0, 0)
        
        self.Qplay_btn = QtGui.QPushButton("▶ Play")
        self.Qplay_btn.clicked.connect(self._toggle_animation)
        anim_btn_layout.addWidget(self.Qplay_btn)
        
        self.Qreset_btn = QtGui.QPushButton("⏮ Reset")
        self.Qreset_btn.clicked.connect(self._reset_animation)
        anim_btn_layout.addWidget(self.Qreset_btn)
        
        anim_layout.addRow("Controls:", anim_btn_widget)
        
        # FPS setting
        self.Qanim_fps = QtGui.QSpinBox()
        self.Qanim_fps.setRange(1, 30)
        self.Qanim_fps.setValue(5)
        self.Qanim_fps.setToolTip("Frames per second for animation")
        anim_layout.addRow("Animation FPS:", self.Qanim_fps)
        
        layout.addRow(anim_group)
        
        # Animation timer
        self.anim_timer = QtCore.QTimer()
        self.anim_timer.timeout.connect(self._animation_step)
        self.is_animating = False
        
        # Info label
        info_label = QtGui.QLabel(
            "<i>Drag the marker in the 3D view to position streamlines.<br>"
            "Run analysis first, then use slider or Play to animate alpha.</i>"
        )
        layout.addRow(info_label)

    def _setup_polars_tab(self):
        """Setup the Polars tab for performance curves."""
        layout = QtGui.QVBoxLayout(self.polars_tab)
        
        # Show Polars button
        self.Qshow_polars = QtGui.QPushButton("📊 Show Polar Plots")
        self.Qshow_polars.clicked.connect(self.show_polars)
        layout.addWidget(self.Qshow_polars)
        
        # Export button
        self.Qexport_csv = QtGui.QPushButton("💾 Export to CSV")
        self.Qexport_csv.clicked.connect(self.export_polars_csv)
        layout.addWidget(self.Qexport_csv)
        
        # Results text area
        self.Qresults_text = QtGui.QTextEdit()
        self.Qresults_text.setReadOnly(True)
        self.Qresults_text.setPlaceholderText("Run analysis to see results...")
        layout.addWidget(self.Qresults_text)
        
        # scipy warning if needed
        if not self.scipy_available:
            warning = QtGui.QLabel(
                "<b style='color: orange'>⚠ scipy not installed</b><br>"
                "Polar plots require scipy. Install with: <code>pip install scipy</code>"
            )
            layout.addWidget(warning)

    def setup_pivy(self):
        """Setup 3D visualization elements."""
        self.cpc.register()
        self.task_separator.addChild(self.cpc)
        self.task_separator.addChild(self.stream)
        self.task_separator.addChild(self.glider_result)
        self.cpc += [self.marker]
        self.marker.on_drag_release.append(self.update_stream)
        self.marker.on_drag.append(self.update_stream_fast)

    def update_rho_display(self):
        """Update the displayed air density based on altitude."""
        rho = rho_isa(self.Qaltitude.value())
        self.Qrho_display.setText(f"<b>{rho:.4f}</b>")

    def update_drag_display(self):
        """Update the auto-calculated drag coefficient displays."""
        area = self.parametric_glider.shape.area
        
        # Line drag from glider model
        try:
            line_drag_raw = self.obj.Proxy.getGliderInstance().lineset.get_normalized_drag()
            cD_lines = line_drag_raw / area * 2
            self.Qline_drag_display.setText(f"<b>{cD_lines:.5f}</b>")
        except Exception:
            self.Qline_drag_display.setText("<i>N/A</i>")
            cD_lines = 0.01
        
        # Profile drag - try to calculate from glider profiles
        CD0_PROFILE, method = self._estimate_profile_drag()
        if method == "xfoil":
            self.Qprofile_drag_display.setText(f"<b>{CD0_PROFILE:.4f}</b> <span style='color:green'>(Xfoil)</span>")
        elif method == "xfoil-bin":
            self.Qprofile_drag_display.setText(f"<b>{CD0_PROFILE:.4f}</b> <span style='color:blue'>(Xfoil bin)</span>")
        else:
            self.Qprofile_drag_display.setText(f"<b>{CD0_PROFILE:.4f}</b> <span style='color:gray'>(t/c est.)</span>")
        return cD_lines, CD0_PROFILE

    def _update_xfoil_status_ui(self):
        """Update UI based on xfoil availability."""
        import shutil
        xfoil_pkg = False
        xfoil_bin = False
        
        try:
            import xfoil
            xfoil_pkg = True
        except ImportError:
            pass
        
        if shutil.which("xfoil"):
            xfoil_bin = True
        
        # Hide install button if available
        if xfoil_pkg or xfoil_bin:
            self.Qinstall_xfoil.hide()
        else:
            self.Qinstall_xfoil.show()

    def _check_xfoil_status(self):
        """Check and display xfoil status."""
        import shutil
        
        status_lines = []
        xfoil_pkg = False
        xfoil_bin_path = None
        
        # Check pip package
        try:
            import xfoil
            xfoil_pkg = True
            status_lines.append("✅ <b>xfoil pip package:</b> Installed")
        except ImportError:
            status_lines.append("❌ <b>xfoil pip package:</b> Not installed")
        
        # Check binary
        xfoil_bin_path = shutil.which("xfoil")
        if xfoil_bin_path:
            status_lines.append(f"✅ <b>xfoil binary:</b> {xfoil_bin_path}")
        else:
            status_lines.append("❌ <b>xfoil binary:</b> Not found in PATH")
        
        # Summary
        if xfoil_pkg or xfoil_bin_path:
            status_lines.append("<br><b style='color:green'>Xfoil is available and will be used for profile drag!</b>")
        else:
            status_lines.append("<br><b style='color:orange'>Xfoil not available - using thickness estimation</b>")
        
        QtGui.QMessageBox.information(
            None,
            "Xfoil Status",
            "<br>".join(status_lines)
        )
        
        # Update UI
        self._update_xfoil_status_ui()

    def _show_xfoil_install_help(self):
        """Show instructions for installing xfoil."""
        QtGui.QMessageBox.information(
            None,
            "Install Xfoil",
            "<b>Install xfoil for accurate profile drag calculation</b><br><br>"
            "<b>Linux (recommended):</b><br>"
            "<code>sudo apt install xfoil</code><br><br>"
            "<b>macOS:</b><br>"
            "<code>brew install xfoil</code><br><br>"
            "<b>Windows:</b><br>"
            "Download from xfoil.com and add to PATH<br><br>"
            "After installation, click 'Check' to verify."
        )

    # === Animation Control Methods ===
    
    def _on_alpha_slider_changed(self, value):
        """Handle alpha slider change - update Cp visualization for this alpha."""
        if not hasattr(self, 'cp_per_alpha') or len(self.cp_per_alpha) == 0:
            self.Qalpha_display.setText("--° (run analysis first)")
            return
        
        # Map slider value (0-100) to array index
        idx = int(value * (len(self.alpha) - 1) / 100)
        idx = max(0, min(idx, len(self.alpha) - 1))
        
        alpha_deg = np.rad2deg(self.alpha[idx])
        cL = self.cL[idx]
        cDi = self.cDi[idx]
        
        # Calculate derived values
        glide_ratio = cL / cDi if cDi > 0.001 else 0
        
        self.Qalpha_display.setText(
            f"<b>{alpha_deg:.1f}°</b> | cL={cL:.3f} | cDi={cDi:.4f} | L/D={glide_ratio:.1f}"
        )
        
        # Store current index
        self._current_alpha_idx = idx
        
        # Update 3D Cp visualization with stored Cp values
        self._update_cp_display(idx)
    
    def _update_case_for_alpha(self, alpha_deg):
        """
        Note: Re-running the solver for each alpha causes crashes in parabem.
        Instead, we display pre-computed polar data.
        """
        pass  # Disabled - use polar data instead
    
    def _toggle_animation(self):
        """Toggle animation play/pause."""
        if self.is_animating:
            self._stop_animation()
        else:
            self._start_animation()
    
    def _start_animation(self):
        """Start alpha animation."""
        if not hasattr(self, 'case') or self.case is None:
            QtGui.QMessageBox.warning(
                None, "Animation", 
                "Run analysis first before animating."
            )
            return
        
        self.is_animating = True
        self.Qplay_btn.setText("⏸ Pause")
        
        # Calculate timer interval from FPS
        fps = self.Qanim_fps.value()
        interval_ms = int(1000 / fps)
        
        self.anim_timer.start(interval_ms)
    
    def _stop_animation(self):
        """Stop alpha animation."""
        self.is_animating = False
        self.Qplay_btn.setText("▶ Play")
        self.anim_timer.stop()
    
    def _animation_step(self):
        """Execute one animation step - advance slider."""
        current = self.Qalpha_slider.value()
        next_val = current + 1
        
        if next_val > 100:
            # Loop back to start or stop
            next_val = 0
        
        self.Qalpha_slider.setValue(next_val)
    
    def _reset_animation(self):
        """Reset animation to start."""
        self._stop_animation()
        self.Qalpha_slider.setValue(0)

    def _update_cp_display(self, alpha_idx):
        """Update 3D visualization with Cp values for given alpha index."""
        if not hasattr(self, 'cp_per_alpha') or alpha_idx >= len(self.cp_per_alpha):
            return
        
        if not hasattr(self, 'glider_result') or self.glider_result is None:
            return
        
        # Get Cp values for this alpha
        cp_values = self.cp_per_alpha[alpha_idx]
        
        # Update the vertex colors in the existing visualization
        self.glider_result.removeAllChildren()
        
        # Get vertices from stored panels
        verts = []
        for panel in self._panels:
            for vert in panel.points:
                if vert.nr >= len(verts):
                    verts.extend([None] * (vert.nr - len(verts) + 1))
                verts[vert.nr] = list(vert)
        
        # Fill in any gaps
        verts = [v if v is not None else [0, 0, 0] for v in verts]
        
        # Build polygons
        pols = []
        pols_i = []
        count = 0
        count_krit = (self.Qmidribs.value() + 1) * (
            self.Qprofile_points.value() - self.Qprofile_points.value() % 2
        )
        for pan in self._panels[::-1]:
            count += 1
            for vert in pan.points:
                pols_i.append(vert.nr)
            pols_i.append(-1)
            if count % count_krit == 0:
                pols.append(pols_i)
                pols_i = []
        if pols_i:
            pols.append(pols_i)
        
        # Create visualization with new colors
        vertex_property = coin.SoVertexProperty()
        for i, col in enumerate(cp_values):
            vertex_property.orderedRGBA.set1Value(
                i, coin.SbColor(self.color(col)).getPackedValue()
            )
        vertex_property.vertex.setValues(0, len(verts), verts)
        vertex_property.materialBinding = coin.SoMaterialBinding.PER_VERTEX_INDEXED
        vertex_property.normalBinding = coin.SoNormalBinding.PER_FACE

        shape_hint = coin.SoShapeHints()
        shape_hint.vertexOrdering = coin.SoShapeHints.COUNTERCLOCKWISE
        shape_hint.creaseAngle = np.pi / 2

        self.glider_result.addChild(shape_hint)

        for cell in pols:
            face_set = coin.SoIndexedFaceSet()
            face_set.vertexProperty = vertex_property
            face_set.coordIndex.setValues(0, len(cell), cell)
            self.glider_result.addChild(face_set)

    def _estimate_profile_drag(self):
        """
        Estimate profile drag coefficient from glider profiles.
        
        Tries in order:
        1. xfoil pip package (fastest, requires 'pip install xfoil')
        2. xfoil binary (requires 'xfoil' in PATH, e.g. 'sudo apt install xfoil')
        3. Empirical estimation from profile thickness
        
        Returns: (cd0_value, method_name)
        """
        # Method 1: Try xfoil pip package
        try:
            import xfoil
            result = self._calculate_xfoil_drag()
            if result is not None:
                return result, "xfoil"
        except ImportError:
            pass
        except Exception as e:
            logging.warning(f"Xfoil package failed: {e}")
        
        # Method 2: Try xfoil binary (OpenGlider's original approach)
        import shutil
        if shutil.which("xfoil"):
            try:
                result = self._calculate_xfoil_binary_drag()
                if result is not None:
                    return result, "xfoil-bin"
            except Exception as e:
                logging.warning(f"Xfoil binary failed: {e}")
        
        # Method 3: Fallback to thickness-based estimation
        return self._estimate_drag_from_thickness(), "thickness"

    def _calculate_xfoil_drag(self):
        """
        Calculate profile drag using the xfoil pip package.
        
        This uses the compiled Fortran library directly via Python bindings,
        which is much faster than calling an external xfoil binary.
        
        Install with: pip install xfoil
        """
        try:
            from xfoil import XFoil
        except ImportError:
            logging.warning("xfoil package not installed. Install with: pip install xfoil")
            return None
        
        profiles = self.parametric_glider.profiles
        if not profiles:
            return 0.008  # Default fallback
        
        # Get representative profile (middle of span)
        profile = profiles[len(profiles) // 2]
        
        # Typical Reynolds number for paraglider
        # Re = V * c / nu, with V ~ 10 m/s, c ~ 2.5m, nu ~ 1.5e-5
        mean_chord = self.parametric_glider.shape.area / self.parametric_glider.shape.span
        velocity = 10  # m/s typical cruise speed
        nu = 1.5e-5  # kinematic viscosity of air at 20°C
        re_number = velocity * mean_chord / nu
        
        # Create Xfoil instance
        xf = XFoil()
        xf.print = False  # Suppress output
        
        # Load airfoil coordinates
        # Profile2D.data gives coordinates as [[x, y], ...]
        coords = profile.data
        if coords is None or len(coords) < 10:
            logging.warning("Invalid profile data for Xfoil")
            return None
        
        # Xfoil expects numpy arrays of x, y
        import numpy as np
        coords_array = np.array(coords)
        xf.airfoil = coords_array
        
        # Set Reynolds number and other parameters
        xf.Re = re_number
        xf.max_iter = 100
        xf.n_crit = 9  # Transition criterion
        
        # Analyze for typical paraglider alpha range
        try:
            a, cl, cd, cm, cp = xf.aseq(4, 10, 1)
            
            # Filter valid results (not NaN)
            valid_mask = ~np.isnan(cd)
            if np.any(valid_mask):
                cd_mean = np.nanmean(cd[valid_mask])
                logging.info(f"Xfoil Cd0 = {cd_mean:.5f} (Re={re_number:.0f}, profile={profile.name})")
                return cd_mean
            else:
                logging.warning("Xfoil returned no valid results")
                return None
        except Exception as e:
            logging.warning(f"Xfoil analysis failed: {e}")
            return None

    def _calculate_xfoil_binary_drag(self):
        """
        Calculate profile drag using the xfoil binary directly.
        
        This version doesn't require pandas - parses xfoil output directly.
        Requires 'xfoil' to be installed and in PATH (e.g. 'sudo apt install xfoil').
        """
        import os
        import re
        import subprocess
        import tempfile

        import numpy as np
        
        try:
            import FreeCAD
            FreeCAD.Console.PrintMessage("Xfoil: Starting binary calculation...\n")
        except:
            pass
        
        profiles = self.parametric_glider.profiles
        if not profiles:
            logging.warning("No profiles available for xfoil")
            return None
        
        # Get representative profile
        profile = profiles[len(profiles) // 2]
        
        try:
            import FreeCAD
            FreeCAD.Console.PrintMessage(f"Xfoil: Using profile '{getattr(profile, 'name', 'unnamed')}'\n")
        except:
            pass
        
        # Calculate Reynolds number
        mean_chord = self.parametric_glider.shape.area / self.parametric_glider.shape.span
        velocity = 10  # m/s
        nu = 1.5e-5
        re_number = velocity * mean_chord / nu
        
        try:
            import FreeCAD
            FreeCAD.Console.PrintMessage(f"Xfoil: Running analysis at Re={re_number:.0f}...\n")
        except:
            pass
        
        with tempfile.TemporaryDirectory() as tempdir:
            # Write airfoil data file
            airfoil_file = os.path.join(tempdir, "airfoil.dat")
            result_file = os.path.join(tempdir, "results.dat")
            cmd_file = os.path.join(tempdir, "xfoil_cmd.txt")
            
            # Export airfoil coordinates
            try:
                profile.export_dat(airfoil_file)
            except:
                # Manual export if method doesn't exist
                with open(airfoil_file, 'w') as f:
                    f.write(f"{getattr(profile, 'name', 'airfoil')}\n")
                    for x, y in profile.data:
                        f.write(f"  {x:.6f}  {y:.6f}\n")
            
            # Create xfoil command file
            alphas = [4, 5, 6, 7, 8]  # Typical paraglider range
            alpha_cmds = "\n".join([f"Alfa\n{a}" for a in alphas])
            
            xfoil_commands = f"""PLOP
g

LOAD {airfoil_file}

CADD
PANEL

OPER
VISC {re_number:.0f}
VPAR
n
9
xtr
0.5
0.5

PACC
{result_file}

{alpha_cmds}

quit
"""
            with open(cmd_file, 'w') as f:
                f.write(xfoil_commands)
            
            # Run xfoil
            try:
                result = subprocess.run(
                    f"xfoil < {cmd_file}",
                    shell=True,
                    cwd=tempdir,
                    capture_output=True,
                    text=True,
                    timeout=30
                )
            except subprocess.TimeoutExpired:
                try:
                    import FreeCAD
                    FreeCAD.Console.PrintWarning("Xfoil: Timeout after 30s\n")
                except:
                    pass
                return None
            except Exception as e:
                try:
                    import FreeCAD
                    FreeCAD.Console.PrintError(f"Xfoil: Subprocess error: {e}\n")
                except:
                    pass
                return None
            
            # Parse results file
            if not os.path.exists(result_file):
                try:
                    import FreeCAD
                    FreeCAD.Console.PrintWarning("Xfoil: No results file generated\n")
                except:
                    pass
                return None
            
            # Read and parse results
            cd_values = []
            rex_number = r"([+-]?\d+\.?\d*)"
            rex_line = re.compile(r"\s+" + r"\s+".join([rex_number] * 9))
            
            with open(result_file) as f:
                for line in f:
                    match = rex_line.match(line)
                    if match:
                        # Values: alpha, CL, CD, CDp, CM, Top_Xtr, Bot_Xtr, Top_Itr, Bot_Itr
                        cd = float(match.group(3))  # CD is 3rd column
                        cd_values.append(cd)
            
            if cd_values:
                cd_mean = np.mean(cd_values)
                try:
                    import FreeCAD
                    FreeCAD.Console.PrintMessage(f"Xfoil: SUCCESS! Cd0 = {cd_mean:.5f} (from {len(cd_values)} points)\n")
                except:
                    pass
                return cd_mean
            else:
                try:
                    import FreeCAD
                    FreeCAD.Console.PrintWarning("Xfoil: No valid CD values found in results\n")
                except:
                    pass
                return None

    def _estimate_drag_from_thickness(self):
        """
        Estimate profile drag from profile thickness.
        
        Empirical relation for paraglider profiles:
        Cd0 ≈ 0.006 + 0.05 * (t/c)²
        
        where t/c is the thickness ratio (typically 15-20% for paragliders)
        """
        profiles = self.parametric_glider.profiles
        if not profiles:
            return 0.008  # Default
        
        # Calculate average thickness ratio
        thickness_sum = 0
        for profile in profiles:
            try:
                thickness_sum += profile.thickness
            except:
                thickness_sum += 0.17  # Default 17% thickness
        
        avg_thickness = thickness_sum / len(profiles)
        
        # Empirical formula
        cd0 = 0.006 + 0.05 * avg_thickness ** 2
        
        logging.info(f"Profile drag estimated from thickness: Cd0 = {cd0:.5f} (t/c = {avg_thickness:.1%})")
        return cd0

    def run_analysis(self):
        """Run the complete aerodynamic analysis."""
        self.Qstatus.setText("Status: Running analysis...")
        self.Qrun.setEnabled(False)
        QtGui.QApplication.processEvents()
        
        try:
            # Step 1: Create panels
            self.Qstatus.setText("Status: Creating mesh...")
            QtGui.QApplication.processEvents()
            self._create_panels()
            
            # Step 2: Run single-point analysis for Cp visualization
            self.Qstatus.setText("Status: Running panel method...")
            QtGui.QApplication.processEvents()
            self._run_single_point()
            
            # Step 3: Compute polars (multi-point)
            self.Qstatus.setText("Status: Computing polars...")
            QtGui.QApplication.processEvents()
            self._compute_polars()
            
            # Step 4: Show results
            self.show_glider()
            self._update_results_text()
            
            self._analysis_complete = True
            self.Qstatus.setText("Status: Analysis complete ✓")
            
            # Switch to Cp View tab
            self.tabs.setCurrentIndex(1)
            
        except Exception as e:
            self.Qstatus.setText(f"Status: Error - {str(e)}")
            logging.error(f"Analysis failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.Qrun.setEnabled(True)

    def _create_panels(self):
        """Create the panel mesh."""
        self._vertices, self._panels, self._trailing_edges, _ = parabem_Panels(
            self.parametric_glider.get_glider_3d(),
            midribs=self.Qmidribs.value(),
            profile_numpoints=self.Qprofile_points.value(),
            num_average=self.Qmean_profile.isChecked() * 5,
            symmetric=self.Qsymmetric.isChecked(),
        )

    def _run_single_point(self):
        """Run single-point analysis for Cp visualization."""
        if self.case is not None:
            del self.case
        
        self.case = self.pan3d.DirichletDoublet0Source0Case3(
            self._panels, self._trailing_edges
        )
        self.case.v_inf = self.parabem.Vector(self.parametric_glider.v_inf)
        self.case.farfield = self.Qfarfield.value()
        
        # Wake length based on mean chord
        mean_chord = self.parametric_glider.shape.area / self.parametric_glider.shape.span
        wake_length = int(100 * mean_chord)
        self.case.create_wake(max(wake_length, 100), 10)
        
        self.case.run()

    def _compute_polars(self):
        """Compute polar curves over alpha range and store Cp for animation."""
        # Create a fresh case for polars
        case = self.pan3d.DirichletDoublet0Source0Case3(
            self._panels, self._trailing_edges
        )
        case.A_ref = self.parametric_glider.shape.area
        case.v_inf = self.parabem.Vector(self.parametric_glider.v_inf)
        case.drag_calc = "trefftz"
        case.farfield = self.Qfarfield.value()
        case.create_wake(10000000, 20)
        
        # Get v_inf vectors for alpha range
        v_inf_list = self.parabem_utils.v_inf_deg_range3(
            case.v_inf,
            self.Qalpha_min.value(),
            self.Qalpha_max.value(),
            self.Qalpha_steps.value()
        )
        
        # Compute polars and store Cp for each alpha
        self.cL = []
        self.cDi = []
        self.cPi = []
        self.alpha = []
        self.cp_per_alpha = []  # Store Cp arrays for animation
        self.cop_x_per_alpha = []  # Center of pressure X position (central)
        self.cop_x_global = []  # Center of pressure X position (global)
        
        try:
            import FreeCAD
            FreeCAD.Console.PrintMessage(f"Computing {len(v_inf_list)} alpha points for animation...\n")
        except:
            pass
        
        for i, v_inf in enumerate(v_inf_list):
            # Create fresh case for each alpha to avoid memory issues
            single_case = self.pan3d.DirichletDoublet0Source0Case3(
                self._panels, self._trailing_edges
            )
            single_case.A_ref = self.parametric_glider.shape.area
            single_case.v_inf = v_inf
            single_case.drag_calc = "trefftz"
            single_case.farfield = self.Qfarfield.value()
            single_case.create_wake(10000000, 20)
            
            single_case.run()
            
            # Calculate alpha from v_inf vector 
            # v_inf typically points in negative x direction for forward flight
            # alpha = angle from horizontal (xz plane)
            v = [v_inf[0], v_inf[1], v_inf[2]]
            v_mag = np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
            # alpha is the angle between v_inf and the negative x-axis (horizontal flight)
            # For typical paraglider: v_inf points roughly in -x direction with small -z component
            alpha_rad = np.arctan2(v[2], v[0]) if v_mag > 0 else 0
            
            # Extract polar data
            self.alpha.append(alpha_rad)
            self.cL.append(single_case.cL)
            self.cDi.append(single_case.cD)
            self.cPi.append(0)  # cP not available on single case
            
            # Store Cp values for animation
            cp_values = [vert.cp for vert in single_case.vertices]
            self.cp_per_alpha.append(cp_values)
            
            # Calculate center of pressure X position (from panels)
            try:
                # Global CoP (all panels)
                sum_cp_x_global = 0.0
                sum_cp_global = 0.0
                # Central CoP (panels near y=0)
                sum_cp_x_central = 0.0
                sum_cp_central = 0.0
                
                vertices = single_case.vertices
                
                for panel in self._panels:
                    # Panel centroid from panel center property if available
                    try:
                        center = panel.center
                        centroid_x = center[0]
                        centroid_y = center[1]
                    except:
                        # Fallback: calculate from points
                        pts = [list(p) for p in panel.points]
                        centroid_x = sum(p[0] for p in pts) / len(pts)
                        centroid_y = sum(p[1] for p in pts) / len(pts)
                    
                    # Panel Cp - use panel.cp if available, otherwise average from vertices
                    try:
                        panel_cp = panel.cp
                    except:
                        try:
                            panel_cp = sum(vertices[p.nr].cp for p in panel.points) / len(panel.points)
                        except:
                            continue
                    
                    # Weight by pressure (negative Cp = suction = lift on upper surface)
                    # Use absolute value of Cp as weight (stronger pressure = more contribution)
                    weight = abs(panel_cp)
                    
                    # Global: all panels
                    sum_cp_x_global += centroid_x * weight
                    sum_cp_global += weight
                    
                    # Central: only panels near y=0 (within 1m for half-model)
                    if abs(centroid_y) < 1.0:
                        sum_cp_x_central += centroid_x * weight
                        sum_cp_central += weight
                
                cop_x_global = sum_cp_x_global / sum_cp_global if sum_cp_global > 0 else 0
                cop_x_central = sum_cp_x_central / sum_cp_central if sum_cp_central > 0 else 0
                
                self.cop_x_global.append(cop_x_global)
                self.cop_x_per_alpha.append(cop_x_central)
                
            except Exception as e:
                import FreeCAD
                FreeCAD.Console.PrintWarning(f"CoP calculation error: {e}\n")
                self.cop_x_global.append(0)
                self.cop_x_per_alpha.append(0)
            
            del single_case
        
        self.alpha = np.array(self.alpha)
        self.cL = np.array(self.cL)
        self.cDi = np.array(self.cDi)
        self.cPi = np.array(self.cPi)
        self.cop_x_per_alpha = np.array(self.cop_x_per_alpha)
        self.cop_x_global = np.array(self.cop_x_global)
        
        try:
            import FreeCAD
            FreeCAD.Console.PrintMessage(f"Polar computation complete with {len(self.cp_per_alpha)} Cp frames.\n")
        except:
            pass

    def _update_results_text(self):
        """Update the results text in the Polars tab."""
        if len(self.cL) == 0:
            return
        
        # Calculate key performance metrics
        area = self.parametric_glider.shape.area
        rho = rho_isa(self.Qaltitude.value())
        mass = self.Qweight.value()
        g = 9.81
        
        # Drag model - auto-calculated
        CD0_PROFILE = 0.008  # Typical paraglider airfoil Cd0
        cD_parasitic = CD0_PROFILE + self.Qpilot_drag.value() / area
        try:
            cD_lines = self.obj.Proxy.getGliderInstance().lineset.get_normalized_drag() / area * 2
        except:
            cD_lines = 0.01
        
        cD_total = self.cDi + cD_parasitic + cD_lines
        
        # Filter valid data points (cL > 0 to avoid division/sqrt issues)
        valid_mask = self.cL > 0.01
        if not np.any(valid_mask):
            self.Qresults_text.setHtml("<b style='color:red'>Error: No valid data points (all cL <= 0)</b>")
            return
        
        cL_valid = self.cL[valid_mask]
        cD_total_valid = cD_total[valid_mask]
        alpha_valid = self.alpha[valid_mask]
        
        gamma = np.arctan(cD_total_valid / cL_valid)
        glide_ratio = cL_valid / cD_total_valid
        
        # Ensure valid sqrt argument
        sqrt_arg = 2 * mass * g * np.cos(gamma) / (rho * area * cL_valid)
        sqrt_arg = np.maximum(sqrt_arg, 0)  # Avoid negative values
        velocity = np.sqrt(sqrt_arg)
        sink_rate = velocity * np.sin(gamma)
        
        # Best glide
        best_idx = np.argmax(glide_ratio)
        # Min sink
        min_sink_idx = np.argmin(sink_rate)
        
        # Store results for other tools (e.g., Lines Auto-placement)
        if hasattr(self, 'cop_x_per_alpha') and len(self.cop_x_per_alpha) > 0:
            try:
                glider_3d = self.parametric_glider.get_glider_3d()
                central_rib = glider_3d.ribs[0] if glider_3d.has_center_cell else glider_3d.ribs[len(glider_3d.ribs)//2]
                le_x = central_rib.pos[0]
                chord = central_rib.chord
                
                # CoP as percentage of central chord (positive backwards from LE)
                cop_central_pct = ((self.cop_x_per_alpha[best_idx] - le_x) / chord) * 100
                cop_global_pct = ((self.cop_x_global[best_idx] - le_x) / chord) * 100 if hasattr(self, 'cop_x_global') else cop_central_pct
                
                self.parametric_glider.aerodynamics_results = {
                    'best_ld_alpha': np.rad2deg(self.alpha[best_idx]),
                    'cop_central_pct': cop_central_pct,
                    'cop_global_pct': cop_global_pct
                }
            except Exception as e:
                import FreeCAD
                FreeCAD.Console.PrintWarning(f"Failed to store aerodynamic CoP results: {e}\n")

        text = f"""<h3>Analysis Results</h3>
<b>Configuration:</b>
• Weight: {mass} kg
• Altitude: {self.Qaltitude.value()} m
• Air density: {rho:.4f} kg/m³
• Wing area: {area:.1f} m²

<b>Best Glide (max L/D):</b>
• L/D: <b>{glide_ratio[best_idx]:.1f}</b>
• Speed: {velocity[best_idx]*3.6:.1f} km/h
• Sink rate: {sink_rate[best_idx]:.2f} m/s
• Alpha: {np.rad2deg(self.alpha[best_idx]):.1f}°

<b>Minimum Sink:</b>
• L/D: {glide_ratio[min_sink_idx]:.1f}
• Speed: {velocity[min_sink_idx]*3.6:.1f} km/h
• Sink rate: <b>{sink_rate[min_sink_idx]:.2f} m/s</b>
• Alpha: {np.rad2deg(self.alpha[min_sink_idx]):.1f}°

<b>Speed Range:</b>
• Min: {velocity.min()*3.6:.1f} km/h
• Max: {velocity.max()*3.6:.1f} km/h
"""
        self.Qresults_text.setHtml(text)

    def show_polars(self):
        """Show the polar plots in a matplotlib window."""
        if len(self.cL) == 0:
            QtGui.QMessageBox.warning(
                None,
                "No Data",
                "Run the analysis first to generate polar data."
            )
            return
        
        # Calculate performance metrics
        area = self.parametric_glider.shape.area
        rho = rho_isa(self.Qaltitude.value())
        mass = self.Qweight.value()
        g = 9.81
        
        # Drag model - auto-calculated
        CD0_PROFILE = 0.008  # Typical paraglider airfoil Cd0
        cD_parasitic = CD0_PROFILE + self.Qpilot_drag.value() / area
        try:
            cD_lines = self.obj.Proxy.getGliderInstance().lineset.get_normalized_drag() / area * 2
        except:
            cD_lines = 0.01
        
        cD_total = self.cDi + cD_parasitic + cD_lines
        
        # Filter valid data points
        valid_mask = self.cL > 0.01
        if not np.any(valid_mask):
            QtGui.QMessageBox.warning(None, "Error", "No valid data points (all cL <= 0)")
            return
        
        cL_valid = self.cL[valid_mask]
        cDi_valid = self.cDi[valid_mask]
        cD_total_valid = cD_total[valid_mask]
        
        gamma = np.arctan(cD_total_valid / cL_valid)
        glide_ratio = cL_valid / cD_total_valid
        sqrt_arg = 2 * mass * g * np.cos(gamma) / (rho * area * cL_valid)
        sqrt_arg = np.maximum(sqrt_arg, 0)
        velocity = np.sqrt(sqrt_arg)
        sink_rate = velocity * np.sin(gamma)
        
        # Find optimal points
        best_idx = np.argmax(glide_ratio)
        min_sink_idx = np.argmin(sink_rate)
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(
            f"Polars - Mass: {mass}kg, Altitude: {self.Qaltitude.value()}m, ρ: {rho:.3f} kg/m³",
            fontsize=14
        )
        
        # Plot 1: Speed Polar
        ax1 = axes[0, 0]
        ax1.plot(velocity * 3.6, glide_ratio, 'b-', linewidth=2)
        ax1.plot(
            velocity[best_idx] * 3.6, glide_ratio[best_idx], 'go',
            markersize=10, 
            label=f"Best L/D: {glide_ratio[best_idx]:.1f} @ {velocity[best_idx]*3.6:.1f} km/h"
        )
        ax1.set_xlabel("Velocity [km/h]")
        ax1.set_ylabel("Glide Ratio (L/D)")
        ax1.set_title("Speed Polar")
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot 2: Sink Rate
        ax2 = axes[0, 1]
        ax2.plot(velocity * 3.6, sink_rate, 'r-', linewidth=2)
        ax2.plot(
            velocity[min_sink_idx] * 3.6, sink_rate[min_sink_idx], 'go',
            markersize=10,
            label=f"Min sink: {sink_rate[min_sink_idx]:.2f} m/s @ {velocity[min_sink_idx]*3.6:.1f} km/h"
        )
        ax2.plot(
            velocity[best_idx] * 3.6, sink_rate[best_idx], 'bo',
            markersize=10,
            label=f"Best L/D: {sink_rate[best_idx]:.2f} m/s"
        )
        ax2.set_xlabel("Velocity [km/h]")
        ax2.set_ylabel("Sink Rate [m/s]")
        ax2.set_title("Sink Rate Polar")
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.invert_yaxis()
        
        # Plot 3: Drag Polar
        ax3 = axes[1, 0]
        ax3.plot(cD_total_valid, cL_valid, 'b-', linewidth=2, label='Total')
        ax3.plot(cDi_valid, cL_valid, 'g--', linewidth=1, label='Induced only')
        ax3.set_xlabel("CD")
        ax3.set_ylabel("CL")
        ax3.set_title("Drag Polar")
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # Plot 4: Center of Pressure vs Alpha
        ax4 = axes[1, 1]
        if hasattr(self, 'cop_x_per_alpha') and len(self.cop_x_per_alpha) > 0:
            alpha_deg = np.rad2deg(self.alpha)
            
            # Get leading edge reference (central cell)
            try:
                glider_3d = self.parametric_glider.get_glider_3d()
                central_rib = glider_3d.ribs[0] if glider_3d.has_center_cell else glider_3d.ribs[len(glider_3d.ribs)//2]
                le_x = central_rib.pos[0]  # Leading edge X position
                chord = central_rib.chord
                
                # Distance from leading edge as % of central chord
                cop_central = ((self.cop_x_per_alpha - le_x) / chord) * 100  # %
                cop_global = ((self.cop_x_global - le_x) / chord) * 100 if hasattr(self, 'cop_x_global') else cop_central
                
                # Plot both
                ax4.plot(alpha_deg, cop_global, 'b-', linewidth=2, label='Average (full wing)')
                ax4.plot(alpha_deg, cop_central, 'r-', linewidth=2, label='Central (y≈0)')
                
                # Mark best glide point
                ax4.plot(alpha_deg[best_idx], cop_central[best_idx], 'go',
                    markersize=10,
                    label=f"Best L/D: {cop_central[best_idx]:.1f}% (α={alpha_deg[best_idx]:.1f}°)")
                
                ax4.set_xlabel("Angle of Attack α [°]")
                ax4.set_ylabel("Position from LE [% chord]")
                ax4.set_title("Center of Pressure Position")
                ax4.grid(True, alpha=0.3)
                ax4.legend()
                
            except Exception as e:
                ax4.text(0.5, 0.5, f"Error: {str(e)}", transform=ax4.transAxes, ha='center')
        else:
            ax4.text(0.5, 0.5, "No CoP data available", transform=ax4.transAxes, ha='center')
            ax4.set_title("Center of Pressure")
        
        plt.tight_layout()
        plt.show()

    def export_polars_csv(self):
        """Export polar data to CSV file."""
        if len(self.cL) == 0:
            QtGui.QMessageBox.warning(
                None,
                "No Data",
                "Run the analysis first to generate polar data."
            )
            return
        
        file_name = QtGui.QFileDialog.getSaveFileName(
            parent=None,
            caption="Export Polars to CSV",
            filter="CSV files (*.csv)"
        )
        
        if not file_name[0]:
            return
        
        # Calculate metrics
        area = self.parametric_glider.shape.area
        rho = rho_isa(self.Qaltitude.value())
        mass = self.Qweight.value()
        g = 9.81
        
        # Drag model - auto-calculated
        CD0_PROFILE = 0.008
        cD_parasitic = CD0_PROFILE + self.Qpilot_drag.value() / area
        try:
            cD_lines = self.obj.Proxy.getGliderInstance().lineset.get_normalized_drag() / area * 2
        except:
            cD_lines = 0.01
        
        cD_total = self.cDi + cD_parasitic + cD_lines
        
        # Filter valid data
        valid_mask = self.cL > 0.01
        cL_valid = self.cL[valid_mask]
        cDi_valid = self.cDi[valid_mask]
        cD_total_valid = cD_total[valid_mask]
        alpha_valid = self.alpha[valid_mask]
        
        gamma = np.arctan(cD_total_valid / cL_valid)
        glide_ratio = cL_valid / cD_total_valid
        sqrt_arg = 2 * mass * g * np.cos(gamma) / (rho * area * cL_valid)
        sqrt_arg = np.maximum(sqrt_arg, 0)
        velocity = np.sqrt(sqrt_arg)
        sink_rate = velocity * np.sin(gamma)
        
        # Write CSV
        with open(file_name[0], 'w') as f:
            f.write("# OpenGlider Polar Export\n")
            f.write(f"# Mass: {mass} kg, Altitude: {self.Qaltitude.value()} m, Rho: {rho:.4f} kg/m³\n")
            f.write("alpha_deg,CL,CDi,CD_total,L/D,velocity_kmh,sink_rate_ms,gamma_deg\n")
            for i in range(len(self.alpha)):
                f.write(f"{np.rad2deg(self.alpha[i]):.2f},{self.cL[i]:.4f},{self.cDi[i]:.5f},"
                       f"{cD_total[i]:.5f},{glide_ratio[i]:.2f},{velocity[i]*3.6:.1f},"
                       f"{sink_rate[i]:.3f},{np.rad2deg(gamma[i]):.2f}\n")
        
        QtGui.QMessageBox.information(
            None,
            "Export Complete",
            f"Polar data exported to:\n{file_name[0]}"
        )

    def show_glider(self):
        """Display the 3D Cp visualization."""
        if self.case is None:
            return
        
        self.glider_result.removeAllChildren()
        verts = [list(i) for i in self.case.vertices]
        cols = [i.cp for i in self.case.vertices]
        pols = []
        pols_i = []
        count = 0
        count_krit = (self.Qmidribs.value() + 1) * (
            self.Qprofile_points.value() - self.Qprofile_points.value() % 2
        )
        for pan in self._panels[::-1]:
            count += 1
            for vert in pan.points:
                pols_i.append(vert.nr)
            pols_i.append(-1)  # end of polygon
            if count % count_krit == 0:
                pols.append(pols_i)
                pols_i = []
        if pols_i:
            pols.append(pols_i)
        
        vertex_property = coin.SoVertexProperty()
        for i, col in enumerate(cols):
            vertex_property.orderedRGBA.set1Value(
                i, coin.SbColor(self.color(col)).getPackedValue()
            )
        vertex_property.vertex.setValues(0, len(verts), verts)
        vertex_property.materialBinding = coin.SoMaterialBinding.PER_VERTEX_INDEXED
        vertex_property.normalBinding = coin.SoNormalBinding.PER_FACE

        shape_hint = coin.SoShapeHints()
        shape_hint.vertexOrdering = coin.SoShapeHints.COUNTERCLOCKWISE
        shape_hint.creaseAngle = np.pi / 2
        self.glider_result.addChild(shape_hint)
        self.glider_result.addChild(vertex_property)
        
        for panels in pols:
            face_set = coin.SoIndexedFaceSet()
            face_set.coordIndex.setValues(0, len(panels), panels)
            self.glider_result.addChild(face_set)

        # Add force vector
        p1 = np.array(self.case.center_of_pressure)
        f = np.array(self.case.force)
        line = Line([p1, p1 + f])
        self.glider_result.addChild(line)

    def color(self, value):
        """Map a Cp value to an RGB color."""
        max_val = self.Qmax_val.value()
        min_val = self.Qmin_val.value()
        
        if max_val == min_val:
            norm_val = 0.5
        else:
            norm_val = np.clip((value - min_val) / (max_val - min_val), 0, 1)
        
        colormap_name = self.Qcolormap.currentText()
        
        if colormap_name == "Red-Yellow-Blue":
            red = np.array(COLORS["red"])
            blue = np.array(COLORS["blue"])
            yellow = np.array(COLORS["yellow"])
            white = np.array(COLORS["white"])
            
            if norm_val < 0.25:
                t = norm_val / 0.25
                color = red * (1 - t) + yellow * t
            elif norm_val < 0.5:
                t = (norm_val - 0.25) / 0.25
                color = yellow * (1 - t) + white * t
            elif norm_val < 0.75:
                t = (norm_val - 0.5) / 0.25
                color = white * (1 - t) + blue * t
            else:
                color = blue
                
        elif colormap_name == "Blue-White-Red":
            blue = np.array([0.0, 0.0, 1.0])
            white = np.array([1.0, 1.0, 1.0])
            red = np.array([1.0, 0.0, 0.0])
            
            if norm_val < 0.5:
                t = norm_val / 0.5
                color = blue * (1 - t) + white * t
            else:
                t = (norm_val - 0.5) / 0.5
                color = white * (1 - t) + red * t
                
        else:  # Rainbow
            if norm_val < 0.25:
                t = norm_val / 0.25
                color = np.array([0, t, 1])
            elif norm_val < 0.5:
                t = (norm_val - 0.25) / 0.25
                color = np.array([0, 1, 1 - t])
            elif norm_val < 0.75:
                t = (norm_val - 0.5) / 0.25
                color = np.array([t, 1, 0])
            else:
                t = (norm_val - 0.75) / 0.25
                color = np.array([1, 1 - t, 0])
        
        return list(color)

    def update_stream(self):
        """Update streamlines visualization."""
        self.stream.removeAllChildren()
        if self.case:
            point = list(self.marker.points[0].getValue())
            points = np.random.random((self.Qstream_points.value(), 3)) - np.array([0.5, 0.5, 0.5])
            points *= self.Qstream_radius.value()
            points += np.array(point)
            points = points.tolist()
            for p in points:
                pts = self.stream_line(
                    p, self.Qstream_interval.value(), self.Qstream_num.value()
                )
                self.stream.addChild(Line(pts, dynamic=False))

    def update_stream_fast(self):
        """Fast streamline update during drag."""
        self.stream.removeAllChildren()
        if self.case:
            point = list(self.marker.points[0].getValue())
            pts = self.stream_line(point, 0.05, 10)
            self.stream.addChild(Line(pts, dynamic=False))

    def stream_line(self, point, interval, numpoints):
        """Calculate a single streamline."""
        flow_path = self.case.flow_path(
            self.parabem.Vector3(*point), interval, numpoints
        )
        return [[p.x, p.y, p.z] for p in flow_path]

    def accept(self):
        """Accept and close dialog - persist aerodynamic results."""
        # Save aerodynamic results to the parametric glider
        self.obj.Proxy.setParametricGlider(self.parametric_glider)
        self.cleanup()
        Gui.Control.closeDialog()

    def reject(self):
        """Reject and close dialog."""
        self.cleanup()
        Gui.Control.closeDialog()

    def cleanup(self):
        """Clean up resources."""
        self.stream.removeAllChildren()
        self.glider_result.removeAllChildren()
        if self.case is not None:
            del self.case
            self.case = None


# Keep legacy classes for backwards compatibility
class Polars(AerodynamicTool):
    """Legacy Polars class - redirects to AerodynamicTool."""
    widget_name = "Aerodynamic Analysis (Polars)"
    hide = False


class PanelTool(AerodynamicTool):
    """Legacy PanelTool class - redirects to AerodynamicTool."""
    widget_name = "Aerodynamic Analysis (Panel)"
    hide = True
