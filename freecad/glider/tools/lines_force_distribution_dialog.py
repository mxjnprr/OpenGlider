"""
Force-distribution dialog for OpenGlider.

Instead of typing the force of every attachment point by hand to balance the
line tensions and the "pattes d'oies", this dialog assigns a *relative* force
to every upper attachment point from an analytical lift distribution:

    force = (span-wise load of the rib station) x (chord-wise share of the point)

The cascade below then balances itself (LineSet.calc_forces sums the branches
of every fork from top to bottom), so only these top forces have to be right.

The heavy lifting is done by ``LineSet2D.distribute_forces`` in
``openglider/glider/parametric/lines.py``; this dialog is only the UI in front
of it.
"""

from PySide import QtGui


class LinesForceDistributionDialog(QtGui.QDialog):
    """Dialog to automatically distribute forces over the attachment points."""

    SPANWISE = [
        ("Elliptical (Cl(y))", "elliptical"),
        ("Proportional to chord (constant Cl)", "chord"),
        ("Uniform (constant load/span)", "uniform"),
    ]
    CHORDWISE = [
        ("Airfoil pressure (front rows carry more)", "airfoil"),
        ("Uniform (tributary only)", "uniform"),
    ]
    NORMALIZE = [
        ("Total load (PTV) -> newtons at 1 g", "load"),
        ("Peak force = 1.0", "max"),
        ("Mean force = 1.0", "mean"),
    ]

    def __init__(self, parametric_glider, parent=None):
        super().__init__(parent)
        self.parametric_glider = parametric_glider
        self.setWindowTitle("Distribute Attachment-Point Forces")
        self.setMinimumWidth(420)
        self.setup_ui()
        self.load_from_glider()

    def setup_ui(self):
        layout = QtGui.QVBoxLayout(self)

        intro = QtGui.QLabel(
            "Assigns a force to every attachment point from a lift distribution: "
            "each point carries the lift of the wing area closest to it "
            "(span-wise and chord-wise). Works with rows of different spacing "
            "(A/B/C every 3rd rib, brakes every 2nd rib), stabilo points... "
            "The forks below balance automatically."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(intro)

        form_group = QtGui.QGroupBox("Distribution")
        form = QtGui.QFormLayout(form_group)

        # Span-wise law
        self.spanwise_combo = QtGui.QComboBox()
        for label, _ in self.SPANWISE:
            self.spanwise_combo.addItem(label)
        self.spanwise_combo.currentIndexChanged.connect(self._update_enabled)
        form.addRow("Span-wise:", self.spanwise_combo)

        # Elliptical exponent
        self.exponent = QtGui.QDoubleSpinBox()
        self.exponent.setRange(0.2, 3.0)
        self.exponent.setSingleStep(0.1)
        self.exponent.setValue(1.0)
        self.exponent.setDecimals(2)
        form.addRow("Ellipse exponent:", self.exponent)
        exp_help = QtGui.QLabel(
            "1.0 = pure ellipse  |  >1 loads the center more  |  <1 loads tips more"
        )
        exp_help.setStyleSheet("color: gray; font-size: 10px;")
        form.addRow("", exp_help)

        # Chord-wise law
        self.chordwise_combo = QtGui.QComboBox()
        for label, _ in self.CHORDWISE:
            self.chordwise_combo.addItem(label)
        form.addRow("Chord-wise:", self.chordwise_combo)

        # Normalisation
        self.normalize_combo = QtGui.QComboBox()
        for label, _ in self.NORMALIZE:
            self.normalize_combo.addItem(label)
        self.normalize_combo.currentIndexChanged.connect(self._update_enabled)
        form.addRow("Normalize:", self.normalize_combo)

        # Total load (all-up weight) for the "load" normalisation
        self.total_load = QtGui.QDoubleSpinBox()
        self.total_load.setRange(10.0, 1000.0)
        self.total_load.setSingleStep(5.0)
        self.total_load.setDecimals(1)
        self.total_load.setValue(100.0)
        self.total_load.setSuffix(" kg")
        form.addRow("Total load (pilot + glider):", self.total_load)
        load_help = QtGui.QLabel(
            "Forces in newtons at 1 g, so the line strength / max-g-force "
            "readout of the Lines tool is meaningful."
        )
        load_help.setWordWrap(True)
        load_help.setStyleSheet("color: gray; font-size: 10px;")
        form.addRow("", load_help)

        layout.addWidget(form_group)

        # Buttons
        button_layout = QtGui.QHBoxLayout()
        button_layout.addStretch()
        cancel_btn = QtGui.QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        apply_btn = QtGui.QPushButton("Apply")
        apply_btn.clicked.connect(self.accept)
        apply_btn.setDefault(True)
        button_layout.addWidget(apply_btn)
        layout.addLayout(button_layout)

        self._update_enabled()

    def _update_enabled(self, *args):
        """Ellipse exponent only matters for the elliptical law, the total
        load only for the "load" normalisation."""
        is_elliptical = self.spanwise_combo.currentIndex() == 0
        self.exponent.setEnabled(is_elliptical)
        is_load = self.NORMALIZE[self.normalize_combo.currentIndex()][1] == "load"
        self.total_load.setEnabled(is_load)

    def load_from_glider(self):
        config = getattr(self.parametric_glider, "force_distribution_config", None)
        if not config:
            return
        spanwise = config.get("spanwise")
        for i, (_, value) in enumerate(self.SPANWISE):
            if value == spanwise:
                self.spanwise_combo.setCurrentIndex(i)
        chordwise = config.get("chordwise")
        for i, (_, value) in enumerate(self.CHORDWISE):
            if value == chordwise:
                self.chordwise_combo.setCurrentIndex(i)
        normalize = config.get("normalize")
        for i, (_, value) in enumerate(self.NORMALIZE):
            if value == normalize:
                self.normalize_combo.setCurrentIndex(i)
        if "spanwise_exponent" in config:
            self.exponent.setValue(config["spanwise_exponent"])
        if config.get("total_load"):
            self.total_load.setValue(config["total_load"])
        self._update_enabled()

    def save_to_glider(self, params):
        self.parametric_glider.force_distribution_config = dict(params)

    def get_parameters(self):
        params = {
            "spanwise": self.SPANWISE[self.spanwise_combo.currentIndex()][1],
            "spanwise_exponent": self.exponent.value(),
            "chordwise": self.CHORDWISE[self.chordwise_combo.currentIndex()][1],
            "normalize": self.NORMALIZE[self.normalize_combo.currentIndex()][1],
            "total_load": self.total_load.value(),
        }
        self.save_to_glider(params)
        return params
