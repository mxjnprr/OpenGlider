#! /usr/bin/python
"""
LE Panel Split Tool - halve the leading-edge panel of chosen cells spanwise.

The leading-edge (LE) panel of a cell is the extrados panel bounded by the
design's LE cut (drawn in the Design tool) and the air intake.  For every
ticked cell that panel is split into two half-panels (_L / _R) with a seam
at mid-cell.  The tool never creates cuts: the LE cut must already exist.

Storage: ParametricGlider.elements["le_panel_splits"], read and written
through ParametricGlider.le_split_cells() / set_le_split_cells().
"""

from pivy import coin
from PySide import QtCore, QtGui

from .glider import draw_glider
from .tools import BaseTool

# Preview colours of the two half-panels (inner half / outer half).
SPLIT_COLOR_INNER = "#ff7f0e"
SPLIT_COLOR_OUTER = "#ffd92f"

CHECKABLE = (
    QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsUserCheckable
)


class LEPanelSplitTool(BaseTool):
    """One checkbox per half-cell: ticked = the LE panel of that cell is split."""

    hide = True
    turn = False
    widget_name = "LE Panel Split"

    def __init__(self, obj):
        super().__init__(obj)
        self.num_cells = self.parametric_glider.shape.half_cell_num
        self._dropped = set()  # stored splits that cannot apply (see _fill_cells_list)

        description = QtGui.QLabel(
            "Fractionne le panneau de bord d'attaque (extrados, entre la coupe BA "
            "et l'entrée d'air) en deux moitiés avec une couture à mi-cellule.\n"
            "La coupe BA doit exister dans l'outil Design : aucune coupe n'est "
            "créée ici.\n"
            "Cellules numérotées de 1 (centre) à N (bout d'aile). Positions en % "
            "de corde : négatif = extrados, positif = intrados.\n"
            "Preview : moitié interne en orange, moitié externe en jaune."
        )
        description.setWordWrap(True)
        self.layout.addRow(description)

        self.cells_list = QtGui.QListWidget()
        self.cells_list.setMinimumHeight(220)
        self.layout.addRow(self.cells_list)

        range_widget = QtGui.QWidget()
        range_layout = QtGui.QHBoxLayout(range_widget)
        range_layout.setContentsMargins(0, 0, 0, 0)
        self.range_start = QtGui.QSpinBox()
        self.range_end = QtGui.QSpinBox()
        for spin in (self.range_start, self.range_end):
            spin.setRange(1, max(1, self.num_cells))
            spin.setValue(self.num_cells)
        self.check_range_button = QtGui.QPushButton("Cocher")
        self.uncheck_range_button = QtGui.QPushButton("Décocher")
        range_layout.addWidget(QtGui.QLabel("de C"))
        range_layout.addWidget(self.range_start)
        range_layout.addWidget(QtGui.QLabel("à C"))
        range_layout.addWidget(self.range_end)
        range_layout.addWidget(self.check_range_button)
        range_layout.addWidget(self.uncheck_range_button)
        self.layout.addRow("Plage :", range_widget)

        self.clear_button = QtGui.QPushButton("Supprimer tous les LE splits")
        self.layout.addRow(self.clear_button)

        self.preview_button = QtGui.QPushButton("Actualiser la preview")
        self.layout.addRow(self.preview_button)

        self.status_label = QtGui.QLabel()
        self.status_label.setWordWrap(True)
        self.layout.addRow(self.status_label)

        self._fill_cells_list()
        self._sync_elements()  # normalise legacy entries, drop orphans
        self._update_status()

        self.cells_list.itemChanged.connect(self._on_item_changed)
        self.check_range_button.clicked.connect(lambda: self._set_range(True))
        self.uncheck_range_button.clicked.connect(lambda: self._set_range(False))
        self.clear_button.clicked.connect(self.clear_all)
        self.preview_button.clicked.connect(self.update_preview)

        self.draw_preview()

    # ------------------------------------------------------------------
    # model <-> widget
    # ------------------------------------------------------------------
    def _le_bounds(self, cell_no):
        """(front, back) of the cell's LE panel, or None if it cannot be split."""
        glider = self.parametric_glider
        bounds = glider.get_le_panel_bounds(cell_no)
        if bounds is None and glider.is_asymmetric:
            bounds = glider.get_le_panel_bounds(cell_no, "right") or glider.get_le_panel_bounds(
                cell_no, "left"
            )
        return bounds

    def _fill_cells_list(self):
        stored = self.parametric_glider.le_split_cells()
        self._dropped = {c for c in stored if not 0 <= c < self.num_cells}
        self.cells_list.blockSignals(True)
        self.cells_list.clear()
        for cell_no in range(self.num_cells):
            bounds = self._le_bounds(cell_no)
            item = QtGui.QListWidgetItem()
            item.setData(QtCore.Qt.UserRole, cell_no)
            if bounds is None:
                item.setText(f"C{cell_no + 1} : pas de coupe BA sur l'extrados")
                item.setFlags(QtCore.Qt.NoItemFlags)
                item.setCheckState(QtCore.Qt.Unchecked)
                if cell_no in stored:
                    self._dropped.add(cell_no)
            else:
                front, back = bounds
                item.setText(
                    f"C{cell_no + 1} : panneau BA de {front * 100:+.1f} % à {back * 100:+.1f} %"
                )
                item.setFlags(CHECKABLE)
                item.setCheckState(
                    QtCore.Qt.Checked if cell_no in stored else QtCore.Qt.Unchecked
                )
            self.cells_list.addItem(item)
        self.cells_list.blockSignals(False)

    def checked_cells(self):
        cells = []
        for i in range(self.cells_list.count()):
            item = self.cells_list.item(i)
            if item.checkState() == QtCore.Qt.Checked:
                cells.append(int(item.data(QtCore.Qt.UserRole)))
        return cells

    def _sync_elements(self):
        self.parametric_glider.set_le_split_cells(self.checked_cells())

    def _update_status(self):
        cells = self.checked_cells()
        if cells:
            text = f"{len(cells)} cellule(s) avec LE split : " + ", ".join(
                f"C{c + 1}" for c in cells
            )
        else:
            text = "Aucun LE split."
        if self._dropped:
            text += "\nEntrées ignorées (pas de coupe BA ou cellule inexistante), supprimées à la validation : " + ", ".join(
                f"C{c + 1}" for c in sorted(self._dropped)
            )
        self.status_label.setText(text)

    def _after_change(self):
        self._sync_elements()
        self._update_status()
        self.update_preview()

    def _on_item_changed(self, item):
        self._after_change()

    def _set_range(self, checked):
        first, last = sorted((self.range_start.value(), self.range_end.value()))
        state = QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked
        self.cells_list.blockSignals(True)
        for i in range(first - 1, last):
            item = self.cells_list.item(i)
            if item.flags() & QtCore.Qt.ItemIsEnabled:
                item.setCheckState(state)
        self.cells_list.blockSignals(False)
        self._after_change()

    def clear_all(self):
        self.cells_list.blockSignals(True)
        for i in range(self.cells_list.count()):
            self.cells_list.item(i).setCheckState(QtCore.Qt.Unchecked)
        self.cells_list.blockSignals(False)
        self._dropped = set()
        self._after_change()

    # ------------------------------------------------------------------
    # preview
    # ------------------------------------------------------------------
    def draw_preview(self):
        """Draw the glider panels; the two halves of every split LE panel are
        recoloured so the seam is visible."""
        _rot = coin.SbRotation()
        _rot.setValue(coin.SbVec3f(0, 1, 0), coin.SbVec3f(1, 0, 0))
        rot = coin.SoRotation()
        rot.rotation.setValue(_rot)
        self.task_separator += rot

        glider = self.parametric_glider.get_glider_3d()
        for cell in glider.cells:
            for panel in cell.panels:
                y_start = getattr(panel, "y_start", 0.0)
                y_end = getattr(panel, "y_end", 1.0)
                if (y_start, y_end) != (0.0, 1.0):
                    panel.material_code = (
                        SPLIT_COLOR_INNER if y_start == 0.0 else SPLIT_COLOR_OUTER
                    )

        draw_glider(
            glider,
            self.task_separator,
            hull="panels",
            ribs=True,
            fill_ribs=False,
        )

    def update_preview(self):
        self.task_separator.removeAllChildren()
        self.draw_preview()

    def accept(self):
        self._sync_elements()
        super().accept()
        self.update_view_glider()

    def reject(self):
        super().reject()
