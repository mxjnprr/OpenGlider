#! /usr/bin/python
"""
LE Panel Split Tool - Tool for splitting leading edge panels.

This tool creates 3D cuts at a specified chord percentage, creating a 
leading edge panel that is then split spanwise for manufacturing.

The cut created is a "3d" type cut visible in the Design and Colors tools.
"""

from pivy import coin
from PySide import QtGui

from .glider import draw_glider
from .tools import BaseTool, input_field


class LEPanelSplitTool(BaseTool):
    """
    Tool for creating leading edge panel cuts and splitting them spanwise.
    
    Creates real "3d" type cuts that appear in Design tool and Colors.
    """
    hide = True
    turn = False

    def __init__(self, obj):
        super().__init__(obj)
        
        # Get number of cells
        self.num_cells = self.parametric_glider.shape.half_cell_num
        
        # Description label
        description = QtGui.QLabel(
            "Crée une coupe 3D au bord d'attaque.\n"
            "Le panneau créé sera visible dans Design et Colors."
        )
        description.setWordWrap(True)
        self.layout.setWidget(0, input_field, description)
        
        # Cut percentage (how far from LE the cut goes)
        self.layout.setWidget(1, QtGui.QFormLayout.LabelRole, QtGui.QLabel("Limite (%):"))
        self.cut_percentage = QtGui.QDoubleSpinBox()
        self.cut_percentage.setRange(1, 50)
        self.cut_percentage.setValue(10.0)
        self.cut_percentage.setSuffix(" % corde")
        self.cut_percentage.setDecimals(1)
        self.layout.setWidget(1, input_field, self.cut_percentage)
        
        # Cell range - start
        self.layout.setWidget(2, QtGui.QFormLayout.LabelRole, QtGui.QLabel("Cellule début:"))
        self.cell_start = QtGui.QSpinBox()
        self.cell_start.setRange(0, self.num_cells - 1)
        self.cell_start.setValue(0)
        self.layout.setWidget(2, input_field, self.cell_start)
        
        # Cell range - end
        self.layout.setWidget(3, QtGui.QFormLayout.LabelRole, QtGui.QLabel("Cellule fin:"))
        self.cell_end = QtGui.QSpinBox()
        self.cell_end.setRange(0, self.num_cells - 1)
        self.cell_end.setValue(self.num_cells - 1)
        self.layout.setWidget(3, input_field, self.cell_end)
        
        # Cut type selector
        self.layout.setWidget(4, QtGui.QFormLayout.LabelRole, QtGui.QLabel("Type de coupe:"))
        self.cut_type = QtGui.QComboBox()
        # Valid types: folded, parallel, orthogonal, singleskin, cut_3d
        self.cut_type.addItems(["cut_3d", "folded", "parallel", "orthogonal", "singleskin"])
        self.cut_type.setCurrentIndex(0)  # Default to "cut_3d"
        self.layout.setWidget(4, input_field, self.cut_type)
        
        # Add cut button
        self.add_cut_button = QtGui.QPushButton("Ajouter la coupe BA")
        self.add_cut_button.clicked.connect(self.add_le_cut)
        self.layout.setWidget(5, input_field, self.add_cut_button)
        
        # Preview button
        self.preview_button = QtGui.QPushButton("Actualiser la preview")
        self.preview_button.clicked.connect(self.update_preview)
        self.layout.setWidget(6, input_field, self.preview_button)
        
        # List of existing LE cuts
        self.layout.setWidget(7, QtGui.QFormLayout.LabelRole, QtGui.QLabel("Coupes BA existantes:"))
        self.cuts_list = QtGui.QListWidget()
        self.cuts_list.setMaximumHeight(120)
        self.layout.setWidget(7, input_field, self.cuts_list)
        
        # Delete selected cut button
        self.delete_cut_button = QtGui.QPushButton("Supprimer coupe sélectionnée")
        self.delete_cut_button.clicked.connect(self.delete_selected_cut)
        self.layout.setWidget(8, input_field, self.delete_cut_button)
        
        # Load existing cuts
        self._load_existing_cuts()
        
        # Draw preview with panels
        self.draw_preview()

    def _load_existing_cuts(self):
        """Load and display existing cuts from ParametricGlider."""
        self.cuts_list.clear()
        cuts = self.parametric_glider.elements.get("cuts", [])
        for i, cut in enumerate(cuts):
            # Show as percentage (convert from absolute rib_pos)
            left_pct = abs(cut["left"]) * 100
            right_pct = abs(cut["right"]) * 100
            cells_str = ",".join(map(str, cut["cells"]))
            side = "upper" if cut["left"] < 0 else "lower"
            text = f"{cut['type']}: {left_pct:.1f}%-{right_pct:.1f}% ({side}) [C{cells_str}]"
            item = QtGui.QListWidgetItem(text)
            item.setData(QtGui.Qt.UserRole, i)  # Store index
            self.cuts_list.addItem(item)

    def add_le_cut(self):
        """Add a new leading edge cut."""
        percentage = self.cut_percentage.value() / 100.0  # Convert to 0-1
        cell_start = self.cell_start.value()
        cell_end = self.cell_end.value()
        cut_type = self.cut_type.currentText()
        
        # Ensure start <= end
        if cell_start > cell_end:
            cell_start, cell_end = cell_end, cell_start
        
        cells = list(range(cell_start, cell_end + 1))
        
        # Create the cut entry
        # negative rib_pos = upper (extrados) side
        new_cut = {
            "cells": cells,
            "left": -percentage,   # Upper side (negative)
            "right": -percentage,  # Same position on both ribs
            "type": cut_type
        }
        
        # Add to existing cuts
        if "cuts" not in self.parametric_glider.elements:
            self.parametric_glider.elements["cuts"] = []
        
        # Check if a similar cut already exists for these cells
        existing_cuts = self.parametric_glider.elements["cuts"]
        
        # Try to merge with existing cut at same position
        merged = False
        for cut in existing_cuts:
            if (abs(cut["left"] - new_cut["left"]) < 0.001 and 
                abs(cut["right"] - new_cut["right"]) < 0.001 and
                cut["type"] == new_cut["type"]):
                # Merge cells
                for c in cells:
                    if c not in cut["cells"]:
                        cut["cells"].append(c)
                cut["cells"].sort()
                merged = True
                break
        
        if not merged:
            existing_cuts.append(new_cut)
        
        # Also store in le_panel_splits for spanwise split generation
        if "le_panel_splits" not in self.parametric_glider.elements:
            self.parametric_glider.elements["le_panel_splits"] = []
        
        # Add or merge le_panel_split entry
        le_splits = self.parametric_glider.elements["le_panel_splits"]
        le_merged = False
        for le_split in le_splits:
            if abs(le_split.get("cut_limit", 0) - percentage) < 0.001:
                for c in cells:
                    if c not in le_split["cells"]:
                        le_split["cells"].append(c)
                le_split["cells"].sort()
                le_merged = True
                break
        
        if not le_merged:
            le_splits.append({
                "cut_limit": percentage,
                "material_code": "",
                "cells": cells.copy()
            })
        
        # Refresh UI
        self._load_existing_cuts()
        self.update_preview()
        
        QtGui.QMessageBox.information(
            None, "Coupe ajoutée",
            f"Coupe {cut_type} à {percentage*100:.1f}% ajoutée pour cellules {cell_start}-{cell_end}"
        )

    def delete_selected_cut(self):
        """Delete the selected cut from the list."""
        current = self.cuts_list.currentItem()
        if not current:
            return
        
        index = current.data(QtGui.Qt.UserRole)
        cuts = self.parametric_glider.elements.get("cuts", [])
        if 0 <= index < len(cuts):
            cut_to_delete = cuts[index]
            cut_limit = abs(cut_to_delete["left"])
            cut_cells = set(cut_to_delete["cells"])
            
            # Delete the cut
            del cuts[index]
            
            # Also delete matching le_panel_splits entry
            le_splits = self.parametric_glider.elements.get("le_panel_splits", [])
            for i, split in enumerate(le_splits[:]):  # Iterate on copy
                if abs(split.get("cut_limit", 0) - cut_limit) < 0.001:
                    # Check if cells overlap
                    split_cells = set(split.get("cells", []))
                    if split_cells & cut_cells:  # Intersection
                        # Remove the matching cells
                        remaining_cells = split_cells - cut_cells
                        if remaining_cells:
                            split["cells"] = sorted(list(remaining_cells))
                        else:
                            le_splits.remove(split)
            
            self._load_existing_cuts()
            self.update_preview()

    def draw_preview(self):
        """Draw preview showing glider with panels."""
        _rot = coin.SbRotation()
        _rot.setValue(coin.SbVec3f(0, 1, 0), coin.SbVec3f(1, 0, 0))
        rot = coin.SoRotation()
        rot.rotation.setValue(_rot)
        self.task_separator += rot
        
        # Draw with panels visible
        draw_glider(
            self.parametric_glider.get_glider_3d(),
            self.task_separator,
            hull="panels",  # Show panels
            ribs=True,
            fill_ribs=False,
        )

    def update_preview(self):
        """Update the 3D preview."""
        self.task_separator.removeAllChildren()
        self.draw_preview()

    def accept(self):
        """Accept changes."""
        super().accept()
        self.update_view_glider()

    def reject(self):
        """Cancel changes."""
        super().reject()
