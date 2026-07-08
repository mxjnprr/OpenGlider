#! /usr/bin/python
#
# Visual Test for LeadingEdgeClosure
# Tests the 3D geometry and 2D flattening of the leading edge closure feature.
#
import math
import os
import sys

try:
    import openglider
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0]))))

import numpy as np

from openglider.glider.ballooning import BallooningBezier
from openglider.glider.cell.cell import Cell
from openglider.glider.cell.elements import LeadingEdgeClosure
from openglider.glider.rib.rib import Rib

# Try to import graphics (optional, needs vtk)
try:
    import openglider.graphics
    HAS_GRAPHICS = True
except ImportError:
    HAS_GRAPHICS = False
    print("Note: vtk not installed, skipping visualization")


# Load test profile
prof = openglider.airfoil.Profile2D.import_from_dat(
    os.path.dirname(os.path.abspath(__file__)) + "/common/testprofile.dat"
)

# Create ballooning
ballooning = BallooningBezier()

# Create two ribs (mirrored for a symmetric cell)
r1 = Rib(prof, [0., 0.12, 0], 1., 20 * math.pi / 180, 2 * math.pi / 180, 0, 7.)
r2 = r1.copy()
r2.mirror()

# Create cell
cell = Cell(r1, r2, ballooning)

# Create LeadingEdgeClosure with 10% chord
le_closure = LeadingEdgeClosure(cut_back_x=0.1, y_position=0.5)

print("=" * 60)
print("LeadingEdgeClosure Visual Test")
print("=" * 60)

# Test 1: Get 3D geometry
print("\n--- Testing 3D Geometry ---")
ribs_left = le_closure.get_3d_left(cell, numribs=4)
ribs_right = le_closure.get_3d_right(cell, numribs=4)

print(f"Left half-panel: {len(ribs_left)} ribs")
print(f"Right half-panel: {len(ribs_right)} ribs")

for i, rib in enumerate(ribs_left):
    print(f"  Left rib {i}: {len(rib)} points")

# Test 2: Get mesh
print("\n--- Testing 3D Mesh ---")
mesh = le_closure.get_mesh(cell, numribs=4)
print(f"Mesh polygons: {list(mesh.polygons.keys())}")

# Test 3: Flatten and check arc lengths match
print("\n--- Testing 2D Flattening ---")
left_boundary_l, right_boundary_l = le_closure.get_flattened(cell, "left", numribs=10)
left_boundary_r, right_boundary_r = le_closure.get_flattened(cell, "right", numribs=10)

if left_boundary_l is not None and left_boundary_r is not None:
    # The inner edges (right_boundary of left, left_boundary of right) should have same arc length
    inner_left_length = right_boundary_l.get_length()
    inner_right_length = left_boundary_r.get_length()
    
    print(f"Left panel - inner edge length: {inner_left_length:.4f} m")
    print(f"Right panel - inner edge length: {inner_right_length:.4f} m")
    print(f"Difference: {abs(inner_left_length - inner_right_length):.6f} m")
    
    if abs(inner_left_length - inner_right_length) < 0.001:
        print("✓ Arc lengths match (sewable)")
    else:
        print("✗ Arc lengths mismatch!")

# Test 4: Get PlotPart
print("\n--- Testing PlotPart Export ---")
plotpart_left = le_closure.get_flattened_plotpart(cell, "left", numribs=10)
plotpart_right = le_closure.get_flattened_plotpart(cell, "right", numribs=10)

if plotpart_left is not None:
    print(f"Left PlotPart: {len(plotpart_left.layers['cuts'])} cuts, {len(plotpart_left.layers['marks'])} marks")
if plotpart_right is not None:
    print(f"Right PlotPart: {len(plotpart_right.layers['cuts'])} cuts, {len(plotpart_right.layers['marks'])} marks")

# Visualization
print("\n--- Visualizing ---")

if HAS_GRAPHICS:
    # 3D Visualization
    try:
        graphics_3d = []
        for rib in ribs_left:
            graphics_3d.append(openglider.graphics.Line(list(rib)))
        for rib in ribs_right:
            graphics_3d.append(openglider.graphics.Line(list(rib)))
        
        # Visualize mesh
        # mesh_graphics = openglider.graphics.Graphics3D(mesh.get_lines())
        
    except Exception as e:
        print(f"3D visualization error: {e}")

    # 2D Visualization
    try:
        if left_boundary_l is not None and right_boundary_l is not None:
            # Left panel
            graphics_2d = [
                openglider.graphics.Line(left_boundary_l.data),
                openglider.graphics.Line(right_boundary_l.data),
            ]
            
            # Right panel (offset to the right for visibility)
            if left_boundary_r is not None:
                offset = np.array([right_boundary_l.data[-1][0] + 0.05, 0])
                left_r_offset = [p + offset for p in left_boundary_r.data]
                right_r_offset = [p + offset for p in right_boundary_r.data]
                graphics_2d.append(openglider.graphics.Line(left_r_offset))
                graphics_2d.append(openglider.graphics.Line(right_r_offset))
            
            print("Displaying 2D flattened panels...")
            openglider.graphics.Graphics2D(graphics_2d)
    except Exception as e:
        print(f"2D visualization error: {e}")
else:
    print("Graphics not available (vtk not installed)")

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)
