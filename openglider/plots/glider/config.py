import numpy as np

from openglider.plots import cuts, marks
from openglider.utils import Config
from openglider.utils.distribution import Distribution


class PatternConfig(Config):
    patterns_scale = 1000
    complete_glider = True
    debug = False
    profile_numpoints = 250

    cut_entry = cuts.FoldedCut
    cut_trailing_edge = cuts.ParallelCut
    cut_design = cuts.ParallelCut
    cut_diagonal_fold = cuts.FoldedCut
    cut_3d = cuts.Cut3D

    midribs = 50

    patterns_align_dist_y = 0.1
    patterns_align_dist_x = patterns_align_dist_y
    patterns_scale = 1000

    allowance_general = 0.01
    allowance_parallel = 0.01
    allowance_orthogonal = 0.01
    allowance_diagonals = 0.01
    allowance_trailing_edge = 0.01
    allowance_entry_open = 0.015
    allowance_rod_sleeve = 0.01
    # Shark-nose reinforcement is glued / sewn flat: seam allowance on the
    # intrados edge only (caught in the rib's intrados seam), net everywhere else.
    allowance_reinforcement = 0.01

    marks_diagonal_front = marks.Inside(marks.Arrow(left=True, name="diagonal_front"))
    marks_diagonal_back = marks.Inside(marks.Arrow(left=False, name="diagonal_back"))
    marks_laser_diagonal = marks.Dot(0.8)

    marks_laser_attachment_point = marks.Dot(0.2, 0.8)
    marks_attachment_point = marks.OnLine(
        marks.Rotate(marks.Cross(name="attachment_point"), np.pi / 4)
    )

    marks_strap = marks.Inside(marks.Line(name="strap"))
    # band-split diagonals: where neighbouring bands meet on the rib / panel
    marks_band_split = marks.Inside(marks.Line(name="band_split"))
    # tick across the allowance at the ends of a band piece's seams
    marks_band_split_seam = marks.Line(name="band_split_seam")
    # letters appended to piece names on complete-glider exports: (left, right)
    wing_side_labels = ("G", "D")

    distribution_controlpoints = Distribution.from_linear(20, -1, 1)
    marks_laser_controlpoint = marks.Dot(0.2)
    marks_controlpoint = marks.Dot(0.2)

    marks_panel_cut = marks.Line(name="panel_cut")
    rib_text_pos = -0.005

    allowance_design = 0.012  # trailing_edge

    drib_allowance_folds = 0.012
    drib_num_folds = 1
    drib_text_position = 0.1

    strap_num_folds = 1

    insert_attachment_point_text = True

    laser_text_mode = True

    dot_spacing = 0.15

    layout_seperate_panels = True

    text_inset_ratio = 0.85  # 0..1: how deep into seam margin (1 = at cut edge, 0 = at stitch line)


class OtherPatternConfig(PatternConfig):
    complete_glider = False
    cut_entry = cuts.SimpleCut
    cut_trailing_edge = cuts.SimpleCut
    cut_design = cuts.SimpleCut
    cut_diagonal_fold = cuts.SimpleCut
    layout_seperate_panels = True
    # draw_rib = None

    allowance_design = 0.01
    drib_allowance_folds = 0.01
    strap_num_folds = 1
    allowance_entry_open = 0.021
