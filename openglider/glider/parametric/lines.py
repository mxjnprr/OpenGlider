import ast
import copy
import logging
import math
import re

import numpy as np

from openglider.glider.rib.elements import AttachmentPoint, CellAttachmentPoint
from openglider.lines import Line, LineSet, Node, line_types
from openglider.utils.table import Table

logging.getLogger(__name__)


class LowerNode2D:
    """lower attachment point"""

    def __init__(self, pos_2D, pos_3D, name="unnamed", layer=None):
        self.pos_2D = pos_2D
        self.pos_3D = pos_3D
        self.name = name
        self.layer = layer or ""

    def __repr__(self):
        return f"<LowerNode2D {self.name}>"

    def __json__(self):
        return {
            "pos_2D": self.pos_2D,
            "pos_3D": self.pos_3D,
            "name": self.name,
            "layer": self.layer,
        }

    def get_2D(self, *args):
        return self.pos_2D

    def get_node(self, glider):
        return Node(node_type=0, position_vector=np.array(self.pos_3D), name=self.name)


class UpperNode2D:
    """stores the 2d data of an attachment point"""

    def __init__(
        self, cell_no, rib_pos, cell_pos=0, force=1.0, name="unnamed", layer=None
    ):
        self.cell_no = cell_no
        self.cell_pos = cell_pos
        self.rib_pos = rib_pos  # value from 0...1
        self.force = force
        self.name = name
        self.layer = layer or ""

    def __json__(self):
        return {
            "cell_no": self.cell_no,
            "rib_pos": self.rib_pos,
            "cell_pos": self.cell_pos,
            "force": self.force,
            "name": self.name,
            "layer": self.layer,
        }

    def __repr__(self):
        return f"<UpperNode2D name:{self.name} cell_no:{self.cell_no} cell_pos: {self.cell_pos} rib_pos:{self.rib_pos}"

    def get_2D(self, parametric_shape):
        return parametric_shape[self.cell_no, self.rib_pos]

    def get_node(self, glider):
        if 1 > self.cell_pos > 0:  # attachment point between two ribs
            cell_idx = self.cell_no + glider.has_center_cell
            if cell_idx < 0 or cell_idx >= len(glider.cells):
                logging.warning(
                    f"UpperNode2D '{self.name}': cell_no {self.cell_no} "
                    f"(index {cell_idx}) out of range "
                    f"(0..{len(glider.cells)-1}). Skipping node."
                )
                return None
            cell = glider.cells[cell_idx]
            if isinstance(self.force, (list, tuple, np.ndarray)):
                force = list(self.force)
            else:
                midrib = cell.midrib(self.cell_pos)
                force1 = np.array([0, self.force, 0])
                plane = midrib.projection_layer
                force = np.array(plane.translation_matrix.dot(force1))[0]

            node = CellAttachmentPoint(
                cell, self.name, self.cell_pos, self.rib_pos, force
            )
        else:  # attachment point on the rib
            rib_idx = self.cell_no + self.cell_pos + glider.has_center_cell
            if rib_idx < 0 or rib_idx >= len(glider.ribs):
                logging.warning(
                    f"UpperNode2D '{self.name}': rib index {rib_idx} "
                    f"(cell_no={self.cell_no}, cell_pos={self.cell_pos}) "
                    f"out of range (0..{len(glider.ribs)-1}). Skipping node."
                )
                return None
            rib = glider.ribs[rib_idx]
            if isinstance(self.force, (list, tuple, np.ndarray)):
                force = list(self.force)
            else:
                force = rib.rotation_matrix(np.array([0, self.force, 0]))

            node = AttachmentPoint(rib, self.name, self.rib_pos, force)

        node.get_position()
        return node


class BatchNode2D:
    def __init__(self, pos_2D, name=None, layer=None):
        self.pos_2D = pos_2D  # pos => 2d coordinates
        self.name = name
        self.layer = layer or ""

    def __json__(self):
        return {"pos_2D": self.pos_2D, "name": self.name, "layer": self.layer}

    def get_node(self, glider):
        return Node(node_type=1)

    def get_2D(self, *args):
        return self.pos_2D


class LineSet2D:
    regex_node = re.compile(r"([a-zA-Z]*)([0-9]*)")

    def __init__(self, line_list):
        self.lines = line_list

    def __json__(self):
        lines = [copy.copy(line) for line in self.lines]
        nodes = self.nodes
        for line in lines:
            line.upper_node = nodes.index(line.upper_node)
            line.lower_node = nodes.index(line.lower_node)
        return {"lines": lines, "nodes": nodes}

    @classmethod
    def __from_json__(cls, lines, nodes):
        lineset = cls(lines)
        for line in lineset.lines:
            if isinstance(line.upper_node, int):
                line.upper_node = nodes[line.upper_node]
            if isinstance(line.lower_node, int):
                line.lower_node = nodes[line.lower_node]
        return lineset

    @property
    def nodes(self):
        nodes = set()
        for line in self.lines:
            nodes.add(line.upper_node)
            nodes.add(line.lower_node)

        return list(nodes)

    def get_upper_nodes(self, rib_no=None):
        nodes = set()
        for line in self.lines:
            node = line.upper_node
            if isinstance(node, UpperNode2D):
                if rib_no is None or node.cell_no == rib_no:
                    nodes.add(line.upper_node)

        return list(nodes)

    def get_upper_node(self, name):
        for node in self.get_upper_nodes():
            if node.name == name:
                return node

    def get_lower_attachment_points(self):
        return [node for node in self.nodes if isinstance(node, LowerNode2D)]

    def return_lineset(self, glider, v_inf):
        """
        Get Lineset_3d
        :param glider: Glider3D
        :param v_inf:
        :return: LineSet (3d)
        """
        # v_inf = v_inf or glider.v_inf
        lines = []
        # now get the connected lines
        # get the other point (change the nodes if necessary)
        for node in self.get_lower_attachment_points():
            self.sort_lines(node)
        self.delete_not_connected(glider)

        nodes_3d = {}
        skipped_nodes = []
        for node in self.nodes:
            node_3d = node.get_node(glider)
            if node_3d is not None:
                nodes_3d[node] = node_3d
            else:
                skipped_nodes.append(node)
                logging.warning(
                    f"Lineset: skipping invalid node '{getattr(node, 'name', '?')}' "
                    f"(out of range after cell count change)"
                )

        # set up the lines!
        for line in self.lines:
            lower = nodes_3d.get(line.lower_node)
            upper = nodes_3d.get(line.upper_node)
            if lower and upper:
                line = Line(
                    number=len(lines),
                    lower_node=lower,
                    upper_node=upper,
                    v_inf=None,
                    target_length=line.target_length,
                    line_type=line.line_type,
                    name=line.name,
                )
                lines.append(line)

        if skipped_nodes:
            logging.warning(
                f"Lineset: {len(skipped_nodes)} node(s) were out of range and skipped. "
                f"Please update the line plan to match the current cell count."
            )

        return LineSet(lines, v_inf)

    def scale_forces(self, glider, node, weight):
        """
        scales all forces to match a certain weight in a node
        """
        # get upper connected force of the node
        # use z-direction of this force
        # compute scaling
        # scale all forces
        pass

    def distribute_forces(
        self,
        parametric_glider,
        spanwise="elliptical",
        spanwise_exponent=1.0,
        chordwise="airfoil",
        normalize="max",
        spanwise_floor=0.05,
        min_force=0.02,
    ):
        """
        Automatically assign a *relative* force to every upper attachment
        point from an analytical lift distribution, instead of setting them
        by hand.

        The force of a point is the share of lift it carries::

            force = (span-wise load of its rib station)
                    x (chord-wise share of that station)

        Because :meth:`LineSet.calc_forces` already sums the branches of every
        "patte d'oie" from top to bottom, only these top forces need to be
        right -- the whole cascade then balances itself.

        :param parametric_glider: the ParametricGlider (needs ``.shape`` and
            ``.arc``); the projected span from the arc is used when available.
        :param spanwise: span-wise loading law
            ``"elliptical"`` -> ``l(eta) = (1 - eta**2) ** (exponent/2)``,
            ``"chord"`` -> proportional to local chord (constant-Cl),
            ``"uniform"`` -> constant load per unit span.
        :param spanwise_exponent: exponent for the elliptical law (1.0 = pure
            ellipse; >1 loads the center more, <1 loads the tips more).
        :param chordwise: chord-wise loading law used to split a rib's load
            between its points (A/B/C/D)
            ``"airfoil"`` -> thin-airfoil loading ``dcp(x) ~ sqrt((1-x)/x)``
            (strong near the leading edge, so front rows carry more),
            ``"uniform"`` -> constant load per unit chord (pure tributary).
        :param normalize: ``"max"`` (largest force = 1.0), ``"mean"`` (mean
            force = 1.0) or ``None`` (keep raw relative values).
        :param spanwise_floor: minimum span-wise loading factor so wing-tip and
            stabilo points keep a small non-zero force (a zero force would
            degenerate in :meth:`LineSet.calc_forces`).
        :param min_force: every force is floored to ``min_force`` times the peak
            force, so no point ends up with a near-zero (numerically singular)
            force.
        :return: ``dict`` mapping each ``UpperNode2D`` to its new force.
        """
        upper_nodes = self.get_upper_nodes()
        if not upper_nodes:
            return {}

        shape = parametric_glider.shape
        x_values = list(shape.rib_x_values)
        n_ribs = len(x_values)

        # span-wise projected position of every rib (arc -> flat fallback)
        try:
            arc_pos = list(parametric_glider.arc.get_arc_positions(x_values))
            y_rib = [abs(float(p[0])) for p in arc_pos]
        except Exception:
            y_rib = [abs(float(x)) for x in x_values]
        if len(y_rib) < 2 or y_rib[-1] <= 0:
            y_rib = list(range(n_ribs)) or [0.0]
        span = y_rib[-1] or 1.0

        # chord length of every rib
        try:
            chord_rib = [abs(float(ba[1] - fr[1])) for fr, ba in shape.ribs]
        except Exception:
            chord_rib = [1.0] * n_ribs
        max_chord = max(chord_rib) or 1.0

        # span-wise tributary width of every rib (half-way to its neighbours).
        # The inner edge of the first rib is the symmetry plane (y=0).
        edges = [0.0] * (n_ribs + 1)
        edges[0] = 0.0
        edges[-1] = y_rib[-1]
        for i in range(1, n_ribs):
            edges[i] = 0.5 * (y_rib[i - 1] + y_rib[i])
        width_rib = [max(edges[i + 1] - edges[i], 1e-9) for i in range(n_ribs)]
        # a rib sitting on the symmetry plane (no center cell) carries a strip
        # on both sides -> count its tributary symmetrically
        if not getattr(shape, "has_center_cell", False):
            width_rib[0] *= 2.0

        def interp(arr, idx):
            if idx <= 0:
                return arr[0]
            if idx >= len(arr) - 1:
                return arr[-1]
            lo = int(idx)
            frac = idx - lo
            return arr[lo] * (1 - frac) + arr[lo + 1] * frac

        def spanwise_load(rib_idx):
            eta = min(max(interp(y_rib, rib_idx) / span, 0.0), 1.0)
            if spanwise == "chord":
                base = interp(chord_rib, rib_idx) / max_chord
            elif spanwise == "uniform":
                base = 1.0
            else:  # elliptical
                base = max(1.0 - eta * eta, 0.0) ** (spanwise_exponent / 2.0)
            # keep tip points lightly loaded instead of exactly zero
            base = max(base, spanwise_floor)
            return base * interp(width_rib, rib_idx)

        def chord_cumulative(x):
            # integral of the chord-wise loading from 0 to x
            x = min(max(x, 0.0), 1.0)
            if chordwise == "uniform":
                return x
            # thin-airfoil: int sqrt((1-x)/x) dx = asin(sqrt(x)) + sqrt(x(1-x))
            sx = math.sqrt(x)
            return math.asin(sx) + sx * math.sqrt(max(1.0 - x, 0.0))

        # group points by rib station, split each station along the chord
        stations = {}
        for node in upper_nodes:
            rib_idx = node.cell_no + getattr(node, "cell_pos", 0)
            stations.setdefault(round(rib_idx, 6), []).append(node)

        raw = {}
        for rib_idx, nodes in stations.items():
            nodes_sorted = sorted(nodes, key=lambda n: n.rib_pos)
            span_load = spanwise_load(rib_idx)
            n = len(nodes_sorted)
            for i, node in enumerate(nodes_sorted):
                pos = nodes_sorted[i].rib_pos
                lo = 0.0 if i == 0 else 0.5 * (nodes_sorted[i - 1].rib_pos + pos)
                hi = 1.0 if i == n - 1 else 0.5 * (pos + nodes_sorted[i + 1].rib_pos)
                chord_share = chord_cumulative(hi) - chord_cumulative(lo)
                raw[node] = span_load * max(chord_share, 0.0)

        # normalise
        values = list(raw.values())
        factor = 1.0
        if values and normalize == "max" and max(values) > 0:
            factor = 1.0 / max(values)
        elif values and normalize == "mean":
            mean = sum(values) / len(values)
            if mean > 0:
                factor = 1.0 / mean

        scaled = {node: value * factor for node, value in raw.items()}
        # floor every force to a fraction of the peak so tip/stabilo points keep
        # a small non-zero force (avoids singular force projection downstream)
        peak = max(scaled.values()) if scaled else 0.0
        floor = min_force * peak
        result = {}
        for node, value in scaled.items():
            node.force = round(max(value, floor), 4)
            result[node] = node.force
        return result

    def scale(self, factor, scale_lower_floor=True):
        lower_nodes = self.get_lower_attachment_points()
        for line in self.lines:
            target_length = getattr(line, "target_length", None)
            if target_length is not None:
                if scale_lower_floor or line.lower_node not in lower_nodes:
                    line.target_length *= factor
        for node in lower_nodes:
            node.pos_3D = np.array(node.pos_3D) * factor
            node.pos_2D = np.array(node.pos_2D) * factor

    def set_default_nodes2d_pos(self, glider):
        lineset_3d = self.return_lineset(glider, [10, 0, 0])
        lineset_3d._calc_geo()
        line_dict = {line_no: line2d for line_no, line2d in enumerate(self.lines)}

        for line in lineset_3d.lines:
            pos_3d = line.upper_node.vec
            pos_2d = [pos_3d[1], pos_3d[2]]
            line_dict[line.number].upper_node.pos_2D = pos_2d

    def sort_lines(self, lower_att):
        """
        Recursive sorting of lines (check direction)
        """
        for line in self.lines:
            if not line.is_sorted:
                if lower_att == line.upper_node:
                    line.lower_node, line.upper_node = line.upper_node, line.lower_node
                if lower_att == line.lower_node:
                    line.is_sorted = True
                    self.sort_lines(line.upper_node)

    def get_upper_connected_lines(self, node):
        return [line for line in self.lines if line.lower_node is node]

    def get_lower_connected_lines(self, node):
        return [line for line in self.lines if line.upper_node is node]

    def get_influence_nodes(self, line):
        if isinstance(line.upper_node, UpperNode2D):
            return [line.upper_node]
        return sum(
            [
                self.get_influence_nodes(l)
                for l in self.get_upper_connected_lines(line.upper_node)
            ],
            [],
        )

    def create_tree(self, start_node=None):
        """
        Create a tree of lines
        :return: [(line, [(upper_line1, []),...]),(...)]
        """
        if start_node is None:
            start_node = self.get_lower_attachment_points()
            lines = []
            for node in start_node:
                lines += self.get_upper_connected_lines(node)
        else:
            lines = self.get_upper_connected_lines(start_node)

        def get_influence_nodes(line):
            if isinstance(line.upper_node, UpperNode2D):
                return [line.upper_node]
            return sum(
                [
                    get_influence_nodes(l)
                    for l in self.get_upper_connected_lines(line.upper_node)
                ],
                [],
            )

        for line in lines:
            if not get_influence_nodes(line):
                return line

        def sort_key(line):
            nodes = get_influence_nodes(line)
            if not nodes:
                pass
                # return -1
            val = sum(
                [
                    100 * (node.cell_no + node.cell_pos) + 100 * node.rib_pos
                    for node in nodes
                ]
            ) / len(nodes)

            return val

        lines.sort(key=sort_key)

        return [(line, self.create_tree(line.upper_node)) for line in lines]

    def get_input_table(self):
        table = Table()

        def insert_block(line, upper, row, column):
            table[row, column + 1] = line.line_type.name
            if upper:
                table[row, column] = round(line.target_length, 3)
                for line, line_upper in upper:
                    row = insert_block(line, line_upper, row, column + 2)
            else:  # Insert a top node
                name = line.upper_node.name
                if not name:
                    name = f"Rib_{line.upper_node.rib_no}/{line.upper_node.rib_pos}"
                table[row, column] = name
                row += 1
            return row

        row = 0
        for node in self.get_lower_attachment_points():
            tree = self.create_tree(node)
            table[row, 0] = node.name
            for line, upper in tree:
                row = insert_block(line, upper, row, 1)

        return table

    @classmethod
    def read_input_table(cls, sheet, attachment_points_lower, attachment_points_upper):
        # upper -> dct {name: node}
        num_rows = sheet.num_rows
        num_cols = sheet.num_columns

        linelist = []
        current_nodes = [None for row in range(num_cols)]
        row = 0
        column = 0
        count = 0

        while row < num_rows:
            value = sheet[row, column]  # length or node_no

            if value:
                if column == 0:  # first (line-)floor
                    lower_node_name = sheet[row, 0]
                    if not type(lower_node_name) == str:
                        lower_node_name = str(int(lower_node_name))

                    current_nodes = [attachment_points_lower[lower_node_name]] + [
                        None for __ in range(num_cols)
                    ]
                    column += 1

                else:
                    # We have a line
                    line_type_name = sheet[row, column + 1]

                    lower_node = current_nodes[column // 2]

                    # gallery
                    if column + 2 >= num_cols - 1 or sheet[row, column + 2] is None:
                        upper = attachment_points_upper[value]
                        line_length = None
                        row += 1
                        column = 0
                    # other line
                    else:
                        upper = BatchNode2D([0, 0])
                        current_nodes[column // 2 + 1] = upper
                        line_length = sheet[row, column]
                        column += 2

                    linelist.append(
                        Line2D(
                            lower_node,
                            upper,
                            target_length=line_length,
                            line_type=line_type_name,
                        )
                    )
                    count += 1

            else:
                if column == 0:
                    column += 1
                elif column + 2 >= num_cols:
                    row += 1
                    column = 0
                else:
                    column += 2

        return cls(linelist)

    def get_attachment_point_table(self):
        nodes = self.get_upper_nodes()
        node_groups = {}
        num_cells = 0
        tables_cell = []
        tables_rib = []
        tables = []

        # sort by layer
        for node in nodes:
            node_name = node.name or ""
            match = self.regex_node.match(node_name)
            if match:
                layer_name = match.group(1)
            else:
                layer_name = "none"

            node_groups.setdefault(layer_name, [])
            node_groups[layer_name].append(node)
            num_cells = max(num_cells, node.cell_no) + 1  # WHAT?

        groups = list(node_groups.keys())

        # per layer table
        def sorted(x):
            res = 0
            for i, character in enumerate(x[::-1]):
                res += (26**i) * (ord(character) - 64)

            return res

        groups.sort(key=sorted)
        for key in groups:
            table = Table()
            group_nodes = node_groups[key]
            is_rib_attachment_point = all(n.cell_pos in (0, 1) for n in group_nodes)

            if is_rib_attachment_point:
                for rib_no in range(num_cells + 1):
                    rib_nodes = filter(
                        lambda n: n.cell_no + n.cell_pos == rib_no, group_nodes
                    )

                    for i, node in enumerate(rib_nodes):
                        # name, rib_pos, force
                        table[0, 3 * i] = "ATP"
                        table[rib_no + 1, 3 * i] = node.name
                        table[rib_no + 1, 3 * i + 1] = node.rib_pos
                        table[rib_no + 1, 3 * i + 2] = node.force

                tables_rib.append(table)

            else:
                for cell_no in range(num_cells):
                    cell_nodes = filter(lambda n: n.cell_no == cell_no, group_nodes)
                    for i, node in enumerate(cell_nodes):
                        # name, cell_pos, rib_pos, force
                        table[0, 4 * i] = "ATP"
                        table[cell_no + 1, 4 * i] = node.name
                        table[cell_no + 1, 4 * i + 1] = node.cell_pos
                        table[cell_no + 1, 4 * i + 2] = node.rib_pos
                        table[cell_no + 1, 4 * i + 3] = node.force

                tables_cell.append(table)

        total_table = Table()
        for table in tables_cell:
            total_table.append_right(table)

        total_table_ribs = Table()
        for table in tables_rib:
            total_table_ribs.append_right(table)
        return total_table_ribs, total_table

    @staticmethod
    def read_attachment_point_table(
        cell_table: Table = None, rib_table: Table = None, half_cell_no=None
    ):
        has_center_cell = False
        if half_cell_no is not None:
            has_center_cell = half_cell_no % 2

        attachment_points = []
        values = ("name", "cell_pos", "rib_pos", "force")
        num_columns = int(cell_table.num_columns / 4)

        def get_force(force):
            if isinstance(force, str):
                return ast.literal_eval(force)
            return force

        for column in range(num_columns):
            column_0 = column * 4
            assert cell_table[0, column_0] in ("ATP", "AHP")

            for row in range(1, cell_table.num_rows):
                name = cell_table[row, column_0]
                if name:
                    force = get_force(cell_table[row, column_0 + 3])

                    cell_no = row - 1
                    if has_center_cell:
                        cell_no -= 1

                    attachment_points.append(
                        UpperNode2D(
                            cell_no=cell_no,
                            name=name,
                            cell_pos=cell_table[row, column_0 + 1],
                            rib_pos=cell_table[row, column_0 + 2],
                            force=force,  # parse list/tuple
                        )
                    )

        for column in range(int(rib_table.num_columns / 3)):
            for row in range(1, rib_table.num_rows):
                name = rib_table[row, column * 3]
                if name:
                    rib_no = row - 1

                    rib_pos = rib_table[row, column * 3 + 1]
                    force = get_force(rib_table[row, column * 3 + 2])

                    attachment_points.append(
                        UpperNode2D(
                            name=name, cell_no=rib_no, rib_pos=rib_pos, force=force
                        )
                    )

        for attachment_point in attachment_points:
            if attachment_point.cell_no >= half_cell_no:
                attachment_point.cell_no -= 1
                attachment_point.cell_pos = 1

        return attachment_points

    def delete_not_connected(self, glider):
        temp = []
        temp_new = []
        for line in self.lines:
            if isinstance(line.upper_node, UpperNode2D):
                if line.upper_node.cell_no >= len(glider.ribs):
                    temp.append(line)
                    self.nodes.remove(line.upper_node)

        while temp:
            for line in temp:
                conn_up_lines = [
                    j
                    for j in self.lines
                    if (j.lower_node == line.lower_node and j != line)
                ]
                conn_lo_lines = [
                    j
                    for j in self.lines
                    if (j.upper_node == line.lower_node and j != line)
                ]
                if len(conn_up_lines) == 0:
                    self.nodes.remove(line.lower_node)
                    self.lines.remove(line)
                    temp_new += conn_lo_lines
                temp.remove(line)
            temp = temp_new


class Line2D:
    def __init__(
        self,
        lower_node,
        upper_node,
        target_length=None,
        line_type="default",
        layer=None,
        name=None,
    ):
        self.lower_node = lower_node
        self.upper_node = upper_node
        self.target_length = target_length
        self.is_sorted = False
        self.line_type = line_types.LineType.get(line_type)
        self.layer = layer or ""
        self.name = name

    def __json__(self):
        return {
            "lower_node": self.lower_node,
            "upper_node": self.upper_node,
            "target_length": self.target_length,
            "line_type": self.line_type.name,
            "layer": self.layer,
            "name": self.name,
        }
