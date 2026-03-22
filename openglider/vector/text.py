import numpy as np

from openglider.vector.functions import normalize
from openglider.vector.polyline import PolyLine2D
from openglider.vector.drawing.part import PlotPart

text_vectors = {
    "1": [[0.2, 0.5], [0.6, 1.0], [0.6, 0.0]],
    "2": [[0.2, 1.0], [0.8, 1.0], [0.8, 0.5], [0.2, 0.5], [0.2, 0.0], [0.8, 0.0]],
    "3": [
        [0.2, 1.0],
        [0.8, 1.0],
        [0.8, 0.5],
        [0.2, 0.5],
        [0.8, 0.5],
        [0.8, 0.0],
        [0.2, 0.0],
    ],
    "4": [[0.8, 0.5], [0.2, 0.5], [0.6, 1.0], [0.6, 0.0]],
    "5": [[0.8, 1.0], [0.2, 1.0], [0.2, 0.5], [0.8, 0.5], [0.8, 0.0], [0.2, 0.0]],
    "6": [
        [0.8, 1.0],
        [0.2, 1.0],
        [0.2, 0.5],
        [0.8, 0.5],
        [0.8, 0.0],
        [0.2, 0.0],
        [0.2, 0.5],
    ],
    "7": [[0.2, 1.0], [0.8, 1.0], [0.4, 0.0]],
    "8": [
        [0.8, 0.5],
        [0.8, 1.0],
        [0.2, 1.0],
        [0.2, 0.5],
        [0.8, 0.5],
        [0.8, 0.0],
        [0.2, 0.0],
        [0.2, 0.5],
    ],
    "9": [[0.8, 0.5], [0.2, 0.5], [0.2, 1.0], [0.8, 1.0], [0.8, 0.0], [0.2, 0.0]],
    "0": [[0.2, 0.0], [0.2, 1.0], [0.8, 1.0], [0.8, 0.0], [0.2, 0.0]],
    "A": [[0.1, 0], [0.5, 1.0], [0.9, 0], [0.7, 0.5], [0.3, 0.5]],
    "B": [
        [0.2, 1.0],
        [0.2, 0],
        [0.7, 0],
        [0.8, 0.1],
        [0.8, 0.4],
        [0.7, 0.5],
        [0.2, 0.5],
        [0.7, 0.5],
        [0.8, 0.6],
        [0.8, 0.9],
        [0.7, 1.0],
        [0.2, 1.0],
    ],
    "C": [[0.8, 1.0], [0.3, 1.0], [0.2, 0.9], [0.2, 0.1], [0.3, 0], [0.8, 0]],
    "D": [
        [0.2, 1.0],
        [0.5, 1.0],
        [0.7, 0.9],
        [0.8, 0.8],
        [0.9, 0.6],
        [0.9, 0.4],
        [0.8, 0.2],
        [0.7, 0.1],
        [0.5, 0],
        [0.2, 0],
        [0.2, 1.0],
    ],
    "E": [
        [0.8, 1.0],
        [0.2, 1.0],
        [0.2, 0.5],
        [0.8, 0.5],
        [0.2, 0.5],
        [0.2, 0],
        [0.8, 0],
    ],
    "F": [[0.8, 1.0], [0.2, 1.0], [0.2, 0.5], [0.7, 0.5], [0.2, 0.5], [0.2, 0]],
    "G": [
        [0.8, 1.0],
        [0.3, 1.0],
        [0.2, 0.9],
        [0.2, 0.1],
        [0.3, 0],
        [0.7, 0],
        [0.8, 0.1],
        [0.8, 0.4],
        [0.7, 0.5],
        [0.5, 0.5],
    ],
    "H": [[0.2, 1.0], [0.2, 0], [0.2, 0.5], [0.8, 0.5], [0.8, 0], [0.8, 1.0]],
    "I": [[0.4, 1.0], [0.6, 1.0], [0.5, 1.0], [0.5, 0], [0.4, 0], [0.7, 0]],
    "J": [[0.3, 1.0], [0.6, 1.0], [0.6, 0.1], [0.5, 0], [0.4, 0.1], [0.4, 0.2]],
    "K": [[0.2, 1.0], [0.2, 0], [0.2, 0.5], [0.8, 1.0], [0.2, 0.5], [0.8, 0]],
    "L": [[0.2, 1.0], [0.2, 0], [0.8, 0]],
    "M": [[0.2, 0], [0.2, 1.0], [0.5, 0.5], [0.8, 1.0], [0.8, 0]],
    "N": [[0.2, 0], [0.2, 1.0], [0.8, 0], [0.8, 1.0]],
    "O": [
        [0.2, 0.9],
        [0.2, 0.1],
        [0.3, 0.0],
        [0.7, 0.0],
        [0.8, 0.1],
        [0.8, 0.9],
        [0.7, 1.0],
        [0.3, 1.0],
        [0.2, 0.9],
    ],
    "P": [
        [0.2, 0.0],
        [0.2, 0.9],
        [0.3, 1.0],
        [0.7, 1.0],
        [0.8, 0.9],
        [0.8, 0.6],
        [0.7, 0.5],
        [0.2, 0.5],
    ],
    "Q": [
        [0.2, 0.9],
        [0.3, 1.0],
        [0.8, 1.0],
        [0.9, 0.9],
        [0.9, 0.1],
        [0.8, 0.0],
        [0.9, 0],
        [0.3, 0],
        [0.2, 0.1],
        [0.2, 0.9],
    ],
    "R": [
        [0.2, 0.0],
        [0.2, 0.9],
        [0.3, 1.0],
        [0.7, 1.0],
        [0.8, 0.9],
        [0.8, 0.6],
        [0.7, 0.5],
        [0.2, 0.5],
        [0.8, 0.0],
    ],
    "S": [
        [0.8, 0.9],
        [0.7, 1.0],
        [0.3, 1.0],
        [0.2, 0.9],
        [0.2, 0.6],
        [0.3, 0.5],
        [0.7, 0.5],
        [0.8, 0.4],
        [0.8, 0.1],
        [0.7, 0],
        [0.3, 0],
        [0.2, 0.1],
    ],
    "T": [[0.2, 1.0], [0.8, 1.0], [0.5, 1.0], [0.5, 0]],
    "U": [[0.2, 1.0], [0.2, 0.1], [0.3, 0], [0.7, 0], [0.8, 0.1], [0.8, 1.0]],
    "V": [[0.2, 1.0], [0.5, 0], [0.8, 1.0]],
    "W": [[0.2, 1.0], [0.3, 0], [0.5, 0.5], [0.7, 0], [0.8, 1.0]],
    "X": [[0.2, 1.0], [0.8, 0], [0.5, 0.5], [0.8, 1.0], [0.2, 0]],
    "Y": [
        [0.2, 1.0],
        [0.2, 0.6],
        [0.3, 0.5],
        [0.7, 0.5],
        [0.8, 0.6],
        [0.8, 1.0],
        [0.8, 0.6],
        [0.7, 0.5],
        [0.8, 0.4],
        [0.8, 0.1],
        [0.7, 0],
    ],
    "Z": [[0.2, 1.0], [0.8, 1.0], [0.2, 0], [0.8, 0]],
    "_": [[0.2, 0.0], [0.8, 0.0]],
    " ": [],
    ".": [[0.48, 0], [0.52, 0], [0.52, 0.04], [0.48, 0.04], [0.48, 0]],
}


class Text(object):
    letters = text_vectors

    def __init__(
        self, text, p1, p2, size=None, align="left", height=0.8, space=0.2, valign=0.5,
        dashed=False, dot_spacing=0.15
    ):
        """
        Vector Text
        :param text: Text
        :param p1: left orientation point
        :param p2: right orientation point
        :param size: letter size. if not set, letters are fit into space (p1/p2)
        :param align: horizontal align: ("left", "right", "center")
        :param height: letter height, relative to width
        :param space: space in between letters
        :param valign: vertical align ( -0.5: bottom, 0: centered, 0.5: top)
        :param dashed: if True, render strokes as perforation dots for laser cutting
        :param dot_spacing: spacing between dots relative to letter unit size (only when dashed=True)
        """
        self.text = text
        self.p1 = np.array(p1[:])
        self.p2 = np.array(p2[:])
        self.size = size
        self.height = height
        self.space = space
        self.align = align
        self.valign = valign
        self.dashed = dashed
        self.dot_spacing = dot_spacing

    def __json__(self):
        return {
            "text": self.text,
            "p1": self.p1,
            "p2": self.p2,
            "size": self.size,
            "height": self.height,
            "space": self.space,
            "align": self.align,
        }

    def get_letter(self, letter, replace_unknown=True):
        letter = letter.upper()
        if letter not in self.letters:
            if replace_unknown:
                letter = "_"
            else:
                raise KeyError(
                    "Letter {} from word '{}' not available".format(
                        letter, self.text.upper()
                    )
                )

        data = self.letters[letter]
        return [text_vectors[letter]]

    def get_vectors(self, replace_unknown=True):
        # todo: add valign (space)
        vectors = []
        diff = (self.p2 - self.p1) / len(self.text)
        if self.size is not None:
            diff = normalize(diff) * self.size

        if self.align == "left":
            p1 = self.p1.copy()
        elif self.align == "center":
            p1 = self.p1 + (self.p2 - self.p1) / 2 - len(self.text) / 2 * diff
        elif self.align == "right":
            p1 = self.p2 - len(self.text) * diff
        else:
            raise ValueError

        r_x, r_y = diff[0], diff[1]

        rot = np.array([[r_x, -r_y * self.height], [r_y, r_x * self.height]])

        p1 += np.array([-r_y, r_x]) * (self.valign - 0.5)

        for letter in self.text:
            points = self.get_letter(letter, replace_unknown=replace_unknown)
            for lst in points:
                if lst:
                    stroke = PolyLine2D([p1 + rot.dot(p) for p in lst], name="text")
                    if self.dashed:
                        vectors += self._dotify(stroke)
                    else:
                        vectors.append(stroke)
            p1 += diff
        return vectors

    @staticmethod
    def _dotify_polyline(polyline, spacing):
        """Place single-point dots along a polyline at regular intervals.

        Each dot is a PolyLine2D with a single point, identical to how
        notch / control-point marks are stored.  The laser hits each
        point once, creating a perforation pattern that forms readable text.

        Returns a list of single-point PolyLine2D objects.
        """
        if len(polyline.data) < 2:
            return [polyline]

        pts = [np.array(p) for p in polyline.data]
        result = []

        # Always place the first point
        result.append(PolyLine2D([pts[0].tolist()], name="text"))

        accumulated = 0.0
        for i in range(len(pts) - 1):
            seg_start = pts[i]
            seg_end = pts[i + 1]
            seg_vec = seg_end - seg_start
            seg_length = np.linalg.norm(seg_vec)
            if seg_length < 1e-12:
                continue
            seg_dir = seg_vec / seg_length
            consumed = 0.0

            while consumed < seg_length - 1e-12:
                available = seg_length - consumed
                if accumulated + available >= spacing:
                    # place a dot
                    step = spacing - accumulated
                    consumed += step
                    point = seg_start + seg_dir * consumed
                    result.append(PolyLine2D([point.tolist()], name="text"))
                    accumulated = 0.0
                else:
                    accumulated += available
                    break

        return result if result else [polyline]

    def _dotify(self, polyline):
        """Dotify a polyline using this Text instance's spacing setting,
        scaled to absolute units based on the letter size."""
        diff = (self.p2 - self.p1) / max(len(self.text), 1)
        if self.size is not None:
            scale = self.size
        else:
            scale = np.linalg.norm(diff)

        abs_spacing = self.dot_spacing * scale

        if abs_spacing < 1e-10:
            return [polyline]

        return self._dotify_polyline(polyline, abs_spacing)

    def get_plotpart(self, replace_unknown=True):
        vectors = self.get_vectors(replace_unknown)
        return PlotPart(vectors)
