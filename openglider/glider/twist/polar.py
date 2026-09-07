"""Section polars for the 2d "kite" model of a rib.

A polar maps the local angle of attack (chord vs. in-plane wind) to the
section coefficients ``(Cl, Cd, Cm_c/4)``.  Only the *direction* of the
resultant and the *chord position* of the centre of pressure feed the twist
model, so a thin-airfoil estimate from the camber line is good enough for a
live tool; an XFoil-backed polar can be swapped in for a check run.
"""

import numpy as np


class Polar:
    """Base class: ``polar(alpha_rad) -> (Cl, Cd, Cm_c4)``."""

    alpha_min = np.radians(-10.0)  # validity window used by the root solver
    alpha_max = np.radians(18.0)

    def __call__(self, alpha):
        raise NotImplementedError

    def centre_of_pressure(self, alpha):
        """Chord fraction of the centre of pressure (from the leading edge).

        ``x_cp = 1/4 - Cm_c4 / Cl``; returns 0.25 where Cl is ~0 (undefined).
        """
        cl, _, cm = self(alpha)
        if abs(cl) < 1e-6:
            return 0.25
        return 0.25 - cm / cl


class ThinAirfoilPolar(Polar):
    """Thin-airfoil theory from a camber line, plus a parabolic drag polar.

    ``Cl = 2 pi (alpha - alpha_0)``, ``Cm_c4 = pi/4 (A2 - A1)`` with the
    Glauert coefficients integrated numerically from the camber slope, and
    ``Cd = cd0 + k Cl**2``.  Cl is soft-clipped at ``cl_max`` so the solver
    never runs off to absurd angles.
    """

    def __init__(self, camber_line=None, alpha_0=None, cm_c4=None,
                 cd0=0.015, k=0.02, cl_max=1.5, name=None):
        if camber_line is not None:
            self.alpha_0, self.cm_c4 = self.from_camber(camber_line)
        else:
            self.alpha_0 = 0.0 if alpha_0 is None else float(alpha_0)
            self.cm_c4 = 0.0 if cm_c4 is None else float(cm_c4)
        if alpha_0 is not None:
            self.alpha_0 = float(alpha_0)
        if cm_c4 is not None:
            self.cm_c4 = float(cm_c4)
        self.cd0 = cd0
        self.k = k
        self.cl_max = cl_max
        self.name = name

    @staticmethod
    def from_camber(camber_line, num=400):
        """Return ``(alpha_0, cm_c4)`` for a camber line ``[(x, z), ...]``.

        ``x`` runs 0 (leading edge) .. 1 (trailing edge) in chord units.  Uses
        the Glauert transformation ``x = (1 - cos theta) / 2``.
        """
        pts = np.asarray(camber_line, dtype=float)
        if len(pts) < 3:
            return 0.0, 0.0
        order = np.argsort(pts[:, 0])
        x = pts[order, 0]
        z = pts[order, 1]
        x0, x1 = x[0], x[-1]
        if x1 - x0 < 1e-9:
            return 0.0, 0.0
        x = (x - x0) / (x1 - x0)
        z = z / (x1 - x0)
        theta = np.linspace(0.0, np.pi, num)
        xs = 0.5 * (1.0 - np.cos(theta))
        zs = np.interp(xs, x, z)
        dz = np.gradient(zs, xs)
        alpha_0 = -1.0 / np.pi * np.trapz(dz * (np.cos(theta) - 1.0), theta)
        a1 = 2.0 / np.pi * np.trapz(dz * np.cos(theta), theta)
        a2 = 2.0 / np.pi * np.trapz(dz * np.cos(2.0 * theta), theta)
        cm_c4 = np.pi / 4.0 * (a2 - a1)
        return float(alpha_0), float(cm_c4)

    @classmethod
    def from_profile(cls, profile, **kwargs):
        """Build from an :class:`openglider.airfoil.Profile2D`."""
        return cls(camber_line=profile.camber_line,
                   name=getattr(profile, "name", None), **kwargs)

    def __call__(self, alpha):
        cl = 2.0 * np.pi * (alpha - self.alpha_0)
        # linear up to 70 % of cl_max, then a smooth (C1, monotonic) saturation
        # so the root solver never runs off to absurd angles
        lin = 0.7 * self.cl_max
        if abs(cl) > lin:
            rest = self.cl_max - lin
            cl = np.sign(cl) * (lin + rest * np.tanh((abs(cl) - lin) / rest))
        cd = self.cd0 + self.k * cl ** 2
        return float(cl), float(cd), self.cm_c4


class TabulatedPolar(Polar):
    """Interpolated polar from tabulated ``alpha_deg, cl, cd, cm`` columns."""

    def __init__(self, alpha_deg, cl, cd, cm, name=None):
        order = np.argsort(alpha_deg)
        self.alpha = np.radians(np.asarray(alpha_deg, dtype=float)[order])
        self.cl = np.asarray(cl, dtype=float)[order]
        self.cd = np.asarray(cd, dtype=float)[order]
        self.cm = np.asarray(cm, dtype=float)[order]
        self.alpha_min = float(self.alpha[0])
        self.alpha_max = float(self.alpha[-1])
        self.name = name

    @classmethod
    def from_xfoil(cls, profile, alpha_deg=None, re_number=None, name=None):
        """Run XFoil on ``profile`` (blocking).  Raises if XFoil is unavailable."""
        import asyncio

        from openglider.airfoil.xfoil import XFoilCalc

        calc = XFoilCalc(profile)
        if alpha_deg is not None:
            calc.alpha = list(alpha_deg)
        if re_number is not None:
            calc.re_number = re_number
        df = asyncio.run(calc.run())
        if df is None or len(df) < 2:
            raise RuntimeError("xfoil returned no converged points")
        return cls(df.index.values, df["ca"].values, df["cw"].values,
                   df["cm"].values, name=name or getattr(profile, "name", None))

    def __call__(self, alpha):
        a = float(np.clip(alpha, self.alpha_min, self.alpha_max))
        return (float(np.interp(a, self.alpha, self.cl)),
                float(np.interp(a, self.alpha, self.cd)),
                float(np.interp(a, self.alpha, self.cm)))
