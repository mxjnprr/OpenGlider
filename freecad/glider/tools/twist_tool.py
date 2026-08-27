"""Twist (vrillage) tool.

Extends the AoA tool with the per-rib "kite" diagnostic of
:mod:`openglider.glider.twist` and a twist <-> sweep corrector.  The panel
is in French (the designer's language); the physics is documented in
:mod:`openglider.glider.twist.model`.
"""

import numpy as np
from pivy import coin
from PySide import QtCore, QtGui

from openglider.glider.twist import TabulatedPolar, TwistModel, TwistObjective

from .span_mapping import AoaTool
from .tools import Line_old, text_field

HIDDEN = [[0.0, 0.0, 0.0]]  # a Line needs at least one point

# colours (r, g, b), also used as html in the legend
GREEN = (0.10, 0.60, 0.10)
MAGENTA = (0.70, 0.10, 0.70)
ORANGE = (0.95, 0.55, 0.05)
LIGHT_ORANGE = (1.00, 0.75, 0.40)
CYAN = (0.05, 0.60, 0.75)
LIGHT_CYAN = (0.55, 0.80, 0.90)
DARK_GREY = (0.35, 0.35, 0.35)


def _html(rgb):
    return "#%02x%02x%02x" % tuple(int(c * 255) for c in rgb)


def _swatch(rgb, text):
    return ('<span style="color:{}; font-weight:bold">&#9644;</span> {}'
            .format(_html(rgb), text))


HELP_HTML = """
<p><b>Ce que montre l'outil</b></p>
<p>Chaque nervure est traitée comme un petit cerf-volant suspendu à ses
propres suspentes.  En vol, l'air pousse sur la nervure avec une seule force
résultante, appliquée à son centre de poussée.  Les suspentes ne peuvent tenir
cette force que si elle pointe droit vers le <i>sommet</i> du cône de
suspentes (l'élévateur).  Sinon la nervure veut tourner : la distance entre la
ligne de la force et le sommet du cône est le <b>bras de moment</b>.</p>
<ul>
<li><b>bras &gt; 0</b> : la force passe <i>devant</i> le sommet &rarr; la
nervure veut <i>cabrer</i> ; les A portent plus, les arrières moins.</li>
<li><b>bras &lt; 0</b> : la force passe <i>derrière</i> le sommet &rarr; la
nervure veut <i>piquer</i> ; les arrières portent plus, les A se détendent en
premier dans une abattée.</li>
</ul>
<p>Seule la <b>variation du bras le long de l'envergure</b> relève du
vrillage.  Si toutes les nervures ont le même bras, l'aile entière est
simplement réglée cabreuse ou piqueuse : ça se corrige par la position des
élévateurs ou la finesse, pas par le vrillage.  C'est pourquoi la courbe verte
est tracée <i>par rapport à la nervure centrale</i> et que le décalage commun
est donné en chiffre.</p>

<p><b>Les deux façons de corriger</b></p>
<ul>
<li><b>Vrillage</b> (changer l'AoA de la nervure) : le centre de poussée se
déplace le long de la corde et la force s'incline un peu.  C'est un levier
<i>faible</i> : il faut souvent plusieurs degrés, et la portance de la nervure
change avec.</li>
<li><b>Flèche</b> (glisser la nervure vers l'avant ou l'arrière, corde
inchangée) : déplace toute la nervure par rapport à son sommet de cône.
Direct et puissant, mais la forme en plan change.</li>
</ul>
<p>Le curseur mélange les deux.  La courbe orange est l'AoA après correction,
la forme en plan orange est la forme avec flèche.  <i>Appliquer la
proposition</i> écrit les deux dans le modèle ; OK enregistre la voile.</p>

<p><b>Objectifs de conception</b> (boîte « Objectifs ») : avec seulement
<i>équilibre</i> actif, l'outil résout chaque nervure exactement.  Activez
<i>charge elliptique</i>, <i>vrillage négatif de bout</i> ou <i>tension des
suspentes de bout</i> et le vrillage devient un compromis pondéré entre tous
les objectifs actifs (moindres carrés, nervure centrale conservée).  La part
« flèche » du curseur ne ferme toujours que le bras de moment.  Les poids sont
relatifs : 100/50 signifie que l'équilibre compte deux fois plus que l'autre
objectif.</p>

<p><b>Hypothèses</b> : forces de section issues d'une polaire « profil mince »
de chaque nervure (cambrure seule) sauf si des polaires XFoil sont chargées ;
la direction du vent vient de la finesse ; les suspentes sont droites des
points d'accrochage à l'élévateur ; les nervures sans suspentes empruntent le
cône de la nervure suspendue la plus proche.</p>
"""


class TwistTool(AoaTool):
    widget_name = "Vrillage"

    def __init__(self, obj):
        self.model = None
        self.proposal = None
        self._arm_scale = 10.0  # mm of arm per degree of the AoA grid
        self._polar_factory = None
        super().__init__(obj)

    # ------------------------------------------------------------------ #
    # widget                                                             #
    # ------------------------------------------------------------------ #
    def _relabel(self, row, text):
        """Rename a label the AoA tool created on ``row`` of the form layout."""
        item = self.layout.itemAt(row, text_field)
        if item is not None and item.widget() is not None:
            item.widget().setText(text)

    def setup_widget(self):
        super().setup_widget()  # rows 0-1: num_points / spline type; row 3: glide
        self._relabel(0, "points de contrôle")
        self._relabel(1, "type de spline")
        self._relabel(3, "finesse")
        self.QGlide.setToolTip(
            "Finesse utilisée pour la direction du vent (pente de trajectoire = "
            "atan(1 / finesse)).\nElle fixe la direction de la force de l'air sur "
            "chaque nervure."
        )
        row = 4
        span = QtGui.QFormLayout.SpanningRole

        # -- 1. legend ---------------------------------------------------- #
        legend = QtGui.QGroupBox("Courbes du graphique", self.base_widget)
        lay = QtGui.QVBoxLayout(legend)
        lines = [
            _swatch((1, 0, 0), "<b>AoA que vous dessinez</b> (déplacez les points noirs) - degrés"),
            _swatch((0, 0, 1), "<b>AoA absolu</b> : corde par rapport à l'horizontale, dans le "
                    "plan de la nervure - degrés"),
            _swatch(GREEN, "<b>bras de moment, relatif à la nervure centrale</b> - mm "
                    "(échelle plus bas). 0 = même équilibre en tangage que le centre. "
                    "Vers le haut = tendance cabreuse, vers le bas = piqueuse."),
            _swatch(ORANGE, "<b>AoA après correction</b> et, dans la forme en plan, les "
                    "<b>nervures déplacées</b> (orange clair)"),
            _swatch(CYAN, "<b>charge de section</b> Cl&middot;corde (1 = elliptique au centre) ; "
                    "cyan clair : la référence elliptique"),
            _swatch(MAGENTA, "<b>inclinaison du cône</b> : angle du cône de suspentes hors du plan "
                    "de la nervure - degrés. Non corrigé ici (voûte / écartement des "
                    "élévateurs) ; indice de tension du tissu"),
            _swatch(DARK_GREY, "<b>nervure charnière</b> (trait vertical) : angle de voûte 45&deg;. "
                    "En deçà le vrillage est un choix aérodynamique, au-delà il règle "
                    "surtout la tension des suspentes de bout"),
        ]
        for text in lines:
            lbl = QtGui.QLabel(text)
            lbl.setWordWrap(True)
            lbl.setTextFormat(QtCore.Qt.RichText)
            lay.addWidget(lbl)
        self.layout.setWidget(row, span, legend)
        row += 1

        # -- 2. diagnosis ------------------------------------------------ #
        diag = QtGui.QGroupBox("Diagnostic", self.base_widget)
        dlay = QtGui.QVBoxLayout(diag)
        self.Qinfo = QtGui.QLabel("")
        self.Qinfo.setWordWrap(True)
        self.Qinfo.setTextFormat(QtCore.Qt.RichText)
        dlay.addWidget(self.Qinfo)
        self.Qtable = QtGui.QTableWidget()
        self.Qtable.setColumnCount(9)
        self.Qtable.setHorizontalHeaderLabels(
            ["nerv.", "AoA °", "bras mm", "rel. mm", "bras % corde", "charge", "incl. °",
             "suspentes", "note"]
        )
        self.Qtable.horizontalHeader().setToolTip(
            "bras mm : bras de moment de la nervure (+ = cabreur)\n"
            "rel. mm : bras moins celui de la nervure centrale (ce que vrillage/flèche "
            "peuvent changer)\n"
            "bras % corde : bras divisé par la corde de la nervure\n"
            "charge : Cl x corde, 1 = référence elliptique au centre\n"
            "incl. ° : cône de suspentes hors du plan de nervure, + = vers le centre\n"
            "suspentes : points d'accrochage trouvés sur cette nervure, ou la nervure "
            "dont le cône est emprunté"
        )
        self.Qtable.verticalHeader().setVisible(False)
        self.Qtable.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.Qtable.setMinimumHeight(200)
        dlay.addWidget(self.Qtable)
        self.layout.setWidget(row, span, diag)
        row += 1

        # -- 3. correction ----------------------------------------------- #
        corr = QtGui.QGroupBox("Correction", self.base_widget)
        form = QtGui.QFormLayout(corr)

        self.Qmix = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.Qmix.setRange(0, 100)
        self.Qmix.setValue(100)
        self.Qmix.setTickInterval(25)
        self.Qmix.setTickPosition(QtGui.QSlider.TicksBelow)
        self.Qmix.setToolTip(
            "Comment la variation du bras en envergure est supprimée :\n"
            "à gauche (0 %) : uniquement en déplaçant les nervures (flèche) - la forme en "
            "plan change, l'AoA non\n"
            "à droite (100 %) : uniquement en vrillant les nervures - l'AoA change, la "
            "forme en plan non\n"
            "entre les deux : la part vrillage est appliquée d'abord, la flèche ferme le reste"
        )
        self.Qmix_label = QtGui.QLabel()
        mix_widget = QtGui.QWidget()
        mix_layout = QtGui.QHBoxLayout(mix_widget)
        mix_layout.setContentsMargins(0, 0, 0, 0)
        mix_layout.addWidget(QtGui.QLabel("flèche"))
        mix_layout.addWidget(self.Qmix)
        mix_layout.addWidget(QtGui.QLabel("vrillage"))
        form.addRow("corriger par", mix_widget)
        form.addRow("", self.Qmix_label)

        self.Qtarget = QtGui.QComboBox()
        self.Qtarget.addItem("équilibrer chaque nervure comme la nervure centrale", "center")
        self.Qtarget.addItem("rendre chaque nervure sans moment (bras = 0)", "zero")
        self.Qtarget.setToolTip(
            "« comme la nervure centrale » (recommandé) : ne supprime que la variation "
            "du bras en envergure - la question du vrillage.\n"
            "« sans moment » : supprime aussi le décalage commun. Le vrillage est faible "
            "pour ça et la forme en plan ne peut pas se décaler en bloc (la nervure "
            "centrale reste à x = 0) : l'outil indique alors de combien déplacer les "
            "élévateurs."
        )
        form.addRow("but", self.Qtarget)

        self.Qsmooth = QtGui.QSpinBox()
        self.Qsmooth.setRange(2, 9)
        self.Qsmooth.setValue(4)
        self.Qsmooth.setToolTip(
            "Nombre de points de contrôle de la courbe de flèche le long de l'envergure.\n"
            "Moins = bord d'attaque plus lisse mais bras résiduel plus grand ; "
            "plus = suit chaque nervure."
        )
        form.addRow("lissage de la flèche", self.Qsmooth)

        self.Qproposal = QtGui.QLabel("")
        self.Qproposal.setWordWrap(True)
        self.Qproposal.setTextFormat(QtCore.Qt.RichText)
        form.addRow(self.Qproposal)

        self.Qapply = QtGui.QPushButton("Appliquer la proposition")
        self.Qapply.setToolTip(
            "Écrit la courbe AoA orange dans la spline AoA (même nombre de points de "
            "contrôle) et la forme en plan orange dans les courbes BA/BF de la forme.\n"
            "Vous pouvez encore modifier ensuite ; rien n'est enregistré avant OK."
        )
        form.addRow(self.Qapply)
        self.layout.setWidget(row, span, corr)
        row += 1

        # -- 3b. design goals (v2) ---------------------------------------- #
        goals = QtGui.QGroupBox("Objectifs de conception du vrillage", self.base_widget)
        gform = QtGui.QFormLayout(goals)
        goals.setToolTip(
            "Poids des objectifs que le vrillage doit satisfaire. Seul « équilibre » "
            "actif = résolution exacte nervure par nervure. Tout autre objectif actif = "
            "compromis pondéré."
        )

        def weight_slider(value, tip):
            sl = QtGui.QSlider(QtCore.Qt.Horizontal)
            sl.setRange(0, 100)
            sl.setValue(value)
            sl.setToolTip(tip)
            lab = QtGui.QLabel(str(value))
            sl.valueChanged.connect(lambda v, lab=lab: lab.setText(str(v)))
            w = QtGui.QWidget()
            lay = QtGui.QHBoxLayout(w)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(sl)
            lay.addWidget(lab)
            return sl, w

        self.Qw_arm, w = weight_slider(100, "Chaque nervure équilibrée comme la nervure "
                                            "centrale (bras de moment) - l'objectif de "
                                            "répartition de charge des suspentes.")
        gform.addRow("équilibre (bras)", w)
        self.Qw_lift, w = weight_slider(0, "La charge de section Cl x corde suit une "
                                           "ellipse - traînée induite minimale.")
        gform.addRow("charge elliptique", w)
        self.Qw_wash, w = weight_slider(0, "Bouts au moins « n » degrés sous le centre et "
                                           "AoA jamais croissant vers l'extérieur - le "
                                           "décrochage commence au centre, les bouts "
                                           "continuent de voler.")
        gform.addRow("vrillage négatif de bout", w)
        self.Qwash_deg = QtGui.QDoubleSpinBox()
        self.Qwash_deg.setRange(0.0, 15.0)
        self.Qwash_deg.setValue(3.0)
        self.Qwash_deg.setSuffix(" °")
        self.Qwash_deg.setToolTip("Écart d'AoA minimal entre le centre et le bout.")
        gform.addRow("    écart centre - bout au moins", self.Qwash_deg)
        self.Qw_tens, w = weight_slider(0, "Au-delà de la nervure charnière, garder le Cl "
                                           "au-dessus du minimum pour que les suspentes de "
                                           "bout restent chargées (un bout qui ne porte "
                                           "pas se replie).")
        gform.addRow("tension des suspentes de bout", w)
        self.Qcl_min = QtGui.QDoubleSpinBox()
        self.Qcl_min.setRange(0.0, 1.5)
        self.Qcl_min.setSingleStep(0.05)
        self.Qcl_min.setValue(0.3)
        self.Qcl_min.setToolTip("Cl de section minimal au-delà de la nervure charnière.")
        gform.addRow("    Cl de bout au moins", self.Qcl_min)
        self.Qsmooth_aoa = QtGui.QDoubleSpinBox()
        self.Qsmooth_aoa.setRange(0.0, 5.0)
        self.Qsmooth_aoa.setSingleStep(0.1)
        self.Qsmooth_aoa.setValue(0.1)
        self.Qsmooth_aoa.setToolTip("Pénalise les zigzags d'AoA entre nervures voisines.")
        gform.addRow("régularité de l'AoA", self.Qsmooth_aoa)
        self.layout.setWidget(row, span, goals)
        row += 1

        # -- 4. display / options ---------------------------------------- #
        opts = QtGui.QGroupBox("Affichage et options physiques", self.base_widget)
        oform = QtGui.QFormLayout(opts)

        self.Qarm_scale = QtGui.QDoubleSpinBox()
        self.Qarm_scale.setRange(0.5, 500.0)
        self.Qarm_scale.setValue(self._arm_scale)
        self.Qarm_scale.setSuffix(" mm par degré de grille")
        self.Qarm_scale.setToolTip("Échelle verticale de la courbe verte (bras) sur la grille AoA.")
        oform.addRow("échelle du bras", self.Qarm_scale)

        self.Qshow_load = QtGui.QCheckBox("afficher la charge de section (cyan)")
        self.Qshow_load.setChecked(True)
        self.Qshow_load.setToolTip(
            "Cl x corde de chaque nervure d'après la polaire, comparé à une répartition "
            "elliptique de même total. Tracé sur la grille en « degrés » : "
            "1,0 = 10 degrés de grille."
        )
        oform.addRow(self.Qshow_load)

        self.Qshow_span = QtGui.QCheckBox("afficher l'inclinaison du cône (magenta)")
        self.Qshow_span.setChecked(False)
        oform.addRow(self.Qshow_span)

        self.Qxfoil = QtGui.QPushButton("Charger les polaires XFoil (lent)")
        self.Qxfoil.setToolTip(
            "Remplace l'estimation « profil mince » par des polaires XFoil du profil de "
            "chaque nervure (nécessite le binaire « xfoil » dans le PATH ; quelques "
            "secondes par nervure). Change Cl, Cd et le centre de poussée, donc le bras."
        )
        self.Qpolar_status = QtGui.QLabel("polaires : profil mince (ligne de cambrure)")
        oform.addRow(self.Qxfoil, self.Qpolar_status)
        self.layout.setWidget(row, span, opts)
        row += 1

        # -- 5. help ------------------------------------------------------ #
        helpbox = QtGui.QGroupBox("Comment lire cet outil", self.base_widget)
        helpbox.setCheckable(True)
        helpbox.setChecked(False)
        hlay = QtGui.QVBoxLayout(helpbox)
        self.Qhelp = QtGui.QLabel(HELP_HTML)
        self.Qhelp.setWordWrap(True)
        self.Qhelp.setTextFormat(QtCore.Qt.RichText)
        self.Qhelp.setVisible(False)
        hlay.addWidget(self.Qhelp)
        helpbox.toggled.connect(self.Qhelp.setVisible)
        self.layout.setWidget(row, span, helpbox)

        self._on_mix_moved(self.Qmix.value(), solve=False)
        self.Qmix.valueChanged.connect(self._on_mix_moved)
        self.Qmix.sliderReleased.connect(self.update_proposal)
        self.Qtarget.currentIndexChanged.connect(self.update_proposal)
        self.Qsmooth.valueChanged.connect(self.update_proposal)
        self.Qarm_scale.valueChanged.connect(self._on_scale)
        self.Qshow_load.toggled.connect(self.update_curves)
        self.Qshow_span.toggled.connect(self.update_curves)
        self.Qapply.clicked.connect(self.apply_proposal)
        self.Qxfoil.clicked.connect(self.load_xfoil_polars)
        for sl in (self.Qw_arm, self.Qw_lift, self.Qw_wash, self.Qw_tens):
            sl.sliderReleased.connect(self.update_proposal)
            sl.valueChanged.connect(self._on_weight_changed)
        for sp in (self.Qwash_deg, self.Qcl_min, self.Qsmooth_aoa):
            sp.valueChanged.connect(self.update_proposal)

    # ------------------------------------------------------------------ #
    # scene                                                              #
    # ------------------------------------------------------------------ #
    def setup_pivy(self):
        self.arm_curve = Line_old([], color=GREEN, width=2)
        self.span_curve = Line_old([], color=MAGENTA, width=1)
        self.load_curve = Line_old([], color=CYAN, width=2)
        self.load_ref_curve = Line_old([], color=LIGHT_CYAN, width=1)
        self.proposed_aoa_curve = Line_old([], color=ORANGE, width=2)
        self.hinge_line = Line_old([], color=DARK_GREY, width=1)
        self.ghost = coin.SoSeparator()
        for curve in (self.arm_curve, self.span_curve, self.load_curve,
                      self.load_ref_curve, self.proposed_aoa_curve, self.hinge_line):
            self.task_separator.addChild(curve.object)
        self.task_separator.addChild(self.ghost)
        super().setup_pivy()  # draws shape, red/blue curves, calls update_glide
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # model                                                              #
    # ------------------------------------------------------------------ #
    def rebuild_model(self):
        try:
            self.model = TwistModel(self.parametric_glider, polar_factory=self._polar_factory)
        except Exception as e:  # keep the AoA tool usable even if lines are odd
            self.model = None
            self.Qinfo.setText("<b>modèle de vrillage indisponible :</b> {}".format(e))

    def update_glide(self, *args):
        super().update_glide(*args)  # sets parametric_glider.glide, redraws aoa
        self.rebuild_model()
        if hasattr(self, "Qmix"):
            self.update_proposal()

    def update_aoa(self):
        super().update_aoa()
        if self.model is not None:
            self.model.set_aoa(self.parametric_glider.aoa)
            self.update_curves()

    def on_release(self):
        super().on_release()
        self.update_proposal()

    def update_num(self):
        super().update_num()
        self.update_proposal()

    def update_spline_type(self):
        super().update_spline_type()
        self.update_proposal()

    def load_xfoil_polars(self):
        """Swap the thin-airfoil polars for XFoil ones (blocking)."""
        import shutil

        if shutil.which("xfoil") is None:
            self.Qpolar_status.setText(
                "polaires : profil mince - binaire « xfoil » introuvable dans le PATH")
            return
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            cache = {}

            def factory(profile):
                key = id(profile)
                if key not in cache:
                    cache[key] = TabulatedPolar.from_xfoil(
                        profile, alpha_deg=list(range(-6, 19, 2)))
                return cache[key]

            self._polar_factory = factory
            self.rebuild_model()
            self.Qpolar_status.setText("polaires : XFoil, Re 2,0e6")
        except Exception as e:
            self._polar_factory = None
            self.rebuild_model()
            self.Qpolar_status.setText(
                "polaires : profil mince - échec XFoil : {}".format(e))
        finally:
            QtGui.QApplication.restoreOverrideCursor()
        self.update_proposal()

    # ------------------------------------------------------------------ #
    # proposal                                                           #
    # ------------------------------------------------------------------ #
    @property
    def mix(self):
        return self.Qmix.value() / 100.0

    def _on_mix_moved(self, value, solve=True):
        if value >= 100:
            txt = "100 % vrillage : seul l'AoA change"
        elif value <= 0:
            txt = "100 % flèche : seule la forme en plan change"
        else:
            txt = "{} % de la correction par vrillage, le reste par flèche".format(value)
        self.Qmix_label.setText(txt)
        if solve and not self.Qmix.isSliderDown():
            self.update_proposal()

    def _on_weight_changed(self, *args):
        if not any(sl.isSliderDown() for sl in
                   (self.Qw_arm, self.Qw_lift, self.Qw_wash, self.Qw_tens)):
            self.update_proposal()

    def objective(self):
        return TwistObjective(
            w_arm=self.Qw_arm.value() / 100.0,
            w_lift=self.Qw_lift.value() / 100.0,
            w_washout=self.Qw_wash.value() / 100.0,
            w_tension=self.Qw_tens.value() / 100.0,
            washout_deg=self.Qwash_deg.value(),
            cl_min=self.Qcl_min.value(),
            smooth=self.Qsmooth_aoa.value(),
        )

    def _on_scale(self, value):
        self._arm_scale = float(value)
        self.update_curves()

    def update_proposal(self, *args):
        if self.model is None:
            self.proposal = None
            self.update_curves()
            return
        try:
            self.proposal = self.model.propose(
                mix=self.mix,
                target=self.Qtarget.currentData(),
                dx_numpoints=self.Qsmooth.value(),
                objective=self.objective(),
            )
        except Exception as e:
            self.proposal = None
            self.Qproposal.setText("<b>échec de la résolution :</b> {}".format(e))
        self.update_curves()

    # ------------------------------------------------------------------ #
    # drawing                                                            #
    # ------------------------------------------------------------------ #
    def _y(self, deg_equivalent):
        """Grid y for a value expressed in 'degrees of the AoA grid'."""
        return np.radians(deg_equivalent) * self.scale[1]

    def update_curves(self, *args):
        curves = (self.arm_curve, self.span_curve, self.load_curve,
                  self.load_ref_curve, self.proposed_aoa_curve, self.hinge_line)
        if self.model is None:
            for curve in curves:
                curve.update(HIDDEN)
            self.ghost.removeAllChildren()
            return

        states = self.model.diagnose()
        ref = states[0].moment_arm
        xs = [s.x * self.scale[0] for s in self.model.stations]

        self.arm_curve.update([
            [x, self._y((st.moment_arm - ref) * 1000.0 / self._arm_scale)]
            for x, st in zip(xs, states)
        ])

        if self.Qshow_span.isChecked():
            self.span_curve.update([[x, self._y(st.spanwise_deg)] for x, st in zip(xs, states)])
        else:
            self.span_curve.update(HIDDEN)

        load, load_ref = self.model.lift_distribution(states)
        if self.Qshow_load.isChecked():
            self.load_curve.update([[x, self._y(10.0 * v)] for x, v in zip(xs, load)])
            self.load_ref_curve.update([[x, self._y(10.0 * v)] for x, v in zip(xs, load_ref)])
        else:
            self.load_curve.update(HIDDEN)
            self.load_ref_curve.update(HIDDEN)

        hinge = self.model.hinge_station()
        if hinge is not None:
            top = max(self._y(np.degrees(st.aoa_rel)) for st in states) * 1.2
            self.hinge_line.update([[hinge * self.scale[0], 0.0], [hinge * self.scale[0], top]])
        else:
            self.hinge_line.update(HIDDEN)

        self.ghost.removeAllChildren()
        self.proposed_aoa_curve.update(HIDDEN)
        if self.proposal is not None:
            pr = self.proposal
            if pr.aoa_curve is not None:
                self.proposed_aoa_curve.update(
                    pr.aoa_curve.get_sequence(num=self.num_on_drag) * self.scale
                )
            if pr.front_curve is not None:
                num = self.num_on_drag
                front = [p for p in pr.front_curve.get_sequence(num=num) if p[0] >= 0]
                back = [p for p in pr.back_curve.get_sequence(num=num) if p[0] >= 0]
                self.ghost += [Line_old(front, color=ORANGE, width=2).object]
                self.ghost += [Line_old(back, color=ORANGE, width=2).object]
                for rib in pr.ribs_2d():
                    self.ghost += [Line_old(rib, color=LIGHT_ORANGE).object]

        self._fill_table(states, load)
        self._fill_info(states, ref, hinge)
        self._fill_proposal()

    def _fill_info(self, states, ref, hinge):
        dev = [(st.moment_arm - ref) * 1000.0 for st in states]
        i_min, i_max = int(np.argmin(dev)), int(np.argmax(dev))
        if ref > 0.005:
            trim = "l'aile entière tend à <b>cabrer</b> (force devant les élévateurs)"
        elif ref < -0.005:
            trim = "l'aile entière tend à <b>piquer</b> (force derrière les élévateurs)"
        else:
            trim = "l'aile est équilibrée autour des élévateurs"
        txt = [
            "<b>Décalage commun</b> (bras de la nervure centrale) : {:+.0f} mm - {}. "
            "Cette part relève du trim en tangage (position des élévateurs / finesse), "
            "pas du vrillage.".format(ref * 1000.0, trim),
            "<b>Variation en envergure</b> (ce que vrillage ou flèche peuvent corriger) : "
            "de {:+.0f} mm à la nervure {} à {:+.0f} mm à la nervure {}. "
            "Les nervures au-dessus de la valeur centrale cabrent par rapport à elle, "
            "celles en dessous piquent.".format(dev[i_min], i_min, dev[i_max], i_max),
        ]
        if hinge is not None:
            txt.append("<b>Nervure charnière</b> à y = {:.2f} m (angle de voûte 45°) : au-delà, "
                       "le vrillage règle surtout la tension des suspentes de bout."
                       .format(hinge))
        self.Qinfo.setText("<br>".join(txt))

    def _fill_proposal(self):
        if self.proposal is None:
            self.Qproposal.setText("")
            return
        pr = self.proposal
        sol = pr.solution
        after = [(a.moment_arm - sol.target) * 1000.0 for a in pr.after]
        parts = []
        if np.any(np.abs(sol.d_aoa) > 1e-9):
            parts.append("vrillage de {:+.1f}° à {:+.1f}° (courbe AoA orange)".format(
                np.degrees(sol.d_aoa.min()), np.degrees(sol.d_aoa.max())))
        if np.any(np.abs(pr.dx_values) > 1e-4):
            parts.append("flèche de {:+.0f} mm à {:+.0f} mm, + = vers le bord de fuite "
                         "(forme en plan orange)".format(pr.dx_values.min() * 1000.0,
                                                         pr.dx_values.max() * 1000.0))
        if not parts:
            parts.append("rien à changer")
        obj = self.objective()
        if obj.active:
            goals = [n for n, w in (("équilibre", obj.w_arm),
                                    ("charge elliptique", obj.w_lift),
                                    ("vrillage négatif de bout", obj.w_washout),
                                    ("tension des suspentes de bout", obj.w_tension)) if w > 0]
            mode = "compromis pondéré entre " + ", ".join(goals)
        else:
            mode = "équilibre exact nervure par nervure"
        txt = ["<b>Proposition</b> ({}) : {}.".format(mode, " ; ".join(parts)),
               "Bras résiduel après lissage : {:.0f} mm.".format(max(abs(a) for a in after))]
        if abs(pr.riser_dx) > 1e-4:
            txt.append("Une flèche commune de {:+.0f} mm ne peut pas entrer dans la forme en "
                       "plan (la nervure centrale reste à x = 0) : <b>déplacez plutôt les "
                       "élévateurs de {:+.0f} mm</b>."
                       .format(pr.riser_dx * 1000.0, -pr.riser_dx * 1000.0))
        if not np.all(sol.reachable):
            bad = ", ".join(str(s.index) for s, ok in zip(sol.stations, sol.reachable) if not ok)
            txt.append("Le vrillage seul n'atteint pas le but sur les nervures {} (la polaire "
                       "sort de sa plage) : de la flèche a été ajoutée là.".format(bad))
        self.Qproposal.setText("<br>".join(txt))

    def _fill_table(self, states, load):
        table = self.Qtable
        stations = self.model.stations
        ref = states[0].moment_arm
        table.setRowCount(len(stations))
        for i, (s, st) in enumerate(zip(stations, states)):
            lines = ",".join(n for n, p in s.line_attachments) or "-"
            note = ""
            if s.apex_from is not None:
                lines = "aucune"
                note = "cône de la nervure {}".format(s.apex_from)
            rel = (st.moment_arm - ref) * 1000.0
            if i > 0 and abs(rel) > 5:
                note = (note + " ; " if note else "") + ("cabreur" if rel > 0 else "piqueur")
            values = [
                str(s.index),
                "{:.2f}".format(np.degrees(st.aoa_rel)),
                "{:+.0f}".format(st.moment_arm * 1000.0),
                "{:+.0f}".format(rel),
                "{:+.1f}".format(100.0 * st.moment_arm / s.chord if s.chord else 0.0),
                "{:.2f}".format(load[i]),
                "{:+.1f}".format(st.spanwise_deg),
                lines,
                note,
            ]
            for j, v in enumerate(values):
                item = QtGui.QTableWidgetItem(v)
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                table.setItem(i, j, item)
        table.resizeColumnsToContents()

    # ------------------------------------------------------------------ #
    # apply                                                              #
    # ------------------------------------------------------------------ #
    def apply_proposal(self):
        if self.proposal is None:
            return
        self.proposal.apply_to(self.parametric_glider)
        # the aoa spline was rewritten in place: rebind the control points
        self.spline_controlpoints.control_pos = (
            np.array(self.spline.controlpoints) * self.scale
        )
        self.spline_controlpoints.control_points[-1].constrained = [0.0, 1.0, 0.0]
        self.Qnum_aoa.blockSignals(True)
        self.Qnum_aoa.setValue(len(self.spline.controlpoints))
        self.Qnum_aoa.blockSignals(False)
        # shape may have changed: redraw the grey planform
        self.ribs = self.parametric_glider.shape.ribs
        self.front = [rib[0] for rib in self.ribs]
        self.back = [rib[1] for rib in self.ribs]
        self.draw_shape()
        self.rebuild_model()
        self.update_aoa()
        self.update_grid(drag_release=True)
        self.update_proposal()

    def accept(self):
        self.spline_controlpoints.remove_callbacks()
        super(AoaTool, self).accept()
