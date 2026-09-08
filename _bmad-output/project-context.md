---
project_name: 'OpenGlider'
user_name: 'Max'
date: '2026-03-21T18:34:54+01:00'
sections_completed: ['technology_stack', 'language_rules', 'framework_rules', 'testing_rules', 'quality_rules', 'workflow_rules', 'anti_patterns']
status: 'complete'
rule_count: 22
optimized_for_llm: true
---

# Project Context for AI Agents

_This file contains critical rules and patterns that AI agents must follow when implementing code in this project. Focus on unobvious details that agents might otherwise miss._

---

## Technology Stack & Versions

**Environnement cible et UI :**
- UI / Intégration : FreeCAD (>=1.0.0) -> *Critique : L'API FreeCAD dicte les versions compatibles des autres bibliothèques.*

**Langage et Outils de Calcul :**
- Python (>=3.8)
- Math & Géométrie : Numpy, Scipy, Parabem, ezdxf, ezodf, meshpy, svgwrite

**Build & Environnement :**
- Gestionnaire d'environnement système : Pixi (`pixi.toml`)
- Build backend : `setuptools` (`pyproject.toml`)

**Test & Qualité :**
- Framework : `unittest` exécuté **exclusivement** via le script `python testall.py` (ne pas invoquer pytest).
- Linter : `pylint` (Standard PEP8)

*Note : Projet purement orienté ingénierie/CAO. Aucun framework Web.*

## Critical Implementation Rules

### Language-Specific Rules

- **Compatibilité** : Le code doit rester compatible avec Python >= 3.8. C'est critique car FreeCAD embarque parfois des environnements Python plus anciens. N'utilisez pas de fonctionnalités exclusives à Python 3.10+ (ex: le pattern matching `match/case`).
- **Imports et structures** : Utilisez des imports absolus clairs depuis la racine (`import openglider.vector`). Le code est scindé entre la logique pure (`openglider/`) et les macros GUI (`freecad/`). Ne mélangez jamais les imports GUI dans la logique métier.
- **Gestion des erreurs géométriques** : Lors de calculs géométriques complexes (maillage, scipy, ezdxf), ne masquez pas les exceptions avec un simple `except Exception: pass`. Faites remonter des erreurs explicites pour faciliter le débogage sous FreeCAD.

### Règles spécifiques au Framework (FreeCAD Workbench)

- **Séparation stricte UI/Logique** : Ne placez aucun calcul métier ou aérodynamique dans le module `freecad/`. Ce dossier est réservé exclusivement à l'interface graphique (Commandes) et aux objets de l'arbre FreeCAD (FeaturePython, ViewProvider). Toute la logique de conception de voile (mesh, calculs) va dans `openglider/`.
- **Propriétés des objets paramétriques** : L'état d'un voile est sérialisé via des propriétés paramétriques FreeCAD. Si vous ajoutez une fonctionnalité, vous devez déclarer ses paramètres dans la classe métier ET les lier à l'API système C++ de FreeCAD (`App.PropertyFloat`, etc.).
- **Calculs et Recomputations** : Les mises à jour de vue 3D dans FreeCAD (`execute`) sont coûteuses. Ne déclenchez pas de re-calculs intempestifs lors d'updates mineurs (ou documentés pour le faire manuellement par l'utilisateur).

### Règles de Test (Testing Rules)

- **Structure et Convention de Naming** : Tous les tests unitaires et visuels doivent se trouver dans le répertoire `tests/`. Le Pattern de détection est strict, les fichiers de test doivent se nommer `test*.py` (ex: `test_profile.py`).
- **Lanceur imposé** : L'exécution de la suite entière ou partielle *doit se faire exclusivement via le script racine* `python testall.py`. Les agents ne doivent pas suggérer d'utiliser des outils CLI externes comme `pytest` ou `nose`.
- **Mocking et Indépendance** : Pour les tests de logique métier (`openglider/`), utiliser `unittest.mock` lorsque c'est pertinent afin de ne pas déclencher le moteur C++ interne de FreeCAD inutilement (accélère l'exécution). Les tests visuels (`-a`) sont gérés à part.

### Règles de Qualité et Style de Code

- **Linting et Formatage** : Le projet repose sur `pylint` (règles de base PEP8). Ne proposez pas de reformatage massif avec des outils comme `black` ou de l'auto-formatting agressif qui détruirait l'historique git. Conformez-vous strictement au style existant et corrigez simplement les avertissements pylint au fur et à mesure.
- **Organisation du code métier** : Les modules géométriques et mathématiques doivent rester dans `openglider/`, classés par abstraction (`airfoil/`, `mesh/`, `vector/`).
- **Conventions de Nommage** : Utilisez le `snake_case` pour les fonctions et variables, et le `PascalCase` pour les classes. Ne mélangez pas les conventions.
- **Documentation métier** : Étant donné la complexité mathématique du projet CAO/voile paramétrique, les ajouts critiques d'algorithmes (splines, lofting, mesh) doivent inclure de solides docstrings expliquant *l'intention* géométrique de la fonction.

### Règles de Flux de travail (Development Workflow)

- **Isolation des fonctionnalités** : Chaque fonctionnalité ou algorithme majeur lié à la conception (ex: winglets, single skin) doit être implémenté et testé de manière isolée dans `openglider/` avant toute tentative d'intégration dans l'interface ou les documents `freecad/`.
- **Validation stricte** : Toute pull request, modification ou "Code Review" générée par l'IA doit garantir que la commande `python testall.py` passe localement à 100%. Aucune régression géométrique n'est tolérée.
- **Déploiement et Gestion des Packages** : La mise à jour des métadonnées du projet (version, dépendances) se fait symétriquement : dans `pyproject.toml` (pour le package standard/PyPI) ET dans `pixi.toml` (pour conda-forge/Pixi). Synchronisez ces modifications.

### Règles Critiques "À ne pas manquer" (Don't-Miss Rules)

- **Calcul des profils géométriques** : En aérodynamique paramétrique, les profils fermés nécessitent une normalisation spécifique au niveau du bord de fuite (Trailing Edge = TE). Ne supprimez ou n'optimisez jamais la logique fermant le TE de l'`airfoil`, sous peine de briser la génération volumique des nervures (ribs).
- **Gestion de la Symétrie** : Les calculs liés au "ballooning" (gonflement) ou au découpage (`Cord Cut Billow`) doivent gérer la disposition en éventail. Portez une attention stricte à la gestion de la cellule centrale (X=0) pour éviter l'asymétrie. N'écrivez jamais de boucles naïves sur les nervures sans vérifier la règle du nombre de cellules.
- **Vecteurs de l'API FreeCAD** : Ne présumez jamais que l'objet `Base.Vector` se comporte implicitement comme une liste ou un tableau Numpy natif. Utilisez le mapping/wrapper existant dans `openglider.vector` si vous passez de Scipy/Numpy vers la CAO FreeCAD.
- **Export 2D** : Les fonctions d'export (PlotMaker, layouts) sont très sensibles aux `IndexError` liés aux mismatchs de tailles de profils (ex: SingleSkinRib vs Rib normal).

---

## Usage Guidelines

**For AI Agents:**

- Read this file before implementing any code
- Follow ALL rules exactly as documented
- When in doubt, prefer the more restrictive option
- Update this file if new patterns emerge

**For Humans:**

- Keep this file lean and focused on agent needs
- Update when technology stack changes
- Review quarterly for outdated rules
- Remove rules that become obvious over time

Last Updated: 2026-03-21T18:34:54+01:00
