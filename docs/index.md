# Project Documentation Index

## Project Overview
- **Type:** monolith
- **Primary Language:** Python
- **Architecture:** Plugin/Library Hybrid

## Quick Reference
- **Tech Stack:** Python 3.8+, FreeCAD, Pixi
- **Entry Point:** `freecad/glider/init_gui.py` and `openglider/__init__.py`
- **Architecture Pattern:** Decoupled Core Logic + FreeCAD GUI Bindings

## Generated Documentation
- [Project Overview](./project-overview.md)
- [Architecture](./architecture.md)
- [Source Tree Analysis](./source-tree-analysis.md)
- [Development Guide](./development-guide.md)

## Existing Documentation
- [README.md](../README.md) - Présentation générale
- [INSTALL.md](../INSTALL.md) - Instructions d'installation
- [TODO.md](../TODO.md) - Historique/Tâches
- [TOOLS.md](./TOOLS.md) - Description des outils
- [freecad_glider_docs/](./freecad_glider_docs/) - Documentation technique
- [lines_doc/](./lines_doc/) - Documentation Suspentes
- [panel_method/](./panel_method/) - Documentation Aérodynamique
- [sphinx/](./sphinx/) - Docs Sphinx

## Getting Started
See the [Development Guide](./development-guide.md) for environment setup with Pixi. Import the module dynamically in FreeCAD or link it into your `FREECAD_USER_DIR/Mod`.
