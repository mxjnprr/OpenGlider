# Development Guide

## Prerequisites
- Python 3.8+
- [Pixi](https://prefix.dev/) (Package Manager)
- FreeCAD (>= 1.0.0)

## Installation & Setup
To set up the development environment, clone the repository and use Pixi to install the dependencies:
```bash
git clone https://github.com/booya-at/OpenGlider.git
cd OpenGlider
pixi install
```

## Project Structure
OpenGlider consists of the headless `openglider` math/geometry engine and the `freecad.glider` GUI workbench. Ensure your FreeCAD installation can import the package by linking or placing it inside `FREECAD_USER_DIR/Mod`.

## Useful Commands
The `pixi.toml` provides easy task endpoints:
- **Testing**: `pixi run test` (runs `testall.py`)
- **Linting**: `pixi run lint` (runs `pylint` against all `.py` files)
