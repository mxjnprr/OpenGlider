# OpenGlider Architecture

## 1. Executive Summary
OpenGlider is a comprehensive software tool for paraglider design. It is built as a hybrid system: a core mathematical and geometric Python engine (`openglider`) that operates independently, and a UI layer deeply integrated into FreeCAD (`freecad.glider`). The core engine handles 3D mesh modeling (MeshPy), aerodynamic solvers (parabem), parametric shaping (Cord Cut Billow, Mini Ribs, Vertical Winglets), and 2D development patterns for manufacturing (PlotMaker). 

## 2. Technology Stack
- **Languages**: Python 3.8+
- **Core Integrations**: [FreeCAD](https://www.freecadweb.org/) >= 1.0.0
- **Geometry & Math**: `svgwrite`, `ezdxf`, `ezodf`, `meshpy`, `lxml`, `pyexcel-ods3`, `matplotlib`, `numpy`, `scipy`, `parabem`
- **Environment Management**: [Pixi](https://prefix.dev)
- **Quality Assurance**: `pylint`, custom testing suite (`testall.py`)

## 3. Architecture Pattern
**Plugin/Library Hybrid Monolith**
The project code is divided into a headless geometric engine and a host-application (FreeCAD) binding. 
- **Headless Domain Logic**: The geometry representations (cells, ribs, mesh, splines, 2D plots) are fully decoupled from the UI.
- **Host Integration (FreeCAD)**: The `freecad.glider` module provides custom ViewProviders, tool commands, and property data structures that map the core OpenGlider objects into the FreeCAD document hierarchy.

## 4. Component Overview
- **Parametric Glider**: Central schema definitions controlling dimensions, profiles, canopy arching, and ballooning.
- **3D Shaping Mechanisms**: Utilizes the 'Sewing Dart Principle'. Includes Cord Cut Billow (CCB), Partial Ribs, and Winglet extensions. Flat 2D mapping is calculated using high-fidelity 3D mesh distance and triangulation constraints.
- **Aerodynamics Analysis (Panel Method)**: Invokes `parabem` to run CFD solving over the internal mesh representation, calculating pressure distributions (Cp) and polars.
- **Manufacturing Flattening (`PlotMaker`)**: Translates 3D representations into flat 2D SVG/DXF cut paths, introducing seam allowances, split edges (LE/TE), and structural cutouts.

## 5. Source Tree
_(See [source-tree-analysis.md](./source-tree-analysis.md) for full details)_
- `openglider/`: Core domain logic
- `freecad/glider/`: CAD UI bindings
- `scripts/`/`tests/`: Utility and quality checks

## 6. Testing Strategy
- Validation tests are located in `tests/` and run collectively using the `testall.py` script. Pixi automates testing using `pixi run test` and `pixi run lint`.
