# Source Tree Analysis

## Overview
OpenGlider is structured as a single cohesive project containing a core numerical/geometry library (`openglider`) and a FreeCAD GUI integration layer (`freecad.glider`).

```text
OpenGlider/
├── openglider/          # Core library (Geometry, math, design logic)
│   ├── airfoil/         # Airfoil processing and coordinates
│   ├── glider/          # Main glider structure models (parametric, cells, ribs)
│   ├── jsonify/         # JSON serialization, migration
│   ├── lines/           # Suspension line layout calculation
│   ├── mesh/            # 3D mesh generation (MeshPy wrapping)
│   ├── plots/           # 2D rendering, SVG export, spreadsheets export
│   └── vector/          # Vector math and splines utilities
├── freecad/             # FreeCAD module integration
│   └── glider/          # OpenGlider Workbench plugin
│       ├── tools/       # FreeCAD interactive tools (commands, property sheets)
│       └── ui/          # UI files (Qt Designer)
├── scripts/             # Standalone scripts and utilities
├── tests/               # Unit and integration tests
├── docs/                # Project documentation
└── pyproject.toml       # Python project metadata and dependencies
```

## Critical Folders

### `openglider/`
This is the math and logic engine. It uses no FreeCAD-specific dependencies and can be run headlessly for simulation or geometric generation. Important submodules include `plots` for 2D flattening/drawing generation, and `mesh` for 3D topological constructions.

### `freecad/glider/`
The UI binding layer. Connects the `openglider` core objects to FreeCAD document objects (`App::FeaturePython`) and provides custom ViewProviders (`Gui::ViewProviderDocumentObject`) and workbench tools.

### `tests/`
Contains the test suite executed by `testall.py`.
