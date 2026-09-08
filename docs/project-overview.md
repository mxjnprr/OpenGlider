# OpenGlider Project Overview

## Project Purpose
OpenGlider is an advanced open-source toolkit for designing paragliders and kites. It provides powerful algorithms for generating 3D canopy models, applying advanced shaping parameters (like Cord Cut Billows and Winglets), evaluating aerodynamic characteristics using a Panel Method solver, and outputting 2D flatten templates ready for precision laser or CNC manufacturing.

## Executive Summary
This repository contains a unified Monolith structure composed of a core Python geometry library and its corresponding FreeCAD plugin wrapper. By separating concerns, the mathematical algorithms remain testable and extensible without relying heavily on the FreeCAD graphical context.

## Quick Reference
- **Repository Structure**: Monolith (1 cohesive part)
- **Project Type**: Library / CAD Extension
- **Primary Language**: Python 3.8+
- **Key Paradigms**: Mesh-Based Triangulation, Panel Method Aerodynamics, FreeCAD FeaturePython integration.

## Documentation Index
- [Architecture](./architecture.md)
- [Source Tree Analysis](./source-tree-analysis.md)
- [Development Guide](./development-guide.md)
- *Existing Docs: [TOOLS.md](./TOOLS.md)*
