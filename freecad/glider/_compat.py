"""Backward-compatibility shims for old .FCStd files.

The FreeCAD workbench package used to be named ``freecad.freecad_glider``.
It was later renamed to ``freecad.glider``.  Documents saved with the old
version store the fully-qualified module path of their Python proxies
(``OGGlider`` / ``OGGliderVP``) inside Document.xml / GuiDocument.xml, e.g.::

    module="freecad.freecad_glider.tools.glider" class="OGGlider"

When such a document is opened, FreeCAD tries to import that module to
reconstruct the proxy and fails with::

    ModuleNotFoundError: No module named 'freecad.freecad_glider'

We install a meta path finder that transparently redirects any import of
``freecad.freecad_glider`` (and its submodules) to the equivalent module
under ``freecad.glider``.  This keeps old files loadable without having to
rewrite them.
"""

import importlib
import importlib.abc
import importlib.util
import sys

_OLD = "freecad.freecad_glider"
_NEW = "freecad.glider"


class _GliderCompatFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Redirect ``freecad.freecad_glider*`` imports to ``freecad.glider*``."""

    def find_spec(self, fullname, path=None, target=None):
        if fullname == _OLD or fullname.startswith(_OLD + "."):
            return importlib.util.spec_from_loader(fullname, self)
        return None

    def create_module(self, spec):
        new_name = _NEW + spec.name[len(_OLD):]
        module = importlib.import_module(new_name)
        # Alias under the old name so subsequent lookups short-circuit here.
        sys.modules[spec.name] = module
        return module

    def exec_module(self, module):
        # Nothing to execute: the real module is already fully initialised.
        pass


def install():
    """Install the compatibility finder (idempotent)."""
    if not any(isinstance(f, _GliderCompatFinder) for f in sys.meta_path):
        sys.meta_path.append(_GliderCompatFinder())
