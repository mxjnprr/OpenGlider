# Install backward-compatibility import aliases for old .FCStd files
# (module renamed from freecad.freecad_glider -> freecad.glider).
from ._compat import install as _install_compat

_install_compat()
