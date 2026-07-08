"""Reference background image for editing tools.

Displays an arbitrary image on a textured quad lying in the Z=0 plane (the same
plane the Arc tool draws in, i.e. the glider seen from the front). The image can
be moved, rotated and scaled both from the task panel and directly in the 3D
view through two draggable handles:

    * a *center* handle  -> moves the image
    * a *corner* handle  -> rotates + scales the image (like an image editor)

The image (and its transform / opacity) can be embedded into the FreeCAD
document so it is restored when the project is reopened.
"""

import math

from pivy import coin
from pivy.graphics import InteractionSeparator, Marker
from PySide import QtGui

# names of the FreeCAD properties used to persist the reference image
_PROP_GROUP = "arc_background"
_PROPS = {
    "file": ("BGImageFile", "App::PropertyFileIncluded", "background reference image"),
    "pos_x": ("BGImagePosX", "App::PropertyFloat", "background image x position"),
    "pos_y": ("BGImagePosY", "App::PropertyFloat", "background image y position"),
    "scale": ("BGImageScale", "App::PropertyFloat", "background image scale"),
    "angle": ("BGImageAngle", "App::PropertyFloat", "background image rotation [deg]"),
    "opacity": ("BGImageOpacity", "App::PropertyFloat", "background image opacity"),
    "width": ("BGImageWidth", "App::PropertyFloat", "background image base width"),
    "aspect": ("BGImageAspect", "App::PropertyFloat", "background image height/width"),
}


class BackgroundImage:
    def __init__(self, render_manager, default_width=2.0, on_change=None):
        """
        render_manager: SoRenderManager used to register the drag handles
        default_width:  base width (scene units) of a freshly loaded image
        on_change:      optional callback fired whenever the transform/opacity
                        changes (so a panel can refresh its widgets)
        """
        self.rm = render_manager
        self.default_width = default_width or 2.0
        self.on_change = on_change

        # editable state
        self.path = ""
        self.pos = [0.0, 0.0]
        self.scale = 1.0
        self.angle = 0.0  # degrees
        self.opacity = 0.5
        # base half-dimensions of the (unscaled) quad, set when an image loads
        self._hw = self.default_width / 2.0
        self._hh = self.default_width / 2.0

        # --- picture scene graph -------------------------------------------
        self.root = coin.SoSeparator()
        self.root.setName("bg_image")

        light_model = coin.SoLightModel()
        light_model.model = coin.SoLightModel.BASE_COLOR  # show texture unshaded

        self.transform = coin.SoTransform()
        self.material = coin.SoMaterial()
        self.material.diffuseColor = (1.0, 1.0, 1.0)
        self.material.transparency = 1.0 - self.opacity

        self.texture = coin.SoTexture2()
        self.texture.model = coin.SoTexture2.MODULATE

        self.tex_coords = coin.SoTextureCoordinate2()
        self.tex_coords.point.setValues(0, 4, [(0, 0), (1, 0), (1, 1), (0, 1)])

        self.coords = coin.SoCoordinate3()
        self.face_set = coin.SoFaceSet()
        self.face_set.numVertices.setValue(4)

        self.root += [
            light_model,
            self.transform,
            self.material,
            self.texture,
            self.tex_coords,
            self.coords,
            self.face_set,
        ]

        # --- interactive handles -------------------------------------------
        self.handles = InteractionSeparator(self.rm)
        self._updating = False  # guard against drag/programmatic feedback loops
        self.center_marker = Marker([[0.0, 0.0, 0.0]], dynamic=True)
        self.corner_marker = Marker([[self._hw, self._hh, 0.0]], dynamic=True)
        self.center_marker.on_drag.append(self._on_center_drag)
        self.corner_marker.on_drag.append(self._on_corner_drag)
        self.center_marker.on_drag_release.append(self._notify)
        self.corner_marker.on_drag_release.append(self._notify)
        self.handles += [self.center_marker, self.corner_marker]
        self.handles.register()

        self._update_quad()
        self._apply_transform()
        self.set_handles_visible(False)

    # ------------------------------------------------------------------ scene
    def add_to(self, separator):
        """Insert the image behind everything already in ``separator`` and add
        the drag handles on top."""
        separator.insertChild(self.root, 0)
        separator += self.handles

    def remove_callbacks(self):
        self.handles.unregister()

    def set_handles_visible(self, visible):
        # an empty coordinate list makes the markers disappear
        if visible and self.path:
            self._refresh_handles()
        else:
            self.center_marker.points = [[0.0, 0.0, 0.0]]
            self.corner_marker.points = [[0.0, 0.0, 0.0]]

    # -------------------------------------------------------------- image i/o
    def load(self, path):
        image = QtGui.QImage(path)
        if image.isNull():
            return False
        self.path = path
        w, h = image.width(), image.height()
        aspect = (h / w) if w else 1.0
        self._hw = self.default_width / 2.0
        self._hh = self._hw * aspect
        self.texture.filename = path
        self._update_quad()
        self._apply_transform()
        self.set_handles_visible(True)
        self._notify()
        return True

    def clear(self):
        self.path = ""
        self.texture.filename = ""
        self.coords.point.setNum(0)
        self.face_set.numVertices.setValue(0)
        self.set_handles_visible(False)
        self._notify()

    def has_image(self):
        return bool(self.path)

    # ---------------------------------------------------------------- setters
    def set_position(self, x, y):
        self.pos = [x, y]
        self._apply_transform()

    def set_scale(self, scale):
        self.scale = max(scale, 1e-4)
        self._apply_transform()

    def set_angle(self, angle_deg):
        self.angle = angle_deg
        self._apply_transform()

    def set_opacity(self, opacity):
        self.opacity = min(max(opacity, 0.0), 1.0)
        self.material.transparency = 1.0 - self.opacity

    # ------------------------------------------------------------- internals
    def _update_quad(self):
        hw, hh = self._hw, self._hh
        self.coords.point.setValues(
            0,
            4,
            [(-hw, -hh, 0), (hw, -hh, 0), (hw, hh, 0), (-hw, hh, 0)],
        )
        self.face_set.numVertices.setValue(4)

    def _apply_transform(self):
        self.transform.translation = (self.pos[0], self.pos[1], 0.0)
        self.transform.rotation.setValue(
            coin.SbVec3f(0, 0, 1), math.radians(self.angle)
        )
        self.transform.scaleFactor = (self.scale, self.scale, 1.0)
        if not self._updating:
            self._refresh_handles()

    def _refresh_handles(self):
        """Reposition the handles to match the current transform."""
        if not self.path:
            return
        cx, cy = self.pos
        a = math.radians(self.angle)
        # top-right corner of the quad in world space
        lx, ly = self.scale * self._hw, self.scale * self._hh
        wx = cx + lx * math.cos(a) - ly * math.sin(a)
        wy = cy + lx * math.sin(a) + ly * math.cos(a)
        self.center_marker.points = [[cx, cy, 0.0]]
        self.corner_marker.points = [[wx, wy, 0.0]]

    def _on_center_drag(self):
        p = self.center_marker.points[0]
        self._updating = True
        self.pos = [p[0], p[1]]
        self._apply_transform()  # keeps corner in sync via _refresh_handles
        self._refresh_handles()
        self._updating = False
        self._notify()

    def _on_corner_drag(self):
        p = self.corner_marker.points[0]
        cx, cy = self.pos
        vx, vy = p[0] - cx, p[1] - cy
        ref = math.hypot(self._hw, self._hh)
        dist = math.hypot(vx, vy)
        self._updating = True
        if ref > 1e-9:
            self.scale = max(dist / ref, 1e-4)
        # angle between the reference corner (hw, hh) and the dragged vector
        self.angle = math.degrees(
            math.atan2(vy, vx) - math.atan2(self._hh, self._hw)
        )
        self._apply_transform()
        self._updating = False
        self._notify()

    def _notify(self):
        if self.on_change and not self._updating:
            self.on_change()

    # ------------------------------------------------------------ persistence
    def _ensure_props(self, obj):
        for _, (name, ptype, doc) in _PROPS.items():
            if name not in obj.PropertiesList:
                obj.addProperty(ptype, name, _PROP_GROUP, doc)

    def save_to(self, obj):
        self._ensure_props(obj)
        obj.BGImageFile = self.path or ""
        obj.BGImagePosX = float(self.pos[0])
        obj.BGImagePosY = float(self.pos[1])
        obj.BGImageScale = float(self.scale)
        obj.BGImageAngle = float(self.angle)
        obj.BGImageOpacity = float(self.opacity)
        obj.BGImageWidth = float(self._hw * 2.0)
        obj.BGImageAspect = float(self._hh / self._hw) if self._hw else 1.0

    def load_from(self, obj):
        if "BGImageFile" not in obj.PropertiesList:
            return False
        path = obj.BGImageFile
        if not path:
            return False
        if obj.BGImageWidth:
            self.default_width = obj.BGImageWidth
        self.pos = [obj.BGImagePosX, obj.BGImagePosY]
        self.scale = obj.BGImageScale or 1.0
        self.angle = obj.BGImageAngle
        if "BGImageOpacity" in obj.PropertiesList and obj.BGImageOpacity:
            self.opacity = obj.BGImageOpacity
        loaded = self.load(path)
        if loaded and obj.BGImageAspect:
            # restore exact aspect/width the image was saved with
            self._hw = (obj.BGImageWidth or self.default_width) / 2.0
            self._hh = self._hw * obj.BGImageAspect
            self._update_quad()
        self.set_opacity(self.opacity)
        self._apply_transform()
        return loaded
