#! /usr/bin/python2
#
# (c) 2013 booya (http://booya.at)
#
# This file is part of the OpenGlider project.
#
# OpenGlider is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# OpenGlider is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OpenGlider.  If not, see <http://www.gnu.org/licenses/>.

import inspect
import json
import html

from openglider.utils.cache import recursive_getattr
import openglider.jsonify
from openglider.utils.table import Table


def sign(val):
    val = float(val)
    return (val > 0) - (val < 0)


def consistent_value(elements, attribute) -> list:
    import numpy as np
    vals = [recursive_getattr(element, attribute) for element in elements]
    
    # Check if all values are equal (handles numpy arrays with tolerance for float precision)
    def values_equal(v1, v2):
        if isinstance(v1, np.ndarray) and isinstance(v2, np.ndarray):
            # Use allclose with tolerance for floating point comparison
            return np.allclose(v1, v2, rtol=1e-9, atol=1e-12)
        if isinstance(v1, (list, tuple)) and isinstance(v2, (list, tuple)):
            if len(v1) != len(v2):
                return False
            return np.allclose(v1, v2, rtol=1e-9, atol=1e-12)
        return v1 == v2
    
    all_equal = all(values_equal(v1, v2) for v1, v2 in zip(vals[:-1], vals[1:]))
    if all_equal:
        return vals[0]

    # Provide better diagnostic info
    element_names = [getattr(e, 'name', str(i)) for i, e in enumerate(elements)]
    val_lengths = [len(v) if hasattr(v, '__len__') else 'N/A' for v in vals]
    
    # Show first differing values for arrays
    diff_info = ""
    if len(vals) >= 2 and hasattr(vals[0], '__iter__') and hasattr(vals[1], '__iter__'):
        for i, (v1, v2) in enumerate(zip(vals[0], vals[1])):
            if v1 != v2:
                diff_info = f", first diff at index {i}: {v1:.6f} vs {v2:.6f}"
                break
    
    raise Exception(f"values not consistent for '{attribute}' across elements {element_names}, lengths: {val_lengths}{diff_info}")


def linspace(start, stop, count):
    return [start + y / (count - 1) * (stop - start) for y in range(count)]


# list_lengths = [len(l) for l in lists]
# list_lengths_set = set(list_lengths)
# list_length = list_lengths[0]
# assert len(list_lengths_set) == 1
# assert list_length > len(lists)
# self.lists = lists
class ZipCmp:
    def __init__(self, list):
        self.list = list

    def __iter__(self):
        for x, y in zip(self.list[:-1], self.list[1:]):
            yield x, y


class dualmethod:
    """
    A Decorator to have a combined class-/instancemethod

    >>>class a:
    ...    @dualmethod
    ...    def test(this):
    ...        return this
    ...
    >>>a.test()
    <class '__main__.a'>
    >>>a().test()
    <__main__.a object at 0x7f133b5f7198>
    >>>

    an instance-check could be:

        is_instance = not type(this) is type
    """

    def __init__(self, func):
        self.func = func

    def __get__(self, obj, cls=None):
        obj = obj or cls
        is_instance = not type(obj) is type

        def temp(*args, **kwargs):
            return self.func(obj, *args, **kwargs)

        return temp


class Config:
    def __init__(self, dct=None):
        self.__dict__ = {}
        items = inspect.getmembers(self.__class__, lambda a: not (inspect.isroutine(a)))
        for key, value in items:
            if not key.startswith("_") and key != "get":
                self.__dict__[key] = value

        self.update(dct)

    def __json__(self):
        return {"dct": self.__dict__}

    def __repr__(self):
        repr_str = f"{self.__class__}\n"
        width = max([len(x) for x in self.__dict__])
        for key, value in self.__dict__.items():
            repr_str += "    -{0: <{width}} -> {value}\n".format(
                key, value=value, width=width
            )

        return repr_str

    def _repr_html_(self):
        html_str = """<table>\n"""
        for key, value in self.__dict__.items():
            html_str += f"""    <tr>
                <td>{key}</td>
                <td>{html.escape(repr(value))}</td>
                </tr>
            """

        html_str += "</table>"

        return html_str

    def __iter__(self):
        for key, value in self.__dict__.items():
            if key != "get":
                yield key, value
        # return self.__dict__.__iter__()

    def __getitem__(self, item):
        return self.__getattribute__(item)

    def get(self, key, default=None):
        if hasattr(self, key):
            return self.__getattribute__(key)
        else:
            return default

    def update(self, dct):
        if dct is None:
            return

        self.__dict__.update(dct)

    def write(self, filename):
        with open(filename, "w") as jsonfile:
            openglider.jsonify.dump(self, jsonfile)

    @classmethod
    def read(cls, filename):
        with open(filename) as jsonfile:
            data = json.load(jsonfile)

        return cls(data["data"]["data"]["dct"])
