'''
Copyright (C) 2023 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import os
import sys
import importlib

# Temporarily add module directory to environment
_old_path = os.environ.get('PATH', '')
_init_path = os.path.dirname(os.path.abspath(__file__))
_dll_path = os.path.join(_init_path, "bin")
os.environ['PATH'] = _init_path + os.pathsep + _dll_path + os.pathsep + _old_path
os.add_dll_directory(_init_path)
os.add_dll_directory(_dll_path)

# Get the current Python version
_python_version = f"cp{sys.version_info.major}{sys.version_info.minor}"

# Get the directory of the correct version of the module
_module_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), _python_version)

# Add the module directory to sys.path
sys.path.insert(0, _module_dir)

# Compiled modules
_pyc = importlib.import_module("MEPython")

# Remove the module directory from sys.path
sys.path.pop(0)
sys.path.pop(0)

_model_dir = os.path.join(_init_path, "models")


# General functions, classes, and constants

def model_path(model_file):
    return os.path.join(_model_dir, model_file)


# Pythonic extensions to native class bindings

# Base
class PointIterator:
    def __init__(self, point):
        self.point = point
        self.index = 0

    def __next__(self):
        if self.index == 0:
            val = self.point.x
        elif self.index == 1:
            val = self.point.y
        else:
            raise StopIteration()
        self.index += 1
        return val


def _point_iter_func(self):
    return PointIterator(self)


def _point_getitem_func(self, item):
    if item == 0:
        return self.x
    elif item == 1:
        return self.y
    raise IndexError()


def _point_setitem_func(self, item, val):
    if item == 0:
        self.x = val
    elif item == 1:
        self.y = val
    raise IndexError()


def _point_len_func(self):
    return 2


_pyc.Point.__iter__ = _point_iter_func
_pyc.Point.__getitem__ = _point_getitem_func
_pyc.Point.__setitem__ = _point_setitem_func
_pyc.Point.__len__ = _point_len_func

_pyc.Pointf.__iter__ = _point_iter_func
_pyc.Pointf.__getitem__ = _point_getitem_func
_pyc.Pointf.__setitem__ = _point_setitem_func
_pyc.Pointf.__len__ = _point_len_func


# Merge compiled module namespace with globals
for name in dir(_pyc):
    if not name.startswith('_'):
        globals()[name] = getattr(_pyc, name)
