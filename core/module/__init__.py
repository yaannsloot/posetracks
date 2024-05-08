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
import json
import importlib
from typing import Union

# Temporarily add module directory to environment
_old_path = os.environ.get('PATH', '')
_init_path = os.path.dirname(os.path.abspath(__file__))
if os.name == 'nt':
    _dll_path = os.path.join(_init_path, "bin")
    os.environ['PATH'] = _init_path + os.pathsep + _dll_path + os.pathsep + _old_path
    os.add_dll_directory(_init_path)
    os.add_dll_directory(_dll_path)
else:
    # Library will be rpathed to lib dir, so no need to add to path
    os.environ['PATH'] = _init_path + os.pathsep + _old_path

# Get the current Python version
_python_version = f"cp{sys.version_info.major}{sys.version_info.minor}"

# Get the directory of the correct version of the module
_module_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), _python_version)

# Add the module directory to sys.path
sys.path.insert(0, _module_dir)

# Compiled modules
_pyc = importlib.import_module("MEPython")

# Remove the module directory from sys.path
sys.path.remove(_module_dir)

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

class Point3dIterator:
    def __init__(self, point):
        self.point = point
        self.index = 0

    def __next__(self):
        if self.index == 0:
            val = self.point.x
        elif self.index == 1:
            val = self.point.y
        elif self.index == 2:
            val = self.point.z
        else:
            raise StopIteration()
        self.index += 1
        return val


def _point3d_iter_func(self):
    return Point3dIterator(self)


def _point3d_getitem_func(self, item):
    if item == 0:
        return self.x
    elif item == 1:
        return self.y
    elif item == 2:
        return self.z
    raise IndexError()


def _point3d_setitem_func(self, item, val):
    if item == 0:
        self.x = val
    elif item == 1:
        self.y = val
    elif item == 2:
        self.z = val
    raise IndexError()


def _point3d_len_func(self):
    return 3


_pyc.Point3D.__iter__ = _point3d_iter_func
_pyc.Point3D.__getitem__ = _point3d_getitem_func
_pyc.Point3D.__setitem__ = _point3d_setitem_func
_pyc.Point3D.__len__ = _point3d_len_func

_pyc.Pointf3D.__iter__ = _point3d_iter_func
_pyc.Pointf3D.__getitem__ = _point3d_getitem_func
_pyc.Pointf3D.__setitem__ = _point3d_setitem_func
_pyc.Pointf3D.__len__ = _point3d_len_func


# Bundled model library functions
def _find_driver(drv_attr_path):
    attributes = drv_attr_path.split('.')
    result = _pyc
    try:
        for attr in attributes:
            result = getattr(result, attr)
    except AttributeError:
        return None
    return result


def _update_model_dict():
    m = {}
    for dirpath, dirnames, filenames in os.walk(_model_dir):
        for filename in filenames:
            if not filename.endswith('.json'):
                continue
            model_def_json = os.path.join(dirpath, filename)
            with open(model_def_json, 'r') as f:
                try:
                    model_def = json.load(f)
                except json.JSONDecodeError as e:
                    continue
            for category in model_def:
                cat_models = model_def[category]
                for cat_model in cat_models:
                    cat_model_path = f'{os.path.join(dirpath, cat_model)}.onnx'
                    cat_model_def = cat_models[cat_model]
                    if not os.path.exists(cat_model_path):
                        continue
                    resolved_driver = _find_driver(cat_model_def['driver'])
                    if resolved_driver is None:
                        continue
                    cat_model_def['driver'] = resolved_driver
                    cat_model_def['path'] = cat_model_path
                    if category not in m:
                        m[category] = {}
                    m[category][cat_model] = cat_model_def
    return m


models = _update_model_dict()


def _normalize(x):
    i_max = max(x)
    i_min = min(x)
    margin = i_max - i_min
    if margin == 0:
        return [1 for a in x]
    return [(a - i_min) / margin for a in x]


def _is_a_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _score_priority(cat_models, sorting_criteria):
    if sorting_criteria is None or not cat_models:
        return
    cat_models = list(cat_models.values())
    criteria_scores = []
    if isinstance(sorting_criteria, str):
        sorting_criteria = [sorting_criteria]
    for criteria in sorting_criteria:
        if not all(criteria in model and _is_a_number(model[criteria]) for model in cat_models):
            continue
        scores = _normalize([model[criteria] for model in cat_models])
        if criteria.startswith('precision'):
            scores = [1 - x for x in scores]
        criteria_scores.append(scores)
    if len(criteria_scores) == 0:
        return
    return _normalize([sum(all_values) for all_values in zip(*criteria_scores)])


def _compare_values(model, key, value):
    if key not in model:
        return False
    if isinstance(value, list):
        if not isinstance(model[key], list):
            return False
        return all(v in model[key] for v in value)
    return model[key] == value


def get_models(model_category, attributes=None, values=None, sorting_criteria=None):
    if model_category not in models:
        return
    cat_models = models[model_category]
    if attributes is not None:
        if isinstance(attributes, str):
            attributes = [attributes]
        cat_models = {k: v for (k, v) in cat_models.items() if all(attr in v for attr in attributes)}
    if values is not None:
        cat_models = {k: v for (k, v) in cat_models.items() if
                      all(_compare_values(v, kv, vv) for (kv, vv) in values.items())}
    if not cat_models:
        return
    scores = _score_priority(cat_models, sorting_criteria)
    cat_models = list(cat_models.items())
    if scores is not None:
        return [model for _, model in sorted(zip(scores, cat_models), key=lambda pair: pair[0])]
    return cat_models


# Merge compiled module namespace with globals
for name in dir(_pyc):
    if not name.startswith('_'):
        globals()[name] = getattr(_pyc, name)
