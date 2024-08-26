'''
Copyright (C) 2024 Ian Sloat

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
import importlib

current_dir = os.path.dirname(__file__) if __file__ else '.'

modules = [file for file in os.listdir(current_dir) if file.endswith('.py') and file != '__init__.py']

ALL_CLASSES = []

for mod in modules:
    module_name = os.path.splitext(mod)[0]
    try:
        module = importlib.import_module(f'.{module_name}', package=__name__)
        if hasattr(module, 'CLASSES'):
            ALL_CLASSES.extend(module.CLASSES)
    except ImportError as e:
        print(f"Error importing module {module_name}: {e}")

