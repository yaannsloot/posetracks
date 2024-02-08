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

modules = [file for file in os.listdir(current_dir) if os.path.isdir(os.path.join(current_dir, file))]

ALL_CLASSES = []

for mod in modules:
    try:
        module = importlib.import_module(f'.{mod}', package=__name__)
        if hasattr(module, 'ALL_CLASSES'):
            ALL_CLASSES.extend(module.ALL_CLASSES)
    except ImportError as e:
        print(f"Error importing module {mod}: {e}")

