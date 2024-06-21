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
import shutil
import core.build

dir_list = ['operators', 'property_groups', 'ui']
file_list = ['__init__.py', 'events.py', 'global_vars.py', 'pose_autogen_defs.py', 'utils.py', 'LICENSE']
def main():
    #core.build.main()
    print('Packaging...')
    base_dir = os.path.dirname(os.path.abspath(__file__))
    build_dir = 'build'
    package_root = os.path.join(build_dir, 'motion_engine')
    package_core_root = os.path.join(package_root, 'MotionEngine')
    redis_path = os.path.join('core', 'build', 'redis')
    build_dir = os.path.abspath(build_dir)
    package_root = os.path.abspath(package_root)
    package_core_root = os.path.abspath(package_core_root)
    redis_path = os.path.abspath(redis_path)
    core.build.prepare_directory(build_dir)
    core.build.prepare_directory(package_root)
    shutil.copytree(redis_path, package_core_root)
    for d in dir_list:
        shutil.copytree(os.path.join(base_dir, d), os.path.join(package_root, d))
    for f in file_list:
        shutil.copy(os.path.join(base_dir, f), os.path.join(package_root, f))
    for f in os.listdir(package_core_root):
        if f.endswith('.pyi'):
            os.remove(os.path.join(package_core_root, f))


if __name__ == '__main__':
    main()
