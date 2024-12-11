"""
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

----------------------------------------------------------------------

Blender related git functions
"""

import git
import os

_repo = None
_versions = []


def init_repo():
    global _repo
    if os.path.exists('blender/.git'):
        _repo = git.Repo('blender')
    else:
        _repo = git.Repo.clone_from('https://github.com/blender/blender.git', 'blender')
    _get_versions()


def _get_versions():
    if _repo is None:
        return
    version_tags = [str(tag) for tag in _repo.tags]
    for ver_str in version_tags:
        try:
            ver = [int(v) for v in ver_str.strip('v').split('.')]
            ver_split = [0, 0, 0]
            for i in range(len(ver)):
                ver_split[i] = ver[i]
        except:
            continue
        _versions.append((ver_str, tuple(ver_split)))
    _versions.sort(key=lambda a: a[1])


def available_versions():
    return [v[1] for v in _versions]


def checkout_version(ver):
    ver_str = ""
    for v in _versions:
        if v[1] == ver:
            ver_str = v[0]
    if not ver_str:
        print(f"{ver} not a valid blender version")
    print(f'Checking out blender version {ver}...')
    _repo.git.checkout(ver_str, '-f')
