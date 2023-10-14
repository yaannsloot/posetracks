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

def set_lock_on_tracks(tracks, lock):
    for track in tracks:
        track.lock = lock


def set_hidden_on_tracks(tracks, hide):
    for track in tracks:
        track.hide = hide


def set_select_tracks(tracks, select):
    for track in tracks:
        track.select = select


def check_list(item, collection):
    for i in collection:
        if item == i:
            return True
    return False

