/*
Copyright (C) 2024 Blender Foundation. All rights reserved.

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

---------------------------------------------------------------------

Various structs from Blender v3.6.8 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_6_8_H
#define MAKESDNA_TYPES_3_6_8_H

#include "makesdna_types_2_93_0.h"

struct NodeTexNoise3_6_8;

struct NodeTexNoise3_6_8 {
    NodeTexBase2_93_0 base;
    int dimensions;
    uint8_t type;
    uint8_t normalize;
    char _pad[2];
};

#endif
