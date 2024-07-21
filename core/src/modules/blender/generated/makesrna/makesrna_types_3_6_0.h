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

Various structs from Blender v3.6 makesrna headers.

This file and others like it (makesrna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESRNA_TYPES_3_6_0_H
#define MAKESRNA_TYPES_3_6_0_H

struct PrimitiveStringRNA3_6_0;
struct PrimitiveIntRNA3_6_0;
struct PrimitiveFloatRNA3_6_0;

struct PrimitiveStringRNA3_6_0 {
    const char *value;
};

struct PrimitiveIntRNA3_6_0 {
    int value;
};

struct PrimitiveFloatRNA3_6_0 {
    float value;
};

#endif
