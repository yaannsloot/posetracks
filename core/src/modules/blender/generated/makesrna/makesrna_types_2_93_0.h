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

Various structs from Blender v2.93 makesrna headers.

This file and others like it (makesrna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESRNA_TYPES_2_93_0_H
#define MAKESRNA_TYPES_2_93_0_H

#include <stdint.h>

struct PropertyElemRNA2_93_0;
struct PointerRNA2_93_0;
struct PropertyPointerRNA2_93_0;
struct PathResolvedRNA2_93_0;
struct CountIterator2_93_0;
struct CollectionPointerLink2_93_0;
struct CollectionListBase2_93_0;
struct EnumPropertyItem2_93_0;
struct ParameterList2_93_0;
struct ParameterIterator2_93_0;

struct PointerRNA2_93_0 {
    void *owner_id;
    void *type;
    void *data;
};

struct CountIterator2_93_0 {
    void *ptr;
    int item;
};

struct CollectionListBase2_93_0 {
    CollectionPointerLink2_93_0 *first, *last;
};

struct EnumPropertyItem2_93_0 {
    int value;
    const char *identifier;
    int icon;
    const char *name;
    const char *description;
};

struct ParameterList2_93_0 {
    void *data;
    void *func;
    int alloc_size;
    int arg_count, ret_count;
};

struct ParameterIterator2_93_0 {
    ParameterList2_93_0 *parms;
    void *data;
    int size, offset;
    void *parm;
    int valid;
};

struct PropertyElemRNA2_93_0 {
    PropertyElemRNA2_93_0 *next, *prev;
    PointerRNA2_93_0 ptr;
    void *prop;
    int index;
};

struct PropertyPointerRNA2_93_0 {
    PointerRNA2_93_0 ptr;
    void *prop;
};

struct PathResolvedRNA2_93_0 {
    PointerRNA2_93_0 ptr;
    void *prop;
    int prop_index;
};

struct CollectionPointerLink2_93_0 {
    CollectionPointerLink2_93_0 *next, *prev;
    PointerRNA2_93_0 ptr;
};

#endif
