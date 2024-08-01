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

Various structs from Blender v4.1.1 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_1_1_H
#define MAKESDNA_TYPES_4_1_1_H

#include <stdint.h>
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_5_0.h"

struct uiList4_1_1;

struct uiList4_1_1 {
    uiList4_1_1 *next, *prev;
    void *type;
    char list_id[128];
    int layout_type;
    int flag;
    int list_scroll;
    int list_grip;
    int list_last_len;
    int list_last_activei;
    char filter_byname[128];
    int filter_flag;
    int filter_sort_flag;
    IDProperty3_5_0 *properties;
    uiListDyn2_93_0 *dyn_data;
};

#endif
