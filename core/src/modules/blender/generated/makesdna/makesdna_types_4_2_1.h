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

Various structs from Blender v4.2.1 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_2_1_H
#define MAKESDNA_TYPES_4_2_1_H

#include <stdint.h>
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_4_1_0.h"

struct wmWindowManager4_2_1;
struct wmOwnerID4_2_1;

struct wmOwnerID4_2_1 {
    wmOwnerID4_2_1 *next, *prev;
    char name[128];
};

struct wmWindowManager4_2_1 {
    ID4_1_0 id;
    wmWindow4_1_0 *windrawable;
    wmWindow4_1_0 *winactive;
    ListBase2_93_0 windows;
    uint8_t init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    ListBase2_93_0 operators;
    ListBase2_93_0 notifier_queue;
    void *notifier_queue_set;
    void *_pad1;
    int extensions_updates;
    int extensions_blocked;
    ListBase2_93_0 jobs;
    ListBase2_93_0 paintcursors;
    ListBase2_93_0 drags;
    ListBase2_93_0 keyconfigs;
    wmKeyConfig2_93_0 *defaultconf;
    wmKeyConfig2_93_0 *addonconf;
    wmKeyConfig2_93_0 *userconf;
    ListBase2_93_0 timers;
    void *autosavetimer;
    char autosave_scheduled;
    char _pad2[7];
    void *undo_stack;
    void *message_bus;
    wmXrData2_93_0 xr;
    void *runtime;
};

#endif
