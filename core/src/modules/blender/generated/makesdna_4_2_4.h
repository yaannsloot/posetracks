/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_4_2_4_H
#define MAKESDNA_4_2_4_H

#include "makesdna_3_6_0.h"
#include "makesdna_4_0_0.h"
#include "makesdna_4_1_0.h"
#include "makesdna_4_1_1.h"
#include "makesdna_4_2_0.h"
#include "makesdna_4_2_1.h"

struct uiPreview4_2_4;
struct wmWindowManager4_2_4;

struct uiPreview4_2_4 {
    struct uiPreview4_2_4 *next;
    struct uiPreview4_2_4 *prev;
    char preview_id[64];
    short height;
    short tag;
    unsigned int id_session_uid;
};

struct wmWindowManager4_2_4 {
    struct ID4_2_0 id;
    struct wmWindow4_2_0 *windrawable;
    struct wmWindow4_2_0 *winactive;
    struct ListBase3_6_0 windows;
    unsigned char init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    struct ListBase3_6_0 operators;
    struct ListBase3_6_0 notifier_queue;
    void *notifier_queue_set;
    const  void *notifier_current;
    int extensions_updates;
    int extensions_blocked;
    struct ListBase3_6_0 jobs;
    struct ListBase3_6_0 paintcursors;
    struct ListBase3_6_0 drags;
    struct ListBase3_6_0 keyconfigs;
    struct wmKeyConfig3_6_0 *defaultconf;
    struct wmKeyConfig3_6_0 *addonconf;
    struct wmKeyConfig3_6_0 *userconf;
    struct ListBase3_6_0 timers;
    void *autosavetimer;
    char autosave_scheduled;
    char _pad2[7];
    void *undo_stack;
    void *message_bus;
    struct wmXrData4_2_0 xr;
    void *runtime;
};

#endif