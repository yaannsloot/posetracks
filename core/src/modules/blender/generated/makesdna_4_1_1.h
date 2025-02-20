/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_4_1_1_H
#define MAKESDNA_4_1_1_H

#include "makesdna_3_6_0.h"
#include "makesdna_4_0_0.h"
#include "makesdna_4_1_0.h"

struct uiList4_1_1;

struct uiList4_1_1 {
    struct uiList4_1_1 *next;
    struct uiList4_1_1 *prev;
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
    struct IDProperty3_6_0 *properties;
    void *dyn_data;
};

#endif