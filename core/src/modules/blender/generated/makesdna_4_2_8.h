/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_4_2_8_H
#define MAKESDNA_4_2_8_H

#include "makesdna_3_6_0.h"
#include "makesdna_4_0_0.h"
#include "makesdna_4_1_0.h"
#include "makesdna_4_1_1.h"
#include "makesdna_4_2_0.h"
#include "makesdna_4_2_1.h"
#include "makesdna_4_2_4.h"

struct FileDirEntry4_2_8;
struct SDNA4_2_8;

struct SDNA4_2_8 {
    const  char *data;
    int data_len;
    int data_alloc;
    int names_len;
    int names_len_alloc;
    const  char **names;
    short *names_array_len;
    int pointer_size;
    const  char **types;
    int types_len;
    short *types_size;
    void *structs;
    int structs_len;
    void *structs_map;
    void *mem_arena;
    struct {
        const  char **names;
        const  char **types;
        void *structs_map;
    } alias;
    int *types_alignment;
};

struct FileDirEntry4_2_8 {
    struct FileDirEntry4_2_8 *next;
    struct FileDirEntry4_2_8 *prev;
    unsigned int uid;
    const  char *name;
    unsigned long long size;
    long long time;
    struct {
        char size_str[16];
        char datetime_str[24];
    } draw_data;
    int typeflag;
    int blentype;
    char *relpath;
    char *redirection_path;
    struct ID4_2_0 *id;
    void *asset;
    int preview_icon_id;
    short flags;
    int attributes;
};

#endif