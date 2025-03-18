/* Copyright (C) 2025 Ian Sloat
* Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. 
*
* Generated automatically with gen_headers.py */

#ifndef MAKESDNA_TYPES_HPP
#define MAKESDNA_TYPES_HPP

#include "makesdna_3_6_0.h"
#include "makesdna_4_0_0.h"
#include "makesdna_4_1_0.h"
#include "makesdna_4_1_1.h"
#include "makesdna_4_2_0.h"
#include "makesdna_4_2_1.h"
#include "makesdna_4_2_4.h"
#include "makesdna_4_2_8.h"
#include "makesdna_4_3_0.h"
#include "makesdna_4_4_0.h"
#include "makesdna_macros.h"

enum class BlenderVersion {
    VER_3_6_0,
    VER_4_0_0,
    VER_4_1_0,
    VER_4_1_1,
    VER_4_2_0,
    VER_4_2_1,
    VER_4_2_4,
    VER_4_2_8,
    VER_4_3_0,
    VER_4_4_0,
};

#define BLENDVER_ENUM_BINDINGS(enum_class, mod_name) \
    py::enum_<enum_class>(mod_name, #enum_class) \
    .value("VER_3_6_0", enum_class::VER_3_6_0) \
    .value("VER_4_0_0", enum_class::VER_4_0_0) \
    .value("VER_4_1_0", enum_class::VER_4_1_0) \
    .value("VER_4_1_1", enum_class::VER_4_1_1) \
    .value("VER_4_2_0", enum_class::VER_4_2_0) \
    .value("VER_4_2_1", enum_class::VER_4_2_1) \
    .value("VER_4_2_4", enum_class::VER_4_2_4) \
    .value("VER_4_2_8", enum_class::VER_4_2_8) \
    .value("VER_4_3_0", enum_class::VER_4_3_0) \
    .value("VER_4_4_0", enum_class::VER_4_4_0) \
    .export_values();

#endif