"""
Copyright (C) 2025 Ian Sloat
Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>.

Code generator functions
"""

from webbrowser import get
import code_analysis
import copy
import datetime


def license():
    return f"""/* SPDX-FileCopyrightText: {datetime.datetime.now().year} Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */"""


def licence2():
    return f"""/* Copyright (C) {datetime.datetime.now().year} Ian Sloat
* Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. 
*
* Generated automatically with gen_headers.py */"""


def versioned_names(ast: code_analysis.SyntaxTree, min_ver=None):
    versions = ast.get_versions()
    if not min_ver:
        min_ver = versions[0]
    output = {}
    for item in ast.items.values():
        item_ver = max(item.ver, min_ver)
        output[item.ref.name] = item.ref.name + \
            f"{item_ver[0]}_{item_ver[1]}_{item_ver[2]}"
    return output


def forward_declarations(ast: code_analysis.SyntaxTree, min_ver=None):
    name_mappings = versioned_names(ast, min_ver)
    max_ver = ast.get_versions()[-1]
    output = set()
    for item in ast.items.values():
        if not (isinstance(item, code_analysis.Revision) and max_ver == max(item.ver, min_ver)
                and isinstance(item.ref, code_analysis.Struct)):
            continue
        item = item.ref
        name = name_mappings.get(item.name, item.name)
        if name.startswith("_"):
            continue
        output.add(f"struct {name};")
    return '\n'.join(sorted(output))


def fix_struct_names(struct: code_analysis.Struct, name_mappings):
    struct.name = name_mappings.get(struct.name, struct.name)
    for field in struct.fields:
        if isinstance(field, code_analysis.Struct):
            fix_struct_names(field, name_mappings)
        elif field.type.is_tagged() and field.type.keyword == "struct":
            field.type.tag = name_mappings.get(field.type.tag, field.type.tag)


def struct_declarations(ast: code_analysis.SyntaxTree, min_ver=None):
    max_ver = ast.get_versions()[-1]
    name_mappings = versioned_names(ast, min_ver)
    order = ast.get_order()
    output = []
    for name in order:
        item = ast.items[name]
        if not (isinstance(item, code_analysis.Revision) and max_ver == max(item.ver, min_ver)
                and isinstance(item.ref, code_analysis.Struct)):
            continue
        struct = copy.deepcopy(item.ref)
        fix_struct_names(struct, name_mappings)
        output.append(str(struct))
    return "\n\n".join(output)


def includes(ast: code_analysis.SyntaxTree, prefix, min_ver=None):
    versions = ast.get_versions()
    output = "#include <stdint.h>" if ast.uses_stdint() else ""
    for v in versions:
        if v < min_ver or v == versions[-1]:
            continue
        if output:
            output += "\n"
        output += f'#include "{prefix}{v[0]}_{v[1]}_{v[2]}.h"'
    return output


def add_include_guard(input, header_name):
    header_name = header_name.replace(".", "_").upper()
    return f"#ifndef {header_name}\n#define {header_name}\n\n" + input + "\n\n#endif"


def gen_header(ast: code_analysis.SyntaxTree, include_prefix, min_ver=None):
    elements = []
    incl = includes(ast, include_prefix, min_ver)
    if incl:
        elements.append(incl)
    f_decl = forward_declarations(ast, min_ver)
    if f_decl:
        elements.append(f_decl)
    s_decl = struct_declarations(ast, min_ver)
    if s_decl:
        elements.append(s_decl)
    body = "\n\n".join(elements)
    max_ver = ast.get_versions()[-1]
    guard_name = f"{include_prefix}{max_ver[0]}_{max_ver[1]}_{max_ver[2]}_H".upper(
    )
    return license() + "\n\n" + add_include_guard(body, guard_name)


def get_pybind_ver_enum_macro(vers):
    output = (
        "#define BLENDVER_ENUM_BINDINGS(enum_class, mod_name) \\\n"
        "    py::enum_<enum_class>(mod_name, #enum_class) \\\n"
    )
    for ver in sorted(vers):
        ver_str = f"VER_{ver[0]}_{ver[1]}_{ver[2]}"
        output += f'    .value("{ver_str}", enum_class::{ver_str}) \\\n'
    return output + "    .export_values();"


def ver_string(ver):
    return f"{ver[0]}_{ver[1]}_{ver[2]}"


def get_version_macro(obj_name, versions, global_ref="blender_ver", global_ref_type="BlenderVersion", class_ref="data_ptr"):
    output = f"#define {obj_name.upper()}_BASE_RETURN_BODY(A, B, C) \\\n"
    versions = sorted(list(versions))
    for i in range(1, len(versions)):
        current_ver = versions[i - 1]
        next_ver = versions[i]
        ref_name = obj_name + ver_string(current_ver)
        output += (f"    if ({global_ref} < {global_ref_type}::VER_{ver_string(next_ver)}) \\\n"
                   f"        return A ( B reinterpret_cast<{ref_name}*>({class_ref})-> C); \\\n")
    last_ref = obj_name + ver_string(versions[-1])
    output += (f"    return A ( B reinterpret_cast<{last_ref}*>({class_ref})-> C); \n"
               f"#define {obj_name.upper()}_RETURN_REF(T, M)    {obj_name.upper()}_BASE_RETURN_BODY(T, &, M)\n"
               f"#define {obj_name.upper()}_RETURN_AS(T, M)     {obj_name.upper()}_BASE_RETURN_BODY(T,, M)\n"
               f"#define {obj_name.upper()}_RETURN(M)           {obj_name.upper()}_BASE_RETURN_BODY(,, M)")
    return output


def generate_macros(ast: code_analysis.VersionedSyntaxTree, file_name, min_ver, max_ver):
    versions = [v for v in ast.get_versions() if v >= min_ver and v <= max_ver]
    macros = []
    for item in ast.master_tree:
        item_vers = []
        try:
            for v in versions:
                rev = ast.get(item, v)
                if (v == rev.ver or v == min_ver) and not isinstance(rev.ref, code_analysis.ItemRemoval):
                    item_vers.append(v)
        except:
            pass
        if item_vers:
            macros.append(get_version_macro(item, item_vers))
    macros = "\n\n".join(sorted(macros))
    return licence2() + "\n\n" + add_include_guard(macros, file_name)


def get_ver_enums(enum_class, vers):
    output = f"enum class {enum_class} {'{'}\n"
    for ver in sorted(vers):
        output += f"    VER_{ver[0]}_{ver[1]}_{ver[2]},\n"
    return output + "};"


def generate_types_hpp(ast: code_analysis.VersionedSyntaxTree, file_name, min_ver, max_ver, macro_header, header_prefix):
    versions = [v for v in ast.get_versions() if v >= min_ver and v <= max_ver]
    includes = [
        f'#include "{header_prefix}{ver_string(v)}.h"' for v in versions]
    includes.append(f'#include "{macro_header}"')
    includes = "\n".join(includes)
    pybind_macro = get_pybind_ver_enum_macro(versions)
    enums = get_ver_enums("BlenderVersion", versions)
    return licence2() + "\n\n" + add_include_guard('\n\n'.join([includes, enums, pybind_macro]), file_name)
