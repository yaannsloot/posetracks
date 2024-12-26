"""
Copyright (C) 2024 Ian Sloat

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

----------------------------------------------------------------------

Generates headers that can be used to access internal memory structures
from blender.
"""

from gen_headers import git
from gen_headers import regex
from gen_headers import types
from gen_headers import depgraph
from gen_headers import db
import os
import re
from peewee import *

def ver_string(ver):
    return f"{ver[0]}_{ver[1]}_{ver[2]}"

def ver_name(name, ver):
    return f"{name}{ver_string(ver)}"

def get_declarations(deps):
    tags = sorted(list({ver_name(dep['tag'], (dep['major'], dep['minor'], dep['patch'])) for dep in deps}))
    return '\n'.join([f"struct {tag};" for tag in tags])
    
def prune_deps(deps, ver):
    for d in deps:
        if ver != (d['major'], d['minor'], d['patch']):
            continue
        yield d

def get_includes(vers, name_prefix):
    return "\n".join([f'#include "{ver_name(name_prefix, ver)}.h"' for ver in sorted(vers)])

def get_ver_enums(enum_class, vers):
    output = f"enum class {enum_class} {'{'}\n"
    for ver in sorted(vers):
        output += f"    VER_{ver[0]}_{ver[1]}_{ver[2]},\n"
    return output + "};"

def get_pybind_ver_enum_macro(vers):
    output = (
        "#define BLENDVER_ENUM_BINDINGS(enum_class, mod_name) \\\n"
        "    py::enum_<enum_class>(mod_name, #enum_class) \\\n"
    )
    for ver in sorted(vers):
        ver_str = f"VER_{ver[0]}_{ver[1]}_{ver[2]}"
        output += f'    .value("{ver_str}", enum_class::{ver_str}) \\\n'
    return output + "    .export_values();"

def add_include_guard(input, header_name):
    header_name = header_name.replace(".", "_").upper()
    return f"#ifndef {header_name}\n#define {header_name}\n\n" + input + "\n\n#endif"

def write_header(name_prefix, ver, dep_graph, deps):
    deps = list(prune_deps(deps, ver))
    dep_graph = dep_graph.get_version_subgraph(ver)
    final_output = ""
    if len(deps) == 0:
        return final_output, ""
    import_vers = set()
    name_mappings = {}
    src_mappings = {}
    for d in deps:
        main_v = (d['major'], d['minor'], d['patch'])
        dep_v = (d["dep_major"], d["dep_minor"], d["dep_patch"])
        name_mappings[d['tag']] = f"{d['tag']}{ver_string(main_v)}"
        src_mappings[d['tag']] = d['src']
        if main_v == dep_v:
            continue
        if d['dep_available']:
            import_vers.add(dep_v)
            name_mappings[d['dep_tag']] = f"{d['dep_tag']}{ver_string(dep_v)}"
        else:
            name_mappings[d['dep_tag']] = "void"
    final_output += "\n#include <stdint.h>\n"
    final_output += get_includes(list(import_vers), name_prefix)
    final_output += f"\n{get_declarations(deps)}\n"
    for tag in dep_graph.sort():
        if tag in src_mappings:
            final_output += '\n' + src_mappings[tag] + '\n'
    for orig, nname in name_mappings.items():
        pattern = rf" {orig}(\s|\*)"
        replacement = rf" {nname}\1"
        final_output = re.sub(pattern, replacement, final_output)
    file_name = f"{ver_name(name_prefix, ver)}.h"
    return add_include_guard(final_output, file_name), file_name

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
    f"#define {obj_name.upper()}_RETURN_REF(T, M)    {obj_name.upper()}_BASE_RETURN_BODY(T, &, M) \n"
    f"#define {obj_name.upper()}_RETURN_AS(T, M)     {obj_name.upper()}_BASE_RETURN_BODY(T,, M) \n"
    f"#define {obj_name.upper()}_RETURN(M)           {obj_name.upper()}_BASE_RETURN_BODY(,, M) \n")
    return output


def write_macros(structs, file_name):
    output = ""
    for s, vers in structs.items():
        macro = get_version_macro(s, vers)
        output += macro + '\n'
    guard = file_name.replace('.', '_').upper()
    return f"#ifndef {guard}\n#define {guard}\n\n" + output + "#endif"

def write_types_hpp(file_name, vers, name_prefix, macro_header):
    includes = get_includes(vers, name_prefix) + f'\n#include "{macro_header}"'
    enums = get_ver_enums("BlenderVersion", vers)
    pybind_macro = get_pybind_ver_enum_macro(vers)
    return add_include_guard('\n\n'.join([includes, enums, pybind_macro]), file_name)

def main():
    git.init_repo()
    versions = git.available_versions()
    dna_dir = 'blender/source/blender/makesdna'
    for v in versions:
        db_ver = db.get_ver_ref(v)
        if not db_ver.loaded:
            print('Checking out blender version...', v)
            git.checkout_version(v)
            print("Loading structs...")
            s_map = regex.load_structs_from_dir(dna_dir)
            s_defs = types.from_mapping(s_map)
            print("Writing structs to db...")
            db.load_revisions_atomic(v, s_defs)
            db_ver.loaded = True
            db_ver.save()
        deps = db.resolve_dependencies(v)
        graph, _ = depgraph.graph_from_dep_query(deps, v)
        if not db_ver.evaluated:
            print("Evaluating dependencies...", v)
            transitive_updates = [(str(node), node.ver[0], node.ver[1], node.ver[2]) for node in graph.get_nodes_in_ver(v) if node.ver != v]
            targets = (db.Revision.select()
                       .join(db.Struct, on=(db.Revision.struct_id == db.Struct.struct_id))
                       .join(db.RevisionVersion, on=(db.Revision.rev_id == db.RevisionVersion.rev_id))
                       .join(db.BlenderVersion, on=(db.RevisionVersion.ver_id == db.BlenderVersion.ver_id))
                       .where(Tuple(db.Struct.tag, db.BlenderVersion.major, db.BlenderVersion.minor, db.BlenderVersion.patch).in_(transitive_updates)))
            db.tag_revisions_with_ver_atomic(targets, v)
            graph, all_tags = depgraph.graph_from_dep_query(deps, v)
            graph.remove_dangling()
            unavailable = all_tags - {tag for tag in graph.nodes.keys()}
            targets = (db.RevisionVersion.select()
                       .join(db.Revision, on=(db.RevisionVersion.rev_id == db.Revision.rev_id))
                       .join(db.Struct, on=(db.Revision.struct_id == db.Struct.struct_id))
                       .where((db.RevisionVersion.ver_id == db_ver) & (db.Struct.tag << unavailable)))
            db.single_val_update_atomic(db.RevisionVersion, {"id": targets}, {"available": 0})
            db_ver.evaluated = True
            db_ver.save()
            deps = db.resolve_dependencies(v)
        print("Writing header file...", v)
        header_out, fname = write_header("makesdna_types_", v, graph, deps)
        if not header_out:
            continue
        os.makedirs("output", exist_ok=True)
        with open(os.path.join("output", fname), 'w') as f:
            f.write(header_out)
    macros_out = write_macros(db.get_version_mappings(), "makesdna_mappings.h")
    with open(os.path.join("output", "makesdna_mappings.h"), 'w') as f:
        f.write(macros_out)
    with open(os.path.join("output", "blender_makesdna.hpp"), 'w') as f:
        f.write(write_types_hpp("blender_makesdna.hpp", db.get_active_versions(), "makesdna_types_", "makesdna_mappings.h"))

if __name__ == "__main__":
    main()