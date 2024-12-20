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

Tests for new gen header functions
"""
import peewee

from gen_headers import git
from gen_headers import regex
from gen_headers import types
from gen_headers import depgraph
from gen_headers import db
import os
import pprint
import re

def test():
    git.init_repo()
    versions = git.available_versions()
    for v in versions:
        git.checkout_version(v)
        dna_dir = 'blender/source/blender/makesdna'
        src_structs = {}
        structs = {}
        for header in os.listdir(dna_dir):
            if not header.endswith(('.h', '.hpp')):
                continue
            header = os.path.join(dna_dir, header)
            with open(header, 'r') as f:
                header = f.read()
            regex.file_structs_to_dict(header, src_structs)
        for n, s in src_structs.items():
            str_obj = types.StructDef(n, s[1:-1])
            if not str_obj.valid:
                continue
            structs[n] = str_obj
        graph = depgraph.graph()
        for s_name, s_obj in structs.items():
            graph.add_node(s_name)
            for line in s_obj.body:
                if line.is_struct and line.ptr_level == 0:
                    graph.add_edge(line.struct_name, s_name)
        graph.remove_dangling()
        pprint.pprint(graph.nodes)
        pprint.pprint(graph.edges)
        pprint.pprint(graph.sort())


def test2():
    graph = depgraph.graph()
    graph.add_node('a')
    graph.add_node('b')
    graph.add_node('c')
    graph.add_node('d')
    graph.add_node('z')
    graph.add_edge('a', 'd')
    graph.add_edge('a', 'b')
    graph.add_edge('e', 'd')
    graph.add_edge('a', 'g')
    graph.add_edge('b', 'c')
    graph.add_edge('a', 'c')
    graph.add_edge('b', 'c')
    #graph.add_edge('c', 'a')
    print(graph.find_dangling())
    print(graph.nodes, graph.edges)
    graph.remove_dangling()
    print(graph.nodes, graph.edges)
    print(graph.sort())


def test3():
    ver, _ = db.BlenderVersion.get_or_create(major=2, minor=3, patch=5)
    struct, _ = db.Struct.get_or_create(tag='TheStruct')
    revision, _ = db.Revision.get_or_create(ver_id=ver, struct_id=struct, src="whoa", crc32=db.crc32("whoa"))
    print([rev for rev in struct.revisions])

def write_header(name_prefix, ver, dep_graph, deps):
    final_output = ""
    if len(deps) == 0:
        return final_output, ""
    ver_string = lambda a: f"{a[0]}_{a[1]}_{a[2]}"
    ver_name = lambda b: f"{name_prefix}{ver_string(b)}"
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
    for v in sorted(list(import_vers)):
        final_output += f'#include "{ver_name(v)}.h"\n'
    for tag in dep_graph.sort():
        if tag in src_mappings:
            final_output += '\n' + src_mappings[tag] + '\n'
    for orig, nname in name_mappings.items():
        pattern = rf" {orig}(\s|\*)"
        replacement = rf" {nname}\1"
        final_output = re.sub(pattern, replacement, final_output)
    guard = f"{ver_name(ver)}_H".upper()
    final_output = f"#ifndef {guard}\n#define {guard}\n" + final_output + "\n#endif"
    return final_output, f"{ver_name(ver)}.h"

def get_version_macro(obj_name, versions, global_ref="blender_ver", global_ref_type="BlenderVersion", class_ref="data_ptr"):
    ver_string = lambda a: f"{a[0]}_{a[1]}_{a[2]}"
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


def test4():
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
        graph, all_tags = depgraph.graph_from_dep_query(deps)
        graph.remove_dangling()
        if not db_ver.evaluated:
            print("Evaluating dependencies...", v)
            unavailable = all_tags - graph.nodes
            targets = db.Revision.select().join(db.Struct).where((db.Revision.ver_id == db_ver) & (db.Struct.tag << unavailable))
            db.single_val_update_atomic(db.Revision, {"rev_id": targets}, {"available": 0})
            db_ver.evaluated = True
            db_ver.save()
            deps = db.resolve_dependencies(v)
        if len(deps) == 0:
            continue
        print("Writing header file...", v)
        header_out, fname = write_header("makesdna_types", v, graph, deps)
        os.makedirs("output", exist_ok=True)
        with open(os.path.join("output", fname), 'w') as f:
            f.write(header_out)
    macros_out = write_macros(db.get_version_mappings(), "makesdna_mappings.h")
    with open(os.path.join("output", "makesdna_mappings.h"), 'w') as f:
            f.write(macros_out)
        

        
        
        
        
        


test4()
#db_ver = db.get_ver_ref((2,25,0))
#unavailable = {"bPoseChannel", "bPose", "bActionChannel"}
#targets = db.Revision.select().join(db.Struct).where((db.Revision.ver_id == db_ver) & (db.Struct.tag << unavailable))
#db.single_val_update_atomic(db.Revision, {"rev_id": targets}, {"available": 0})

#print(gen_version_map("Object", [(1,0,0),(2,0,0),(2,2,0),(3,0,0)]))