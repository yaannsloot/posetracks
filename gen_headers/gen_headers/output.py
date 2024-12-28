import re


def ver_string(ver):
    return f"{ver[0]}_{ver[1]}_{ver[2]}"


def ver_name(name, ver):
    return f"{name}{ver_string(ver)}"


def get_declarations(deps):
    tags = sorted(list(
        {ver_name(dep['tag'], (dep['major'], dep['minor'], dep['patch'])) for dep in deps}), key=lambda a: a.upper())
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


def generate_header(name_prefix, ver, dep_graph, deps):
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


def generate_macros(structs, file_name):
    output = ""
    for s, vers in structs.items():
        macro = get_version_macro(s, vers)
        output += macro + '\n'
    guard = file_name.replace('.', '_').upper()
    return f"#ifndef {guard}\n#define {guard}\n\n" + output + "#endif"


def generate_types_hpp(file_name, vers, name_prefix, macro_header):
    includes = get_includes(vers, name_prefix) + f'\n#include "{macro_header}"'
    enums = get_ver_enums("BlenderVersion", vers)
    pybind_macro = get_pybind_ver_enum_macro(vers)
    return add_include_guard('\n\n'.join([includes, enums, pybind_macro]), file_name)
