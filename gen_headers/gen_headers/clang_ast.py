"""
Copyright (C) 2025 Ian Sloat
Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>.

TODO: Add support for function declarations, enums, and unions (might work but not tested)
TODO: Add checks for tag refs

Keywords and aliases from C standards, stddef.h, and sys/types.h
Using x64 type definitions for stddef.h

stdint.h should be included in output headers if types are present
"""

import json
import re
import os
import textwrap
import clang.cindex

# Must have clang pip package installed for this to work
clang.cindex.Config.set_library_path(os.path.join(
    os.path.dirname(os.path.realpath(clang.cindex.__file__)), 'native'))

keywords = {
    "char": {},
    "signed char": {},
    "unsigned char": {"u_char"},
    "short": {},
    "unsigned short": {"ushort", "u_short"},
    "int": {},
    "unsigned int": {"u_int", "uint"},
    "long": {"off_t", "ssize_t", "ptrdiff_t"},
    "unsigned long": {"u_long", "size_t", "nullptr_t"},
    "float": {},
    "double": {},
    "void": {},
    "long long": {},
    "unsigned long long": {},
    "long double": {},
    "bool": {"_Bool"},  # special kw as of C23
}

tagged = {
    "struct",
    "union",
    "enum",
}


stdint = {
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "int_fast8_t",
    "int_fast16_t",
    "int_fast32_t",
    "int_fast64_t",
    "int_least8_t",
    "int_least16_t",
    "int_least32_t",
    "int_least64_t",
    "intmax_t",
    "intptr_t",
    "uint8_t",
    "uint16_t",
    "uint32_t",
    "uint64_t",
    "uint_fast8_t",
    "uint_fast16_t",
    "uint_fast32_t",
    "uint_fast64_t",
    "uint_least8_t",
    "uint_least16_t",
    "uint_least32_t",
    "uint_least64_t",
    "uintmax_t",
    "uintptr_t",
}

keywords.update({kw: {} for kw in stdint})

aliases = {}
for kw, alts in keywords.items():
    for alias in alts:
        aliases[alias] = kw

kw_pattern = sorted(list(keywords.keys()) +
                    list(aliases.keys()) + list(tagged), key=len, reverse=True)
kw_pattern = r'^(' + "|".join(map(re.escape,
                                  map(lambda s: s + ' ', kw_pattern))) + ")"


def normalize_text(text):
    text = text.replace(';', '')
    text = re.sub(r'\s+', ' ', text).strip()  # remove extra spaces
    # remove righthand spaces from ptr chars
    text = re.sub(r'\*\s+', '*', text)
    # collapse spaces for left bracket
    text = re.sub(r'\s+\[\s+|\[\s+|\s+\[', '[', text)
    # collapse spaces for right bracket
    text = re.sub(r'\s+\]\s+|\]\s+|\s+\]', ']', text)
    # collapse righthand spaces for left parenthesis
    text = re.sub(r'\(\s+', '(', text)
    # collapse spaces for right parenthesis
    text = re.sub(r'\s+\)\s+|\)\s+|\s+\)', ')', text)
    return text


def find_sep(text, start=0):
    output = []
    level = 0
    for i in range(start, len(text)):
        if level < 0:
            break
        elif text[i] in "([":
            level += 1
        elif text[i] in ")]":
            level -= 1
        elif text[i] == ',' and level == 0:
            output.append(i)
    return output


def match_kw(text):
    text = normalize_text(text)
    match = re.match(kw_pattern, text)
    if match:
        type = match.group(1).strip()
        return re.sub(kw_pattern, '', text, 1), aliases.get(type, type)
    return text, None


def wrap_parenthesis(text):
    text = normalize_text(text)
    pattern = r'^\((.+)\)$'
    match = re.match(pattern, text)
    if match:
        text = match.group(1).strip()
        return {"type": "parenthesis", "inner": wrap_ptr(re.sub(pattern, '', text, 1))}
    return {"type": "name", "val": text}


def wrap_array(text):
    text = normalize_text(text)
    pattern = r'\[([^\[\]]*)\]$'
    match = re.search(pattern, text)
    if match:
        val = match.group(1).strip()
        return {"type": "array", "val": val, "inner": wrap_ptr(re.sub(pattern, '', text, 1))}
    return wrap_parenthesis(text)


def wrap_ptr(text):
    text = normalize_text(text)
    pattern = r'^(\*)'
    match = re.match(pattern, text)
    if match:
        return {"type": "ptr", "inner": wrap_ptr(re.sub(pattern, '', text, 1))}
    return wrap_array(text)


def extract_inline_type(text, cursor):
    type = None
    tag = None
    first_child = None
    if cursor:
        first_child = next(cursor.get_children(), None)
    if first_child:
        def process_as(t):
            return text[text.rfind("}") + 1:], t, first_child.spelling
        if first_child.kind == clang.cindex.CursorKind.STRUCT_DECL:
            text, type, tag = process_as("struct")
        elif first_child.kind == clang.cindex.CursorKind.UNION_DECL:
            text, type, tag = process_as("union")
        elif first_child.kind == clang.cindex.CursorKind.ENUM_DECL:
            text, type, tag = process_as("enum")
    return text, type, tag


def wrap_type(text, typedefs=None, cursor=None):
    if not isinstance(text, str):
        return text
    text, type, tag = extract_inline_type(normalize_text(text), cursor)
    if not type:
        text, type = match_kw(text)
    pattern = r'^(\w+\s+)'
    if not type:
        if typedefs is None:
            raise ValueError(
                f"A custom type was found with no provided type dictionary.")
        match = re.match(pattern, text)
        if not match:
            raise ValueError(f"Type not found.")
        type = match.group(1).strip()
        typedef = typedefs.get(type, None)
        if not typedef:
            raise ValueError(f'Typedef "{type}" not found.')
        return wrap_type(re.sub(pattern, typedef + ' ', text, 1))
    output = {"type": "basic_type", "val": type}
    if not tag and type in tagged:
        match = re.match(pattern, text)
        if not match:
            raise ValueError(f"Tag not found for {type}.")
        tag = type = match.group(1).strip()
        text = re.sub(pattern, '', text, 1)
    if tag:
        output["tag"] = tag
    separators = find_sep(text)
    if separators:
        text = text[separators[-1] + 1:]
    output["inner"] = wrap_ptr(text)
    return output


def wrap_decl(text, typedefs=None, cursor=None):
    if not isinstance(text, str):
        return text
    text = normalize_text(text)
    pattern = r'^(const\ )'
    match = re.match(pattern, text)
    try:
        if match:
            return {"type": "const", "inner": wrap_type(re.sub(pattern, '', text, 1), typedefs, cursor)}
        return wrap_type(text, typedefs, cursor)
    except Exception as e:
        raise e.__class__(f'In decl "{text}": {str(e)}')


def get_decl_name(decl):
    if decl["type"] == "name":
        return decl["val"]
    return get_decl_name(decl["inner"])


def get_decl_type(decl):
    if decl["type"] == "basic_type":
        return decl["val"], decl.get("tag", None)
    return get_decl_type(decl["inner"])


def is_using_stdint(decl):
    type, _ = get_decl_type(decl)
    return type in stdint


def is_typedef(decl):
    type, _ = get_decl_type(decl)
    return type not in keywords and type not in tagged and type not in stdint


def join_fields(decl):
    body = ""
    last_struct = None
    for field in decl["fields"]:
        if body:
            body += '\n'
        field_str = decl_to_str(field, False)
        body += field_str
        if field["type"] == "struct_decl":
            last_struct = field["name"]
        elif field["type"] == "basic_type" and field["val"] == "struct" and field["tag"] == last_struct:
            # collapse decl
            if body.endswith(f"{'}'};\n{field_str}"):
                body = body.removesuffix(f"{'}'};\n{field_str}")
                body += "}" + \
                    field_str.removeprefix(f'struct {last_struct}')
            else:
                body = body.replace(
                    f";\n{field_str}", f",{field_str.removeprefix(f'struct {last_struct}')}")
        else:
            last_struct = None
    return body


def decl_to_str(decl, typedef_structs=True):
    output = ""

    def traverse(decl):
        nonlocal output
        type = decl["type"]
        if type == "name":
            output += decl["val"]
        elif type == "ptr":
            traverse(decl["inner"])
            output = "*" + output
        elif type == "array":
            traverse(decl["inner"])
            output += f"[{decl['val'] if decl['val'] else ''}]"
        elif type == "basic_type":
            traverse(decl["inner"])
            type = f"{decl['val']} "
            if "tag" in decl:
                type += f"{decl['tag']} "
            output = type + output
        elif type == "const":
            traverse(decl["inner"])
            output = "const " + output
        elif type == "parenthesis":
            traverse(decl["inner"])
            output = f"({output})"
        elif type == "struct_decl":
            body = textwrap.indent(join_fields(decl), '    ')
            if typedef_structs:
                output = f"typedef struct{' ' + decl['name'] if '(unnamed at ' not in decl['name'] else ''} {'{'}\n{body}\n{'}'} {decl['name']}"
            else:
                output = f"struct{' ' + decl['name'] if '(unnamed at ' not in decl['name'] else ''} {'{'}\n{body}\n{'}'}"

    traverse(decl)
    return output + ";"


def debug_decl(decl):
    # Type is evaluated from the inside out
    output = ""

    def traverse(decl):
        nonlocal output
        type = decl["type"]
        if type == "name":
            output += decl["val"] + " is a(n) "
        elif type == "ptr":
            traverse(decl["inner"])
            if output.endswith("] of "):
                output += "pointers to "
            else:
                output += "pointer to "
        elif type == "array":
            traverse(decl["inner"])
            word = "array"
            if output.endswith("] of "):
                word = "arrays"
            output += f"{word}[{decl['val'] if decl['val'] else 'sizeless'}] "
        elif type == "basic_type":
            traverse(decl["inner"])
            output += f"type {decl['val']}"
            if "tag" in decl:
                output += f" {decl['tag']}"
        elif type == "const":
            traverse(decl["inner"])
            output += " (const)"
        elif type == "parenthesis":
            traverse(decl["inner"])
        if output.endswith("] "):
            output += "of "
    traverse(decl)
    return output


def wrap_struct(cursor, typedefs=None):
    if cursor.kind != clang.cindex.CursorKind.STRUCT_DECL:
        raise ValueError("Cursor is not a struct declaration")
    fields = []
    for field in cursor.get_children():
        if field.kind == clang.cindex.CursorKind.FIELD_DECL:
            line = ' '.join([token.spelling for token in field.get_tokens()])
            fields.append(
                wrap_decl(line, typedefs, field))
        elif field.kind == clang.cindex.CursorKind.STRUCT_DECL:
            inner_td_scope = {}
            inner_td_scope.update(typedefs)
            fields.append(wrap_struct(field, inner_td_scope))
    if typedefs:
        typedefs[cursor.spelling] = "struct " + cursor.spelling
    output = {"type": "struct_decl", "name": cursor.spelling, "fields": fields}
    return output


def debug_cursors(cursor, level=0):
    output = f"{level} {str(cursor.kind)} {cursor.spelling}"
    return '\n'.join([output] + [debug_cursors(child, level + 1) for child in cursor.get_children()])


def process_file(file_path, debug=False):
    index = clang.cindex.Index.create()
    tu = index.parse(
        file_path,
        args=["-E", "-CC", "-detailed-preprocessing-record"],
    )
    if debug:
        with open(os.path.basename(file_path) + ".txt", "w") as f:
            f.write(debug_cursors(tu.cursor))

    output = {}
    typedefs = {}
    for cursor in tu.cursor.get_children():
        try:
            if cursor.kind == clang.cindex.CursorKind.TYPEDEF_DECL:
                typedefs[cursor.spelling] = cursor.underlying_typedef_type.spelling
            elif cursor.kind == clang.cindex.CursorKind.STRUCT_DECL:
                struct = wrap_struct(cursor, typedefs)
                if struct["fields"]:
                    output[struct["name"]] = struct
        except ValueError as e:
            print(f"Encountered error while processing {file_path}:", e)
    return output


if __name__ == "__main__":
    line = "const uint8_t *(*(**foo [][8]))[]; "
    decl = wrap_decl(line)
    print(debug_decl(decl))
    print(decl_to_str(decl))
    print(get_decl_type(decl))
    print(get_decl_name(decl))
    print(is_using_stdint(decl))
    print(is_typedef(decl))

    line = "int (**app)[4][4][4]; "
    decl = wrap_decl(line)
    print(debug_decl(decl))
    print(decl_to_str(decl))
    print(get_decl_type(decl))
    print(get_decl_name(decl))
    print(is_using_stdint(decl))
    print(is_typedef(decl))
    test_ast = process_file("tests/regex/load_from_file/test.h", True)
    print(test_ast)
    print('\n\n'.join([decl_to_str(struct) for struct in test_ast.values()]))

    file_path = "tests/regex/load_from_file"
    file_path = "blender/source/blender/makesdna"
    alltypes = {}
    for file in os.listdir(file_path):
        if not file.endswith((".hpp", ".h")):
            continue
        alltypes.update(process_file(os.path.join(file_path, file)))
    with open("output.json", "w") as f:
        json.dump(alltypes, f)

    print(find_sep("(a,b),(b,c),(d,(e,f))", 13))
