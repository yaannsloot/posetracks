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

Regex functions for processing C header source code
"""

import regex as re
from collections import UserDict
from enum import Enum
from . import git


def remove_comments(text):
    pattern = re.compile(
        r'\/\/.*?$|\/\*.*?\*\/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, "", text)


def remove_if_directives(text):
    pattern = re.compile(
        r'#ifdef.*?#endif',
        re.DOTALL
    )
    return re.sub(pattern, '', text)


def remove_macros(text):
    pattern = re.compile(
        r'#define[ \t]+[A-Za-z_][A-Za-z0-9_]*(?:\(.*?\))?(?:[ \t]+.*?|\\\n.*?)*(?=\n|$)',
        re.DOTALL
    )
    return re.sub(pattern, '', text)


def find_structs(file_content):
    struct_pattern = re.compile(
        r'(?:typedef\s+)?struct\s*(\w+)?\s*\{((?:[^{}]*|\{(?:[^{}]*|\{[^{}]*\})*\})*)\}\s*(\w+)?\s*;', re.DOTALL
    )
    matches = struct_pattern.finditer(file_content)
    output = {}
    for match in matches:
        name = match.group(1)
        body = match.group(2)
        alias = match.group(3)
        tag = name or alias
        output[tag] = {"body": body, "match": match.group(
        ), "replacement": f"struct {tag} {'{}'};"}
    return output


def serialize_struct_body(struct_body):
    structs = find_structs(struct_body)
    for body in structs.values():
        struct_body = struct_body.replace(body["match"], body["replacement"])
    for line in struct_body.split(';'):
        line = line.replace('DNA_DEPRECATED', '').strip() + ';'
        if line == ';':
            continue
        struct = find_structs(line)
        if struct:
            tag = next(iter(struct.keys()))
            yield {tag: list(serialize_struct_body(structs[tag]["body"]))}
        else:
            line = re.sub(r'\s+', ' ', line)
            line = re.sub(r'(\w+)\s*\*\s*(\w+)', r'\1* \2', line)
            yield line


def remove_include_guard(file_content):
    pattern = re.compile(
        r"#ifndef\s+([A-Z][A-Z0-9_]*)\s+#define\s+\1\s+(.*\n)*?#endif\s*$", re.DOTALL
    )
    match = next(pattern.finditer(file_content), None)
    if match is None:
        return file_content
    match = match.group(2)
    return re.sub(pattern, match, file_content)


def strip_src(content):
    content = remove_comments(content)
    content = remove_include_guard(content)
    content = remove_if_directives(content)
    content = remove_macros(content)
    return content


def extract_struct_definitions(file_content):
    file_content = strip_src(file_content)
    structs = find_structs(file_content)
    structs = {tag: list(serialize_struct_body(
        body["body"])) for tag, body in structs.items()}
    return structs


def extract_enum_definitions(file_content):
    enum_pattern = re.compile(
        r'(typedef\s+)?enum\s+(\w+)?\s*\{(?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*\}\s*\w*\s*;', re.DOTALL
    )
    matches = enum_pattern.finditer(file_content)
    return [[line for line in remove_comments(match.group(0)).splitlines() if not line.strip() == '']
            for match in matches]


def load_structs_from_file(path):
    with open(path, 'r') as f:
        src = f.read()
    return extract_struct_definitions(src)


def load_structs_from_dir(path):
    structs = {}
    for header in git.list_headers(path):
        structs.update(load_structs_from_file(header))
    return structs


primitive_types = [
    'void',
    'bool',
    'char',
    'short',
    'ushort',
    'int',
    'long',
    'float',
    'double',
    'int8_t',
    'uint8_t',
    'int16_t',
    'uint16_t',
    'int32_t',
    'uint32_t',
    'int64_t',
    'uint64_t'
]

keywords = [
    'signed',
    'unsigned',
    'const',
    'struct'
]


class Type(Enum):
    PRIMITIVE = 0
    USER_DEF = 1
    NESTED_STRUCTURE = 2


def get_ptr_level(line: str):
    return line.count('*')


def get_typename(line):
    for kw in keywords:
        line = line.replace(kw, '')
    line = [word for word in line.replace('*', ' ').split(' ') if word != '']
    return line[0]


def get_src_type(src):
    if isinstance(src, dict):
        return Type.NESTED_STRUCTURE
    matches = {p_type: src.count(p_type)
               for p_type in primitive_types}
    if sum(matches.values()) == 0:
        return Type.USER_DEF
    return Type.PRIMITIVE


def find_user_deps(i, output):
    for item in i:
        t = get_src_type(item)
        if t == Type.NESTED_STRUCTURE:
            find_user_deps(next(iter(item.values())), output)
            continue
        elif t == Type.PRIMITIVE:
            continue
        tn = get_typename(item)
        is_ptr = get_ptr_level(item) > 0
        val = output.setdefault(tn, False)
        if not val:
            output[tn] = is_ptr


def get_dependencies(src):
    output = {}
    for tag, lines in src.items():
        dependencies = {}
        find_user_deps(lines, dependencies)
        if not dependencies:
            continue
        output[tag] = dependencies
    return output

def deserialize_lines(lines, out, indent):
    i = " " * (indent * 4)
    for line in lines:
        t = get_src_type(line)
        if t == Type.NESTED_STRUCTURE:
            tag, nested = next(iter(line.items()))
            out.append(i + "struct {")
            deserialize_lines(nested, out, indent + 1)
            out.append(i + f"{'}'} {tag};")
            continue
        elif t == Type.USER_DEF:
            line = line.replace('struct', '').strip()
        out.append(i + line)

def deserialize_structs(structs):
    output = {}
    for tag, lines in structs.items():
        body = [f"struct {tag} {'{'}"]
        deserialize_lines(lines, body, 1)
        body.append("};")
        output[tag] = '\n'.join(body)
    return output