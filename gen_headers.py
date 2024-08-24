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

This script is a bit messy but can be used as a starting point for
generating headers that contain all variations of Blender's
DNA and RNA structures.

enum extraction is included but not used

Expect this file to be heavily refactored at a later date
"""

import os
import difflib
import regex as re
import git

min_ver = (2, 93, 0)
max_ver = (4, 2, 1)
base_dir = 'blender_generated'
source_dir = base_dir
rna_dir = os.path.join(base_dir, 'makesrna')
dna_dir = os.path.join(base_dir, 'makesdna')


def prepare_directory(dir):
    os.makedirs(dir, exist_ok=True)
    for file in os.listdir(dir):
        file = os.path.join(dir, file)
        if os.path.isfile(file):
            os.remove(file)


def remove_comments(text):
    def nl(s):
        return "" + ("\n" * s.count('\n'))

    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            return nl(s)
        else:
            return s

    pattern = re.compile(
        r'\/\/.*?$|\/\*.*?\*\/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, replacer, text)


def remove_if_directives(text):
    pattern = re.compile(
        r'#if.*#endif',
        re.DOTALL
    )
    return re.sub(pattern, '', text)


def remove_macros(text):
    pattern = re.compile(
        r'\s([A-Z_]*\(\w*\))\s',
        re.DOTALL
    )
    return re.sub(pattern, '', text)


def get_versions(repo):
    version_tags = [str(tag) for tag in repo.tags]
    versions = []
    for ver_str in version_tags:
        try:
            ver = [int(v) for v in ver_str.strip('v').split('.')]
            ver_split = [0, 0, 0]
            for i in range(len(ver)):
                ver_split[i] = ver[i]
        except:
            continue
        versions.append((ver_str, tuple(ver_split)))
    versions.sort(key=lambda a: a[1])
    return versions


def get_version_index(ver, versions):
    for i in range(len(versions)):
        if versions[i][1] == ver:
            return i


def read_file(path):
    with open(path, 'r') as f:
        return remove_comments(f.read())


def extract_struct_definitions(file_content):
    struct_pattern = re.compile(
        r'(typedef\s+)?struct\s+(\w+)\s*\{(?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*\}\s*(\w+)?\s*;', re.DOTALL
    )
    matches = struct_pattern.finditer(file_content)
    return {
        match.group(2): [line for line in remove_macros(remove_if_directives(match.group(0))).splitlines()
                         if not line.strip() == '']
        for match in matches}


def sep_enum(text):
    """
    Separate text such that each substring ends at the last comma found before the next equals sign.
    Best results when the enumeration is comma separated for every element including the last.
    """
    indices = [(i, c) for i, c in enumerate(text) if c == ',' or c == '=']
    result = []
    last_comma = -1
    start = 0
    for i, (str_i, c) in enumerate(indices):
        if c == ',':
            last_comma = str_i
            continue
        if c == '=' and last_comma > -1:
            if start > last_comma or start >= len(text):
                break
            result.append(text[start:last_comma])
            start = last_comma + 1
    if start > last_comma:
        last_comma = len(text)
    result.append(text[start:last_comma])
    return result


def extract_enum_definitions(file_content):
    enum_pattern = re.compile(
        r'(typedef\s+)?enum\s+(\w+)?\s*\{(?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*\}\s*\w*\s*;', re.DOTALL
    )
    matches = enum_pattern.finditer(file_content)
    return [[line for line in remove_comments(match.group(0)).splitlines() if not line.strip() == '']
            for match in matches]


def get_structs_in_dir(path):
    source_files = [f_name for f_name in os.listdir(path) if f_name.endswith(('.hpp', '.h'))]
    output = {}
    for src in source_files:
        for struct, body in extract_struct_definitions(read_file(os.path.join(path, src))).items():
            body = ([body[0]] +
                    ['    ' + line.replace('DNA_DEPRECATED', '').strip() + ';' for line in
                     ''.join(body[1:-1]).split(';') if line.strip() != ''] +
                    [body[-1]])
            output[struct] = (body, src)
    return output


def get_enums_in_dir(path):
    source_files = [f_name for f_name in os.listdir(path) if f_name.endswith(('.hpp', '.h'))]
    output = {}
    for src in source_files:
        for lines in extract_enum_definitions(read_file(os.path.join(path, src))):
            lines = ' '.join([line for line in lines[1:-1] if not line.startswith('#')])
            lines = [line.strip() for line in sep_enum(lines)]
            lines = [line for line in lines if line != '']
            for line in lines:
                line = EnumVal(line)
                if not line.valid:
                    continue
                output[line.name] = line
    return output


class EnumVal:
    def __init__(self, line):
        pattern = re.compile(
            r'(\w+)(?:\s*=\s*(.+))', re.DOTALL
        )
        line = re.sub(r'\s+', ' ', line)
        matches = pattern.finditer(line)
        match = [(match.group(1), match.group(2)) for match in matches]
        self.valid = len(match) > 0
        if not self.valid:
            return
        self.name = match[0][0]
        self.val = match[0][1]
        self.line = line.strip()

    def __str__(self):
        return self.name + ' = ' + self.val + ','


class EnumCollection:
    def __init__(self, src_dir):
        self.dict = get_enums_in_dir(src_dir)

    def __str__(self):
        return 'enum {\n' + '\n'.join(['    ' + str(enum) for enum in self.dict.values()]) + '\n};'


class LineDef:
    primitive_types = [
        'void',
        'char',
        'short',
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
        'const'
    ]

    def __init__(self, src_line: str):
        self.valid = True
        if src_line == '' or '(' in src_line or ')' in src_line:
            self.valid = False
            return
        src_line = src_line.strip()
        src_line = src_line.replace('struct ', '').replace('DNA_DEPRECATED', '').strip()
        self.line = src_line
        self.ptr_level = src_line.count('*')
        src_line = [word for word in ' '.join(src_line.split('*')).split(' ') if word != '']
        type_dict = {p_type: src_line.count(p_type) for p_type in self.primitive_types}
        self.struct_name = ''
        self.is_struct = sum(type_dict.values()) == 0
        if self.is_struct:
            for kw in self.keywords:
                while True:
                    try:
                        src_line.remove(kw)
                    except:
                        break
            self.struct_name = src_line[0]
        self.orig_name = self.struct_name

    def set_struct_name(self, new_type):
        if not self.valid:
            return
        new_type = new_type.replace(' ', '_')
        if self.is_struct:
            self.line = self.line.replace(self.struct_name, new_type, 1)
            self.struct_name = new_type

    def __str__(self):
        return self.line

    def orig_str(self):
        if self.is_struct:
            return self.line.replace(self.struct_name, self.orig_name)
        return self.line


class StructDef:
    def __init__(self, name, body):
        self.valid = True
        self.name = name
        self.orig_name = name
        self.body = [LineDef(line) for line in body]
        self.valid = all(line.valid for line in self.body)

    def __str__(self):
        return 'struct ' + self.name + ' {\n' + '\n'.join(['    ' + str(line) for line in self.body]) + '\n};'

    def orig_str(self):
        return 'struct ' + self.orig_name + ' {\n' + '\n'.join(
            ['    ' + line.orig_str() for line in self.body]) + '\n};'


class StructCollection:
    def __init__(self, src_dir):
        self.dict = {name: StructDef(name, body[1:-1]) for name, (body, src) in get_structs_in_dir(src_dir).items()}

    def reduce(self, old_structs=None):
        while True:
            known_structs = set(self.dict.keys())
            finished = True
            for struct in known_structs:
                struct = self.dict[struct]
                valid = struct.valid
                for line in struct.body:
                    if not valid:
                        break
                    if line.is_struct and line.orig_name not in self.dict:
                        if line.orig_name in old_structs:
                            line.set_struct_name(old_structs[line.orig_name].name)
                            continue
                        elif line.ptr_level > 0:
                            l = line.line.split()
                            for i, word in enumerate(l):
                                if i < len(l) - 1:
                                    l[i] = word.replace(line.struct_name, 'void')
                            line.line = ' '.join(l)
                            line.is_struct = False
                        else:
                            valid = False
                if not valid:
                    self.dict.pop(struct.orig_name)
                    finished = False
            if finished:
                break

    def __str__(self):
        no_ref_structs = set()
        for name, struct in self.dict.items():
            no_ref = True
            for line in struct.body:
                if line.is_struct and line.ptr_level == 0:
                    no_ref = False
                    break
            if no_ref:
                no_ref_structs.add(name)

        def keyfunc(struct):
            order = 0
            for line in struct.body:
                if line.is_struct and line.ptr_level == 0:
                    if order < 1 and line.orig_name in no_ref_structs:
                        order = 1
                    elif order < 2 and line.orig_name not in no_ref_structs:
                        order = 2
            return order

        return '\n\n'.join(str(struct) for struct in sorted(list(self.dict.values()), key=keyfunc))

    def set_version_suffix(self, suffix):
        for name, struct in self.dict.items():
            if not struct.valid:
                continue
            for line in struct.body:
                if not line.valid:
                    continue
                line.set_struct_name(line.orig_name + suffix)
            struct.name = name + suffix


class Header:
    def __init__(self, ver='', prefix='makesdna'):
        self.structs = None
        self.enums = None
        self.ver = ver.replace(".", "_").strip("v")
        self.old_structs = None
        self.prefix = prefix

    def load_structs(self, src_dir):
        self.structs = StructCollection(src_dir)
        self.structs.set_version_suffix(self.ver)

    def load_enums(self, src_dir):
        self.enums = EnumCollection(src_dir)

    def dedupe_structs(self, old_structs):
        self.structs.reduce(old_structs)
        known_structs = list(self.structs.dict.keys())
        new_structs = self.structs.dict.copy()
        for name in known_structs:
            struct = self.structs.dict[name]
            struct_lines = struct.orig_str().splitlines()
            if name in old_structs:
                old_lines = old_structs[name].orig_str().splitlines()
                matcher = difflib.SequenceMatcher(None, struct_lines, old_lines)
                if matcher.ratio() == 1.0:
                    new_structs.pop(name)
                    continue
            old_structs[name] = struct
        while True:
            new_count = len(new_structs)
            for name in known_structs:
                if name in new_structs:
                    continue
                struct = self.structs.dict[name]
                changed = False
                for line in struct.body:
                    if line.is_struct and line.ptr_level == 0 and line.orig_name in new_structs:
                        line.set_struct_name(new_structs[line.orig_name].name)
                        changed = True
                if changed:
                    new_structs[name] = struct
            if len(new_structs) == new_count:
                break
        self.structs.dict = new_structs
        self.old_structs = old_structs

    def __str__(self):
        header_name = f'{self.prefix}_types_{self.ver}.h'
        guard = header_name.replace('.', '_').upper()
        header_body = [
            f'#ifndef {guard}\n',
            f'#define {guard}\n',
            '\n'
        ]
        self.structs.reduce(self.old_structs)
        includes = set()
        for struct in self.structs.dict.values():
            for line in struct.body:
                if line.is_struct and not line.struct_name.endswith(self.ver):
                    includes.add(line.struct_name.replace(line.orig_name, ''))
        header_body.append("#include <stdint.h>\n")
        header_body.extend([f'#include "{self.prefix}_types_{line}.h"\n' for line in includes])
        header_body.append('\n')
        if self.enums is not None:
            header_body.append(str(self.enums) + '\n\n')
        if self.structs is not None:
            header_body.append('\n'.join(['struct ' + struct.name + ';' for struct in self.structs.dict.values()]))
            header_body.append('\n\n' + str(self.structs))
        header_body.append('\n\n#endif\n')
        return ''.join(header_body)

    def is_empty(self):
        structs = self.structs is not None and self.structs.dict
        enums = self.enums is not None and self.enums.dict
        return not (structs or enums)


def main():
    print('Updating blender repository...')
    if os.path.exists('blender/.git'):
        blender = git.Repo('blender')
    else:
        blender = git.Repo.clone_from('https://github.com/blender/blender.git', 'blender')
    blender.remotes.origin.fetch()
    prepare_directory(source_dir)
    prepare_directory(rna_dir)
    prepare_directory(dna_dir)
    versions = get_versions(blender)
    min_ver_idx = get_version_index(min_ver, versions)
    max_ver_idx = get_version_index(max_ver, versions)
    old_structs_dna = {}
    old_structs_rna = {}
    headers_dna = []
    headers_rna = []
    for i in range(min_ver_idx, max_ver_idx + 1):
        ver = versions[i][0]
        print(f'Checking out blender version {ver}...')
        blender.git.checkout(versions[i][0], '-f')
        header_dna = Header(ver)
        header_dna.load_structs('blender/source/blender/makesdna')
        header_dna.dedupe_structs(old_structs_dna)
        if not header_dna.is_empty():
            headers_dna.append(header_dna)
            header_name = f'makesdna_types_{ver.replace(".", "_").strip("v")}.h'
            header_path = os.path.join(dna_dir, header_name)
            with open(header_path, 'w') as f:
                f.writelines(str(header_dna))
        header_rna = Header(ver, 'makesrna')
        header_rna.load_structs('blender/source/blender/makesrna')
        header_rna.dedupe_structs(old_structs_rna)
        if header_rna.is_empty():
            continue
        headers_rna.append(header_rna)
        header_name = f'makesrna_types_{ver.replace(".", "_").strip("v")}.h'
        header_path = os.path.join(rna_dir, header_name)
        with open(header_path, 'w') as f:
            f.writelines(str(header_rna))
    include_all_name = 'blender_types.hpp'
    include_all_path = os.path.join(source_dir, include_all_name)
    include_all_guard = include_all_name.replace('.', '_').upper()
    include_all_body = [
        f'#ifndef {include_all_guard}\n',
        f'#define {include_all_guard}\n\n'
    ]
    ver_enum = ['\nenum class BlenderVersion {\n']
    versions = set()
    for header_dna in headers_dna:
        include_all_body.append(f'#include "makesdna/makesdna_types_{header_dna.ver}.h"\n')
        versions.add(header_dna.ver)
    for header_rna in headers_rna:
        include_all_body.append(f'#include "makesrna/makesrna_types_{header_rna.ver}.h"\n')
        versions.add(header_rna.ver)
    for ver in sorted(list(versions)):
        ver_enum.append('    VER_' + ver + ',\n')
    ver_enum.append('};\n\n')
    include_all_body.extend(ver_enum)
    include_all_body.append('#endif\n')
    with open(include_all_path, 'w') as f:
        f.writelines(include_all_body)


if __name__ == '__main__':
    main()
