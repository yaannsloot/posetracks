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

Extra C struct declaration types for processing source code
"""

import regex as re


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


class LineDef:
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
        'const'
    ]

    def __init__(self, src_line: str):
        self.valid = True
        if src_line == '' or '(' in src_line or ')' in src_line:
            self.valid = False
            return
        src_line = src_line.strip()
        src_line = re.sub('\t+', ' ', src_line)

        # A bandaid to a problem that should be investigated
        if src_line.startswith('{'):
            self.valid = False

        src_line = src_line.replace('struct ', '').replace(
            'DNA_DEPRECATED', '').strip()
        self.line = src_line
        self.ptr_level = src_line.count('*')
        src_line = [word for word in ' '.join(
            src_line.split('*')).split(' ') if word != '']
        type_dict = {p_type: src_line.count(p_type)
                     for p_type in self.primitive_types}
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


def from_mapping(struct_dict, keep_invalid=False):
    output = {}
    for n, s in struct_dict.items():
        s_def = StructDef(n, s[1:-1])
        if not s_def.valid and not keep_invalid:
            continue
        output[n] = s_def
    return output
