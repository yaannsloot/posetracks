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


def extract_struct_definitions(file_content):
    struct_pattern = re.compile(
        r'(typedef\s+)?struct\s+(\w+)\s*\{(?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*\}\s*(\w+)?\s*;', re.DOTALL
    )
    matches = struct_pattern.finditer(file_content)
    return {
        match.group(2): [line for line in remove_macros(remove_if_directives(match.group(0))).splitlines()
                         if not line.strip() == '']
        for match in matches}


def extract_enum_definitions(file_content):
    enum_pattern = re.compile(
        r'(typedef\s+)?enum\s+(\w+)?\s*\{(?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*\}\s*\w*\s*;', re.DOTALL
    )
    matches = enum_pattern.finditer(file_content)
    return [[line for line in remove_comments(match.group(0)).splitlines() if not line.strip() == '']
            for match in matches]


def src_structs_to_dict(src_content):
    output = {}
    content = remove_macros(remove_comments(src_content))
    for struct, body in extract_struct_definitions(content).items():
        body = ([body[0]] +
                ['    ' + line.replace('DNA_DEPRECATED', '').strip() + ';' for line in
                 ''.join(body[1:-1]).split(';') if line.strip() != ''] +
                [body[-1]])
        output[struct] = body
    return output
    

def load_structs_from_file(path):
    with open(path, 'r') as f:
        src = f.read()
    return src_structs_to_dict(src)
