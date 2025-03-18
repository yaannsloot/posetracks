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
import copy
import struct
import textwrap
import clang.cindex
from bisect import bisect
from typing import Any, List, Union
from collections import deque

import regex
from yaml import scan

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
kw_pattern = r'^(' + "|".join(map(re.escape, kw_pattern)) + ")"


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


def find_group(text):
    start = -1
    stop = -1
    level = 0
    for i in range(len(text) - 1, -1, -1):
        if text[i] == ")":
            if level == 0:
                stop = i + 1
            level += 1
        elif text[i] == "(":
            level -= 1
        if level == 0 and stop != -1:
            start = i
            break
    return start, stop


def lfind_group(text):
    start = -1
    stop = -1
    level = 0
    for i in range(len(text)):
        if text[i] == "(":
            if level == 0:
                start = i
            level += 1
        elif text[i] == ")":
            level -= 1
        if level == 0 and start != -1:
            stop = i + 1
            break
    return start, stop


class Parenthesis:
    def to_dict(self):
        return {"type": "parenthesis"}

    @staticmethod
    def __call__(text):
        return f"({text})"

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        group = find_group(text)
        if group[0] == 0 and group[1] == len(text):
            return text[group[0] + 1: group[1] - 1], cls()
        return text, None


class Function:
    # TODO: add params
    def __init__(self):
        pass

    def __call__(self, text):
        return f"{text}()"

    def to_dict(self):
        return {"type": "function"}

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        group = find_group(text)
        if group[0] != -1 and group[1] == len(text):
            return text[:group[0]], cls()
        return text, None


class Array:
    def __init__(self, size: int = None):
        self.size = size if size is not None else None

    def __call__(self, text):
        return f"{text}[{self.size if self.size is not None else ''}]"

    def to_dict(self):
        return {"type": "array", "size": self.size}

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        pattern = r'\[([^\[\]]*)\]$'
        match = re.search(pattern, text)
        size = None
        if match:
            size = match.group(1).strip()
            size = int(size) if size else None
            text = re.sub(pattern, '', text, 1)
        return text, cls(size) if match else None


class Pointer:
    def to_dict(self):
        return {"type": "pointer"}

    @staticmethod
    def __call__(text):
        return f"*{text}"

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        pattern = r'^(\*)'
        match = re.match(pattern, text)
        if match:
            text = re.sub(pattern, '', text, 1)
        return text, cls() if match else None


def _a(cursor):
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


def _match_any(text):
    cases = [Pointer, Array, Parenthesis, Function]
    for case in cases:
        text, match = case.from_str(text)
        if match:
            if isinstance(match, Function):
                raise ValueError("Functions are not supported")
            break
    return text, match


def _get_structure(text):
    structure = []
    while True:
        text, match = _match_any(text)
        if not match:
            break
        structure.append(match)
    return text, structure


def _next_word(text):
    pattern = r"^\s*([\w\d]+)\s+"
    match = re.match(pattern, text)
    if match:
        return re.sub(pattern, '', text, 1), match.group(1)
    return text, None


def _get_type(text):
    text, keyword = match_kw(text)
    if not keyword:
        return text, None, None
    if keyword in tagged:
        text, tag = _next_word(text)
        return text, keyword, tag
    return text, keyword, None


def _strip_body(text):
    return re.sub(r"^.*}", '', text, 1)


class VarType:
    def __init__(self, keyword: str, tag: str = None,
                 structure: List[Union[Parenthesis, Array, Pointer]] = None):
        self.keyword = keyword.strip()
        self.tag = tag.strip() if tag else None
        self.structure = structure if structure else []

    def is_tagged(self):
        return bool(self.tag)

    def reduce(self):
        new_structure = []
        for mod in reversed(self.structure):
            if isinstance(mod, (Pointer, Array)):
                new_structure.insert(0, mod)
                if isinstance(mod, Pointer):
                    break
        if not new_structure or not isinstance(new_structure[0], Pointer):
            raise IrreducibleVariableError(
                "Variable is not a pointer and cannot be reduced")
        self.keyword = "void"
        self.tag = None
        self.structure = new_structure

    def __str__(self):
        return self.keyword + (f" {self.tag}" if self.tag else '')

    def __call__(self, text):
        for s in self.structure:
            text = s(text)
        return self.keyword + (f" {self.tag}" if self.tag else '') + f" {text}"

    def is_stdint(self):
        return self.keyword in stdint

    def to_dict(self):
        return {"keyword": self.keyword, "tag": self.tag,
                "structure": [s.to_dict() for s in self.structure]}

    @classmethod
    def from_str(cls, text):
        text, keyword, tag = _get_type(text)
        text = _strip_body(text).replace(';', '')
        separators = find_sep(text)
        if separators:
            text = text[separators[-1] + 1:]
        text, structure = _get_structure(text)
        return text, cls(keyword, tag, structure)

    @classmethod
    def from_cursor(cls, cursor):
        if cursor.type.get_declaration().kind == clang.cindex.CursorKind.TYPEDEF_DECL:
            text = cursor.type.get_declaration().underlying_typedef_type.spelling
        else:
            text = cursor.type.get_canonical().spelling
        pattern = r'^(const\ )'
        const = re.match(pattern, text)
        if const:
            text = re.sub(pattern, '', text, 1)
        text, keyword, tag = _get_type(text)
        if keyword in tagged and not tag:
            start, end = lfind_group(text)
            if start == -1 or end == -1:
                tag = text
                text = ''
            else:
                tag = text[start:end]
                text = text[end:]
        text, structure = _get_structure(text)
        return cls(keyword, tag, structure)


class IrreducibleVariableError(Exception):
    pass


class Variable:
    def __init__(self, type: VarType, name: str, const: bool = False):
        self.type = type
        self.name = name
        self.const = const

    def __str__(self):
        left = "const " if self.const else ""
        right = self.type(self.name)
        return f"{left} {right};".strip()

    def voidable(self):
        for mod in self.type.structure:
            if isinstance(mod, Pointer):
                return True
        return False

    def to_dict(self):
        return {"type": self.type.to_dict(), "name": self.name, "is_const": self.const}

    @classmethod
    def from_str(cls, text, typedefs=None, cursor=None):
        text = normalize_text(text)
        pattern = r'^(const\ )'
        const = bool(re.match(pattern, text))
        if const:
            text = re.sub(pattern, '', text, 1)
        try:
            text, type = VarType.from_str(text, typedefs, cursor)
        except Exception as e:
            raise e.__class__(f'In decl "{text}": {str(e)}')
        structure = []
        while True:
            text, match = _match_any(text)
            if not match:
                break
            structure.append(match)
        return cls(type, text, const, structure)

    @classmethod
    def from_cursor(cls, cursor):
        name = cursor.spelling
        type = VarType.from_cursor(cursor)
        if cursor.type.get_declaration().kind == clang.cindex.CursorKind.TYPEDEF_DECL:
            text = cursor.type.get_declaration().underlying_typedef_type.spelling
        else:
            text = cursor.type.get_canonical().spelling
        const = bool(re.match(r'^(const\ )', text))
        return cls(type, name, const)


def is_function(cursor):
    return cursor.kind in {clang.cindex.CursorKind.FUNCTION_DECL,
                           clang.cindex.CursorKind.CXX_METHOD}


def is_function_pointer(cursor):
    type_ = cursor.type
    return (type_.kind == clang.cindex.TypeKind.POINTER and
            type_.get_pointee().kind in {clang.cindex.TypeKind.FUNCTIONPROTO,
                                         clang.cindex.TypeKind.FUNCTIONNOPROTO})


def cursor_has_func(cursor):
    if is_function(cursor) or is_function_pointer(cursor):
        return True
    for child in cursor.get_children():
        if cursor_has_func(child):
            return True
    return False


class Struct:
    def __init__(self, name, fields: List[Union[Variable, "Struct"]] = None):
        self.name = name
        self.fields = fields if fields else []

    def dependencies(self, include_pointers=True):
        local_structs = set()
        dependencies = set()

        def traverse(struct):
            local_structs.add(struct.name)
            for field in struct.fields:
                if isinstance(field, Struct):
                    traverse(field)
                elif field.type.is_tagged():
                    if not include_pointers and field.voidable():
                        continue
                    dependencies.add(field.type.tag)
        traverse(self)
        return dependencies - local_structs

    def void_dependency(self, name):
        for field in self.fields:
            if isinstance(field, Struct):
                field.void_dependency(name)
            elif field.type.is_tagged() and field.type.tag == name:
                field.type.reduce()

    def uses_stdint(self):
        for field in self.fields:
            if ((isinstance(field, Struct) and field.uses_stdint()) or
                    (isinstance(field, Variable) and field.type.is_stdint())):
                return True
        return False

    def __eq__(self, value):
        if isinstance(value, Struct):
            return str(self) == str(value)
        return False

    def __str__(self):
        body = ""
        last_struct = None
        for field in self.fields:
            body += '\n' if body else ''
            field_str = str(field)
            body += field_str
            if isinstance(field, Struct):
                last_struct = field.name
            elif last_struct and field.type.is_tagged() and field.type.tag == last_struct:
                if body.endswith(f"{'}'};\n{field_str}"):
                    body = body.removesuffix(f"{'}'};\n{field_str}")
                    body += "}" + \
                        field_str.removeprefix(f'struct {last_struct}')
                else:
                    body = body.replace(
                        f";\n{field_str}", f",{field_str.removeprefix(f'struct {last_struct}')}")
            else:
                last_struct = None
        begin = f"struct{' ' + self.name if '(unnamed ' not in self.name else ''} {'{'}"
        return '\n'.join([begin, textwrap.indent(body, '    '), '};'])

    def to_dict(self):
        return {"name": self.name, "fields": [item.to_dict() for item in self.fields],
                "dependencies": list(self.dependencies())}

    @classmethod
    def from_cursor(cls, cursor):
        if cursor.kind != clang.cindex.CursorKind.STRUCT_DECL:
            raise ValueError("Cursor is not a struct declaration")
        fields = []
        for field in cursor.get_children():
            if field.kind == clang.cindex.CursorKind.FIELD_DECL:
                fields.append(Variable.from_cursor(field))
            elif field.kind == clang.cindex.CursorKind.STRUCT_DECL:
                struct = cls.from_cursor(field)
                if struct.fields:
                    fields.append(struct)
        name = cursor.spelling
        if name.startswith("struct "):
            name = name[7:].strip()
        return cls(name, fields)


class SyntaxTree:
    def __init__(self):
        self.items = {}

    def add(self, item):
        if not hasattr(item, "name") and (isinstance(item, Revision) and not hasattr(item.ref, "name")):
            raise ValueError("Item has no name attribute")
        if ((isinstance(item, Struct) and not item.fields) or
                (isinstance(item, Revision) and isinstance(item.ref, Struct) and not item.ref.fields)):
            return
        if isinstance(item, Revision):
            self.items[item.ref.name] = item
        else:
            self.items[item.name] = item

    def merge(self, other: "SyntaxTree"):
        for item in other.items.values():
            self.add(item)

    def dependency_graph(self, include_pointers=True, with_leafs=False):
        dependencies = {}
        for item in self.items.values():
            if isinstance(item, Revision):
                item = item.ref
            if not isinstance(item, Struct):
                continue
            if with_leafs:
                dependencies.setdefault(item.name, set())
            for dependency in item.dependencies(include_pointers):
                depends = dependencies.setdefault(dependency, set())
                depends.add(item.name)
        return dependencies

    def structs(self):
        structs = {}
        for item in self.items.values():
            if isinstance(item, Revision):
                item = item.ref
            if not isinstance(item, Struct):
                continue
            structs[item.name] = item
        return structs

    def get_fixed_tree(self):
        new_tree = copy.deepcopy(self)
        names = set(new_tree.items.keys())
        dependencies = new_tree.dependency_graph()
        structs = new_tree.structs()
        while new_tree.items:
            missing_refs = set(dependencies.keys()) - names
            if not missing_refs:
                break
            for missing in missing_refs:
                for dependent in dependencies[missing]:
                    struct = structs.get(dependent)
                    if not struct:
                        continue
                    try:
                        struct.void_dependency(missing)
                    except IrreducibleVariableError:
                        new_tree.items.pop(struct.name, None)
                        names.discard(struct.name)
                dependencies.pop(missing, None)
        return new_tree

    def get_order(self):
        dependencies = self.dependency_graph(False, True)
        degrees = {name: 0 for name in dependencies.keys()}
        for children in dependencies.values():
            for child in children:
                degrees[child] = degrees.get(child, 0) + 1
        order = []
        last_len = len(degrees)
        while degrees:
            for name, degree in degrees.copy().items():
                if degree <= 0:
                    children = dependencies.get(name, set())
                    for child in children:
                        degrees[child] -= 1
                    degrees.pop(name, None)
                    if name in self.items:
                        order.append(name)
            if len(degrees) == last_len:
                raise RuntimeError('Cycle detected while sorting graph')
            last_len = len(degrees)
        return order

    def get_versions(self):
        versions = set()
        for item in self.items.values():
            if isinstance(item, Revision):
                versions.add(item.ver)
        return sorted(list(versions))

    def get_transitive_updates(self):
        output = set()
        versions = self.get_versions()
        if not versions or len(versions) == 1:
            return output
        latest = versions[-1]
        dependencies = self.dependency_graph()
        queue = deque()
        for dependency, dependents in dependencies.items():
            obj = self.items.get(dependency)
            if obj and isinstance(obj, Revision) and obj.ver == latest:
                queue.append(dependency)
        current_deps = set(queue)
        while queue:
            dependency = queue.popleft()
            if dependency in output:
                continue
            if dependency not in current_deps:
                output.add(dependency)
            for dependent in dependencies.get(dependency, []):
                obj = self.items.get(dependent)
                if obj and isinstance(obj, Revision) and obj.ver != latest:
                    queue.append(dependent)
        return output

    def uses_stdint(self):
        for item in self.items.values():
            if isinstance(item, Revision):
                item = item.ref
            if not isinstance(item, Struct):
                continue
            if item.uses_stdint():
                return True
        return False

    @classmethod
    def from_file(cls, path):
        index = clang.cindex.Index.create()
        tu = index.parse(
            path,
            args=["-E", "-CC", "-detailed-preprocessing-record"],
        )
        tree = cls()
        for cursor in tu.cursor.get_children():
            try:
                # scan_cursor is temporarily acting as a guardrail to prevent processing of unsupported cursor types
                if cursor.kind == clang.cindex.CursorKind.STRUCT_DECL and not cursor_has_func(cursor):
                    tree.add(Struct.from_cursor(cursor))
            except ValueError as e:
                print(f"Encountered error while processing {path}:", e)
        return tree

    def to_dict(self):
        return {name: item.to_dict() for name, item in self.items.items()}


class Revision:
    def __init__(self, ref: Any, ver: Union[int, float, tuple]):
        self.ref = ref
        self.ver = ver

    def compare_refs(self, other: "Revision"):
        if isinstance(self.ref, Struct) and isinstance(other.ref, Struct):
            return str(self.ref) == str(other.ref)
        return self.ref == other.ref

    def to_dict(self):
        return {"type": "revision", "ver": list(self.ver) if isinstance(self.ver, tuple) else self.ver,
                "obj": self.ref.to_dict()}


class ItemRemoval:
    def __init__(self, name: str):
        self.name = name

    def __eq__(self, value):
        if isinstance(value, ItemRemoval):
            return self.name == value.name
        return False

    def to_dict(self):
        return {"type": "removal_tag"}


class DependencyUpdate:
    def __init__(self, name):
        self.name = name

    def to_dict(self):
        return {"type": "dependency_update"}


class VersionedSyntaxTree:
    def __init__(self):
        self.master_tree = {}

    def add_tree(self, ast: SyntaxTree, ver):
        for item in ast.items.values():
            self.add(item, ver)
        for name in set(self.master_tree.keys()) - set(ast.items.keys()):
            self.add(ItemRemoval(name), ver)
        transitive_updates = self.get_tree(ver).get_transitive_updates()
        for name in transitive_updates:
            self.add(DependencyUpdate(name), ver)

    def add(self, item, ver):
        if not hasattr(item, "name"):
            raise ValueError("Item has no name attribute")
        branch = self.master_tree.setdefault(item.name, [])
        insertion_point = bisect(branch, ver, key=lambda a: a.ver)
        prev = branch[insertion_point - 1] if insertion_point > 0 else None
        if prev and (prev.ver == ver or prev.ref == item or
                     (isinstance(prev.ref, DependencyUpdate) and self.get(prev.ref.name, prev.ver).ref == item)):
            return
        branch.insert(insertion_point, Revision(item, ver))

    def get_tree(self, ver):
        sub_tree = SyntaxTree()
        for name in self.master_tree.keys():
            try:
                item = self.get(name, ver)
                if not isinstance(item.ref, ItemRemoval):
                    sub_tree.add(item)
            except KeyError:
                pass

        return sub_tree

    def get(self, name, ver=None):
        if name not in self.master_tree:
            raise KeyError(f"{name} not in master tree")
        branch = self.master_tree[name]
        if ver is None:
            return branch[-1]
        target = bisect(self.master_tree[name], ver, key=lambda a: a.ver) - 1
        original_target = target
        while True:
            if target < 0:
                raise KeyError(
                    f"{ver} is less than lowest version {branch[0].ver}")
            if isinstance(branch[target].ref, DependencyUpdate):
                target -= 1
            else:
                break
        ref = copy.copy(branch[target])
        ref.ver = branch[original_target].ver
        return ref

    def get_versions(self):
        versions = set()
        for branch in self.master_tree.values():
            for item in branch:
                if not isinstance(item.ref, ItemRemoval):
                    versions.add(item.ver)
        return sorted(list(versions))

    def to_dict(self):
        return {name: [item.to_dict() for item in revs] for name, revs in self.master_tree.items()}


if __name__ == "__main__":
    test_a = "a ()((afaef) (asdasd))"
    print(Function.from_str(test_a))

    input = "struct piss (*cock)[8];"
    print(f"input: {input}")
    name, type = VarType.from_str(input)
    print(f"var name: {name}")
    print(f"type obj: {type.to_dict()}")
    print(f"reconstructed: {type(name)}")

    path = "tests/regex/load_from_file/test.h"
    index = clang.cindex.Index.create()
    tu = index.parse(
        path,
        args=["-E", "-CC", "-detailed-preprocessing-record"],
    )

    def print_cursors(cursor, level=0):
        print(level, str(cursor.kind))
        if cursor.kind == clang.cindex.CursorKind.TYPEDEF_DECL or cursor.kind == clang.cindex.CursorKind.FIELD_DECL:
            print(cursor.spelling)
            print(cursor.type.get_canonical().spelling)
            if cursor.type.get_declaration().kind == clang.cindex.CursorKind.TYPEDEF_DECL:
                print(cursor.type.get_declaration(
                ).underlying_typedef_type.spelling)
            print(cursor.underlying_typedef_type.spelling)
        for child in cursor.get_children():
            print_cursors(child, level + 1)

    print_cursors(tu.cursor)

    path = "tests/regex/load_from_file/test.h"
    syntax_tree = SyntaxTree.from_file(path)
    print(json.dumps(syntax_tree.to_dict()))

    for struct in syntax_tree.items.values():
        print(struct)
