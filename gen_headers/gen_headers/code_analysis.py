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
from sys import version
import textwrap
from webbrowser import get
import clang.cindex
from bisect import bisect
from typing import Any, List, Union
from collections import deque

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


class Parenthesis:
    def to_dict(self):
        return {"type": "parenthesis"}

    @staticmethod
    def wrap(text):
        return f"({text})"

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        pattern = r'^\((.+)\)$'
        match = re.match(pattern, text)
        if match:
            text = match.group(1).strip()
        return text, cls() if match else None


class Array:
    def __init__(self, size: int = None):
        self.size = size if size is not None else None

    def wrap(self, text):
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
    def wrap(text):
        return f"*{text}"

    @classmethod
    def from_str(cls, text):
        text = normalize_text(text)
        pattern = r'^(\*)'
        match = re.match(pattern, text)
        if match:
            text = re.sub(pattern, '', text, 1)
        return text, cls() if match else None


def _extract_inline_type(text, cursor):
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


class VarType:
    def __init__(self, keyword: str, tag: str = None):
        self.keyword = keyword.strip()
        self.tag = tag.strip() if tag else None

    def is_tagged(self):
        return bool(self.tag)

    def __str__(self):
        return self.keyword + (f" {self.tag}" if self.tag else '')

    def is_stdint(self):
        return self.keyword in stdint

    def to_dict(self):
        return {"keyword": self.keyword, "tag": self.tag}

    @classmethod
    def from_str(cls, text, typedefs=None, cursor=None):
        text, type, tag = _extract_inline_type(normalize_text(text), cursor)
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
            typedef = typedefs.get(type)
            if not typedef:
                raise ValueError(f'Typedef "{type}" not found.')
            return cls.from_str(re.sub(pattern, typedef + ' ', text, 1))
        if not tag and type in tagged:
            match = re.match(pattern, text)
            if not match:
                raise ValueError(f"Tag not found for {type}.")
            tag = match.group(1).strip()
            text = re.sub(pattern, '', text, 1)
        separators = find_sep(text)
        if separators:
            text = text[separators[-1] + 1:]
        return text, cls(type, tag)


def _match_any(text):
    cases = [Pointer, Array, Parenthesis]
    for case in cases:
        text, match = case.from_str(text)
        if match:
            break
    return text, match


class IrreducibleVariableError(Exception):
    pass


class Variable:
    def __init__(self, type: VarType, name: str, const: bool = False,
                 structure: List[Union[Parenthesis, Array, Pointer]] = None):
        self.type = type
        self.name = name
        self.const = const
        self.structure = structure if structure else []

    def __str__(self):
        left = f"const {str(self.type)}" if self.const else str(self.type)
        right = self.name
        for s in reversed(self.structure):
            right = s.wrap(right)
        return f"{left} {right};"

    def voidable(self):
        for mod in self.structure:
            if isinstance(mod, Pointer):
                return True
        return False

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
        self.type = VarType("void")
        self.structure = new_structure

    def to_dict(self):
        return {"type": self.type.to_dict(), "name": self.name, "is_const": self.const,
                "structure": [item.to_dict() for item in self.structure]}

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
                field.reduce()

    def uses_stdint(self):
        for field in self.fields:
            if isinstance(field, Struct) and field.uses_stdint():
                return True
            if field.type.is_stdint():
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
    def from_cursor(cls, cursor, typedefs=None):
        if cursor.kind != clang.cindex.CursorKind.STRUCT_DECL:
            raise ValueError("Cursor is not a struct declaration")
        fields = []
        for field in cursor.get_children():
            if field.kind == clang.cindex.CursorKind.FIELD_DECL:
                line = ' '.join(
                    [token.spelling for token in field.get_tokens()])
                fields.append(Variable.from_str(line, typedefs, field))
            elif field.kind == clang.cindex.CursorKind.STRUCT_DECL:
                inner_td_scope = typedefs.copy()
                struct = cls.from_cursor(field, inner_td_scope)
                if struct.fields:
                    fields.append(struct)
        if typedefs:
            typedefs[cursor.spelling] = "struct " + cursor.spelling
        return cls(cursor.spelling, fields)


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
        for item in self.items:
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
        typedefs = {}
        tree = cls()
        for cursor in tu.cursor.get_children():
            try:
                if cursor.kind == clang.cindex.CursorKind.TYPEDEF_DECL:
                    typedefs[cursor.spelling] = cursor.underlying_typedef_type.spelling
                elif cursor.kind == clang.cindex.CursorKind.STRUCT_DECL:
                    tree.add(Struct.from_cursor(cursor, typedefs))
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
            item = self.get(name, ver)
            if not isinstance(item.ref, ItemRemoval):
                sub_tree.add(item)
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
                versions.add(item.ver)
        return sorted(list(versions))

    def to_dict(self):
        return {name: [item.to_dict() for item in revs] for name, revs in self.master_tree.items()}


if __name__ == "__main__":
    test = SyntaxTree.from_file("tests/regex/load_from_file/test.h")
    print(test.items["ComplexStruct3"])
    print(test.items["ComplexStruct3"].dependencies())
    print(test.items["NoStruct"].dependencies())
    print(test.items["NoStruct"])
    new_tree = test.get_fixed_tree()
    print(test.items.keys())
    print(new_tree.items.keys())
    print(test.get_order())

    dna_dir = 'blender/source/blender/makesdna'
    ast = SyntaxTree()
    for file in os.listdir(dna_dir):
        if not file.endswith((".hpp", ".h")):
            continue
        ast.merge(SyntaxTree.from_file(os.path.join(dna_dir, file)))
    print(json.dumps(test.to_dict()))

    ast2 = VersionedSyntaxTree()
    ast2.add_tree(test, 2)
    test_item = list(ast2.master_tree.keys())[0]
    ast3 = ast2.get_tree(2)
    ast3.items[test_item].ver = 3
    print(ast2.master_tree[test_item][0].ver)
    print(test_item)
    print(ast3.get_transitive_updates())

    test_var = Variable.from_str("void *(*e[4][4])[4][4];")
    test_var.reduce()
    print(test_var)
