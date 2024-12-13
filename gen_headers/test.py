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

from gen_headers import git
from gen_headers import regex
from gen_headers import types
from gen_headers import depgraph
from gen_headers import db
import os
import pprint


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



test3()
