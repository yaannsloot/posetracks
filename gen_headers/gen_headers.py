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

Generates headers that can be used to access internal memory structures
from blender.
"""

from gen_headers import git
from gen_headers import regex
from gen_headers import types
from gen_headers import depgraph
from gen_headers import db
from gen_headers import output
import os
from peewee import *


def main():
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
        graph, _ = depgraph.graph_from_dep_query(deps, v)
        if not db_ver.evaluated:
            print("Evaluating dependencies...", v)
            transitive_updates = [(str(node), node.ver[0], node.ver[1], node.ver[2])
                                  for node in graph.get_nodes_in_ver(v) if node.ver != v]
            targets = (db.Revision.select()
                       .join(db.Struct, on=(db.Revision.struct_id == db.Struct.struct_id))
                       .join(db.RevisionVersion, on=(db.Revision.rev_id == db.RevisionVersion.rev_id))
                       .join(db.BlenderVersion, on=(db.RevisionVersion.ver_id == db.BlenderVersion.ver_id))
                       .where(Tuple(db.Struct.tag, db.BlenderVersion.major, db.BlenderVersion.minor, db.BlenderVersion.patch).in_(transitive_updates)))
            db.tag_revisions_with_ver_atomic(targets, v)
            graph, all_tags = depgraph.graph_from_dep_query(deps, v)
            graph.remove_dangling()
            unavailable = all_tags - {tag for tag in graph.nodes.keys()}
            targets = (db.RevisionVersion.select()
                       .join(db.Revision, on=(db.RevisionVersion.rev_id == db.Revision.rev_id))
                       .join(db.Struct, on=(db.Revision.struct_id == db.Struct.struct_id))
                       .where((db.RevisionVersion.ver_id == db_ver) & (db.Struct.tag << unavailable)))
            db.single_val_update_atomic(
                db.RevisionVersion, {"id": targets}, {"available": 0})
            db_ver.evaluated = True
            db_ver.save()
            deps = db.resolve_dependencies(v)
        print("Writing header file...", v)
        header_out, fname = output.write_header(
            "makesdna_types_", v, graph, deps)
        if not header_out:
            continue
        os.makedirs("output", exist_ok=True)
        with open(os.path.join("output", fname), 'w') as f:
            f.write(header_out)
    macros_out = output.write_macros(
        db.get_version_mappings(), "makesdna_mappings.h")
    with open(os.path.join("output", "makesdna_mappings.h"), 'w') as f:
        f.write(macros_out)
    with open(os.path.join("output", "blender_makesdna.hpp"), 'w') as f:
        f.write(output.write_types_hpp("blender_makesdna.hpp",
                db.get_active_versions(), "makesdna_types_", "makesdna_mappings.h"))


if __name__ == "__main__":
    main()
