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

Unit tests for gen header tool
NOT COMPLETE!
"""

import unittest
import shutil
import os
import uuid
import json
from contextlib import contextmanager
from gen_headers import depgraph
from gen_headers import git
from gen_headers import regex
from gen_headers import db
from gen_headers import types
from gen_headers import output


@contextmanager
def testdb_template():
    instances = {os.path.splitext(file)[0] for file in os.listdir(
        'db_test') if os.path.isfile(os.path.join('db_test', file) and file.endswith('.db'))}
    while True:
        new_instance = str(uuid.uuid4())
        if new_instance not in instances:
            break
    template_path = 'db_test/template.db'
    dest_path = f'db_test/{new_instance}.db'
    try:
        db._close_db()  # always close db before switching
        shutil.copy2(template_path, dest_path)
        db._init_db(template_path)
        yield
    finally:
        db._close_db()


class TestDB(unittest.TestCase):
    def setUp(self):
        os.makedirs('db_test', exist_ok=True)
        db._init_db('db_test/template.db')
        test_vers = [
            (0, 0, 1),
            (0, 0, 2),
            (0, 1, 1),
            (1, 0, 4),
            (2, 0, 2),
            (3, 0, 0),
        ]
        ver_fields = [db.BlenderVersion.major,
                      db.BlenderVersion.minor,
                      db.BlenderVersion.patch]
        db.BlenderVersion.insert_many(
            test_vers, fields=ver_fields).on_conflict_ignore().execute()
        test_structs = [
            "StructA",
            "StructB",
            "StructC",
            "StructD",
            "StructE",
            "StructF",
            "StructG",
            "StructH",
            "StructI",
            "StructJ",
        ]
        db.Struct.insert_many(test_structs, fields=[
                              db.Struct.tag]).on_conflict_ignore().execute()

        db._close_db()

    def tearDown(self):
        db._close_db()
        shutil.rmtree('db_test')

    def test_crc32(self):
        cases = [
            # Test cases mentioned in https://stackoverflow.com/a/39805598
            # Trivial one.
            (int("0x00000000", 0), ""),
            # Source: https://rosettacode.org/wiki/CRC-32
            (int("0x414FA339", 0), "The quick brown fox jumps over the lazy dog"),
            # Source: http://cryptomanager.com/tv.html
            (int("0x9BD366AE", 0), "various CRC algorithms input data"),
            # Source: http://www.febooti.com/products/filetweak/members/hash-and-crc/test-vectors/
            (int("0x0C877F61", 0), "Test vector from febooti.com"),
        ]
        self.assertTrue(all([case[0] == db.crc32(case[1]) for case in cases]))

    def test_resolve_dependencies(self):
        with testdb_template():
            pass

    def test_get_ver_ref(self):
        with testdb_template():
            pass

    def test_add_dependency(self):
        with testdb_template():
            pass

    def test_add_revision(self):
        with testdb_template():
            pass

    def test_single_val_update_atomic(self):
        with testdb_template():
            pass

    def test_tag_revisions_with_ver_atomic(self):
        with testdb_template():
            pass

    def test_load_revisions_atomic(self):
        with testdb_template():
            pass

    def test_get_version_mappings(self):
        with testdb_template():
            pass

    def test_get_active_versions(self):
        with testdb_template():
            pass


def test_equal_str(case, base_path, unit):
    for file in os.listdir(base_path):
        if not (file.startswith('in') and file.endswith('.txt')):
            continue
        src = os.path.join(base_path, file)
        target = os.path.join(base_path, file.replace('in', 'out'))
        with open(src, 'r') as f:
            src = f.read()
        with open(target, 'r') as f:
            target = f.read()
        case.assertEqual(unit(src), target)


class TestRegex(unittest.TestCase):
    def setUp(self):
        return super().setUp()

    def test_remove_comments(self):
        # Expected:
        # Double slash removes chars from start of sequence to newline or EOF
        # Slash asterisk comments should be removed from opening /* to ending */
        test_equal_str(self, "tests/regex/remove_comments",
                       regex.remove_comments)

    def test_remove_if_directives(self):
        # Expected:
        # All chars from #if to #endif inclusive are to be removed.
        # Trailing whitespace including newlines are not to be removed.
        test_equal_str(self, "tests/regex/remove_if_directives",
                       regex.remove_if_directives)

    def test_remove_macros(self):
        # Expected:
        # All macro directives (indicated with #define) are to be removed
        # up to the first terminating newline. A backtick indicates a deferred
        # terminaton of the macro definition, causing the following newline to
        # be ignored. Terminating newlines are not to be removed.
        test_equal_str(self, "tests/regex/remove_macros", regex.remove_macros)

    def test_remove_include_guard(self):
        test_equal_str(self, "tests/regex/remove_include_guard",
                       regex.remove_include_guard)

    def test_extract_enum_definitions(self):  # not used currently
        pass

    def test_load_structs_from_file(self):
        # Also tests extract_struct_definitions under the hood
        # Expected:
        # - Extracts all struct definitions (those starting with `typedef struct ...`)
        #   into a dictionary formatted as: {struct_tag: [normalized lines...]}.
        # - Normalized lines refer to the variables and terms encapsulated within
        #   the struct definition, with the following adjustments:
        #   - Remove all leading and trailing whitespace.
        #   - Reduce spaces between operators and other terms to a single occurrence.
        # - If a nested struct is present, represent it using the same dictionary format
        #   instead of a string.
        # - Nested dictionaries should contain only a single key, unlike the root dictionary.
        # - All pointer asterisks should be on the left side next to the type name.
        base_path = "tests/regex/load_from_file"
        for file in os.listdir(base_path):
            if not file.endswith(('.h', '.hpp')):
                continue
            results = os.path.join(
                base_path, os.path.splitext(file)[0] + '.json')
            with open(results, 'r') as f:
                results = json.load(f)
            file = os.path.join(base_path, file)
            self.assertEqual(regex.load_structs_from_file(file), results)


class TestGit(unittest.TestCase):
    def setUp(self):
        git.init_repo()

    def test_init_repo(self):
        self.assertTrue(os.path.exists('blender'))
        self.assertTrue(os.path.exists('blender/.git'))

    def test_get_versions(self):
        vers = git.available_versions()
        self.assertSequenceEqual(vers, sorted(vers))
        self.assertEqual(len(vers[0]), 3)
        self.assertSequenceEqual({v[0] for v in vers}, {2, 3, 4})

    def test_checkout_version(self):
        git.checkout_version((2, 25, 0))
        target_commit = next(
            (tag for tag in git._repo.tags if str(tag) == "v2.25"), None)
        self.assertEqual(target_commit.commit, git._repo.head.commit)

    def test_list_headers(self):
        paths = git.list_headers('blender/source/blender/makesdna')
        self.assertTrue(all(path.endswith(('.h', '.hpp')) for path in paths))
        self.assertTrue(all(os.path.exists(path) for path in paths))


class TestGraph(unittest.TestCase):
    def setUp(self):
        self.graph = depgraph.Graph()
        self.graph.add_node('A', 1)
        self.graph.add_node('B', 2)
        self.graph.add_node('C', 1)
        self.graph.add_node('D', 3)
        self.graph.add_edge('A', 'B')
        self.graph.add_edge('B', 'C')
        self.graph.add_edge('C', 'D')

    def test_add_node_and_edge(self):
        self.assertIn('A', self.graph.nodes)
        self.assertEqual(self.graph.nodes['A'].ver, 1)
        self.assertIn('B', self.graph.edges['A'])
        self.assertIn('C', self.graph.edges['B'])

    def test_find_dangling(self):
        self.graph.add_edge('X', 'A')
        dangling, lost_parents = self.graph.find_dangling()
        self.assertIn('X', dangling)
        self.assertIn('X', lost_parents)

    def test_remove_dangling(self):
        self.graph.add_edge('X', 'A')
        clean_graph = self.graph.remove_dangling()
        self.assertNotIn('X', clean_graph.edges)
        self.assertNotIn('X', clean_graph.nodes)

    def test_topological_sort(self):
        order = self.graph.sort()
        self.assertEqual(order, ['A', 'B', 'C', 'D'])

    def test_cycle_detection(self):
        self.graph.add_edge('D', 'A')
        with self.assertRaises(RuntimeError):
            self.graph.sort()

    def test_get_nodes_in_ver(self):
        reachable_nodes = self.graph.get_nodes_in_ver(1)
        reachable_tags = {str(node) for node in reachable_nodes}
        self.assertIn('A', reachable_tags)
        self.assertIn('C', reachable_tags)
        self.assertNotIn('B', reachable_tags)

    def test_get_loopless_edges(self):
        self.graph.add_edge('A', 'A')
        loopless_edges = self.graph.get_loopless_edges()
        self.assertNotIn('A', loopless_edges['A'])


class TestOutput(unittest.TestCase):
    def setUp(self):
        return super().setUp()

    def test_ver_string(self):
        test_cases = {
            (0, 0, 1): "0_0_1",
            (0, 0, 2): "0_0_2",
            (0, 1, 1): "0_1_1",
            (1, 0, 4): "1_0_4",
            (2, 0, 2): "2_0_2",
            (3, 0, 0): "3_0_0",
        }
        self.assertTrue(all(output.ver_string(
            k) == v for k, v in test_cases.items()))

    def test_ver_name(self):
        test_cases = {
            ("a_name", (0, 0, 1)): "a_name0_0_1",
            ("THE_NAME", (0, 0, 2)): "THE_NAME0_0_2",
            ("NaME", (0, 1, 1)): "NaME0_1_1",
            ("wow", (1, 0, 4)): "wow1_0_4",
            ("OK", (2, 0, 2)): "OK2_0_2",
            ("333", (3, 0, 0)): "3333_0_0",
        }
        self.assertTrue(all(output.ver_name(
            *k) == v for k, v in test_cases.items()))

    def test_get_declarations(self):
        test_input = [
            {"tag": "a_name", "major": 0, "minor": 0, "patch": 1},
            {"tag": "THE_NAME", "major": 0, "minor": 0, "patch": 2},
            {"tag": "NaME", "major": 0, "minor": 1, "patch": 1},
            {"tag": "wow", "major": 1, "minor": 0, "patch": 4},
            {"tag": "OK", "major": 2, "minor": 0, "patch": 2},
            {"tag": "333", "major": 3, "minor": 0, "patch": 0}
        ]
        expected_output = "\n".join(f"struct {name};" for name in [
            "3333_0_0",
            "a_name0_0_1",
            "NaME0_1_1",
            "OK2_0_2",
            "THE_NAME0_0_2",
            "wow1_0_4",
        ])
        self.assertEqual(output.get_declarations(test_input), expected_output)

    def test_prune_deps(self):
        pass

    def test_get_includes(self):
        pass

    def test_get_ver_enums(self):
        pass

    def test_get_pybind_ver_enum_macro(self):
        pass

    def test_add_include_guard(self):
        pass

    def test_generate_header(self):
        pass

    def test_get_version_macro(self):
        pass

    def test_generate_macros(self):
        pass

    def test_generate_types_hpp(self):
        pass


if __name__ == '__main__':
    unittest.main(verbosity=2)
