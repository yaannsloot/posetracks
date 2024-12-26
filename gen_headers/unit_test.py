import unittest
import shutil
import os
from gen_headers import depgraph
from gen_headers import git
from gen_headers import regex
from gen_headers import db

# ----------------- DATABASE -------------------

class TestDB(unittest.TestCase):
    def setUp(self):
        os.makedirs('db_test')

    def tearDown(self):
        shutil.rmtree('db_test')

    

# ----------------- REGEX -------------------

class TestRegex(unittest.TestCase):
    def setUp(self):
        return super().setUp()
    
    def test_remove_comments(self):
        # Expected:
        # Double slash removes chars from start of sequence to newline or EOF
        # Slash asterisk comments should be removed from opening /* to ending */
        test_src1 = (
            "int a;\n"
            "//a comment\n"
            "char b;\n"
            "int wow; // int not_a_var;    \n"
        )
        expected_output1 = (
            "int a;\n\n"
            "char b;\n"
            "int wow; \n"
        )
        test_src2 = (
            "int a;\n"
            "//a comment\n"
            "/*  */int wow /* int not_a_var; */ = 2;    \n"
            "/* some  \n"
            "   text     \n"
            "    */  \n"
            "int c = 5; // aaa"
        )
        expected_output2 = (
            "int a;\n\n"
            "int wow  = 2;    \n  \n"
            "int c = 5; "
        )
        self.assertEqual(regex.remove_comments(test_src1), expected_output1)
        self.assertEqual(regex.remove_comments(test_src2), expected_output2)

    def test_remove_if_directives(self):
        pass

    def test_remove_macros(self):
        pass

    def test_extract_struct_definitions(self):
        pass

    def test_extract_enum_definitions(self):
        pass

    def test_src_structs_to_dict(self):
        pass

    def test_load_structs_from_file(self):
        pass

    def test_load_structs_from_dir(self):
        pass

# ----------------- GIT -------------------

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
        git.checkout_version((2,25,0))
        target_commit = next((tag for tag in git._repo.tags if str(tag) == "v2.25"), None)
        self.assertEqual(target_commit.commit, git._repo.head.commit)

    def test_list_headers(self):
        paths = git.list_headers('blender/source/blender/makesdna')
        self.assertTrue(all(path.endswith(('.h', '.hpp')) for path in paths))
        self.assertTrue(all(os.path.exists(path) for path in paths))

# ----------------- DEPGRAPH -------------------

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

if __name__ == '__main__':
    unittest.main(verbosity=2)