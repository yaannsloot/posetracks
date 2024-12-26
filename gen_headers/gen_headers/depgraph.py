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

Graph functions used for building and processing a C struct dependency tree
"""

# MODIFY THIS TO WORK WITH CONVERSIONS TO VOID PTRS

class Node:
    def __init__(self, tag, ver):
        self.tag = tag
        self.ver = ver
    
    def __eq__(self, value):
        if isinstance(value, Node):
            return self.tag == value.tag
        return False
    
    def __str__(self):
        return self.tag
    
    def __hash__(self):
        return hash(self.tag)

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}

    def add_node(self, name, ver):
        self.nodes[name] = Node(name, ver)

    def add_edge(self, parent, child):
        parent = str(parent)
        child = str(child)
        child_set = self.edges.setdefault(parent, set())
        child_set.add(child)

    def find_dangling(self):
        lost_parents = set(self.edges.keys()) - set(self.nodes.keys())
        output = set()
        for ref in lost_parents:
            output = output.union(self.edges[ref])
        return output.union(lost_parents), lost_parents

    def remove_dangling(self):
        output = Graph()
        output.nodes = self.nodes
        output.edges = self.edges
        while len(output.nodes) > 0:
            bad_nodes, bad_edges = output.find_dangling()
            if len(bad_edges) == 0 and len(bad_nodes) == 0:
                break
            for node in bad_nodes:
                output.nodes.pop(node, None)
            for edge in bad_edges:
                output.edges.pop(edge, None)
            empty_edges = set()
            for parent, children in output.edges.items():
                lost_children = children - set(output.nodes.keys())
                for child in lost_children:
                    children.discard(child)
                if not children:
                    empty_edges.add(parent)
            for parent in empty_edges:
                output.edges.pop(parent, None)
        return output
    
    def get_nodes_in_ver(self, ver):
        output = set()
        def visit_adj(node):
            if node in output or node is None or node.ver > ver:
                return
            output.add(node)
            for node in self.edges.get(str(node), []):
                visit_adj(self.nodes.get(node, None))
        for node in self.nodes.values():
            if node.ver == ver:
                visit_adj(node)
        return output
    
    def get_version_subgraph(self, ver):
        output = Graph()
        output.nodes = {node.tag: node for node in self.get_nodes_in_ver(ver)}
        tags = set(output.nodes.keys())
        output.edges = {parent: children & tags for parent, children in self.edges.items() if parent in tags}
        return output

    def get_loopless_edges(self):
        output = {}
        for node, children in self.edges.items():
            no_loop_children = children.copy()
            no_loop_children.discard(node)
            if no_loop_children:
                output[node] = no_loop_children
        return output

    def sort(self):
        node_degrees = {str(node): 0 for node in self.nodes}
        no_loop_edges = self.get_loopless_edges()
        for children in no_loop_edges.values():
            for child in children:
                node_degrees[child] += 1
        order = []
        last_len = len(node_degrees)
        while len(node_degrees) > 0:
            for node, degree in node_degrees.copy().items():
                if degree <= 0:
                    children = no_loop_edges.get(node, set())
                    for child in children:
                        node_degrees[child] -= 1
                    node_degrees.pop(node, None)
                    order.append(node)
            if len(node_degrees) == last_len:
                raise RuntimeError('Cycle detected while sorting graph')
            last_len = len(node_degrees)
        return order

def graph_from_dep_query(deps, current_ver):
    g = Graph()
    all_tags = set()
    for row in deps:
        tag = row['tag']
        ver = (row['major'], row['minor'], row['patch'])
        if current_ver == ver:
            all_tags.add(tag)
        if row['rev_available']:
            g.add_node(tag, ver)
        if row['dep_available']:
            g.add_node(tag, ver)
        if row['dep_available'] is not None and not row['dep_is_ptr']: 
            g.add_edge(row['dep_tag'], tag)
    return g, all_tags