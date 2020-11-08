import nose.tools as nt
import sys
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from tree_builder import TreeBuilder

def build_simple_path_builder():
    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3, p=0.5)
    pb.add_edge(2, 4, p=0.5)
    return pb


def build_path_builder_with_cycle():
    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3)
    pb.add_edge(3, 4)
    pb.add_edge(4, 0)
    return pb

def test_class_creation():
    pb = TreeBuilder()

def test_add_edge():
    pb = TreeBuilder()
    pb.add_edge(0, 1)

    nt.assert_equals(pb.n_nodes(), 2)
    nt.assert_equals(pb.p(0,1), 1)

    pb.add_edge(1, 2)

    nt.assert_equals(pb.n_nodes(), 3)
    nt.assert_equals(pb.p(0, 1), 1)
    nt.assert_equals(pb.p(1, 2), 1)

    pb.add_edge(2, 3, p=0.5)

    nt.assert_equals(pb.n_nodes(), 4)
    nt.assert_equals(pb.p(2, 3), 0.5)

    pb.add_edge(2, 4, p=0.5)

    nt.assert_equals(pb.n_nodes(), 5)
    nt.assert_equals(pb.p(2, 4), 0.5)

def test_get_leafs():
    pb = build_simple_path_builder()
    nt.assert_equals([3, 4], pb.get_leafs())

def test_get_parents():
    pb = build_simple_path_builder()
    nt.assert_equals([], pb.get_parents(0))

    nt.assert_equals([0], pb.get_parents(1))

    nt.assert_equals([1], pb.get_parents(2))

    nt.assert_equals([2], pb.get_parents(3))

    nt.assert_equals([2], pb.get_parents(4))

def test_get_paths():
    pb = build_simple_path_builder()
    nt.assert_equals(2, len(pb.get_paths()))
    nt.assert_equals([(0, 1.0), (1, 1.0), (2, 1.0), (3, 0.5)], pb.get_paths()[0])
    nt.assert_equals([(0, 1.0), (1, 1.0), (2, 1.0), (4, 0.5)], pb.get_paths()[1])

def test_get_paths_with_cycles():
    pb = build_path_builder_with_cycle()
    nt.assert_equals(1, len(pb.get_cycles()))
