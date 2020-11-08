import numpy as np

class TreeBuilder:
    def __init__(self):
        self.adjacency_matrix = np.empty(shape=(0, 0))

    def add_edge(self, _from, _to, p=1.0):
        max_node = max(_from, _to)
        size = max_node+1
        if self.adjacency_matrix.shape[0] < size:
            old_adjacency_matrix = np.copy(self.adjacency_matrix)
            old_size = old_adjacency_matrix.shape[0]
            adjacency_matrix = np.zeros((size, size))
            adjacency_matrix[:old_size, :old_size] = old_adjacency_matrix
            self.adjacency_matrix = adjacency_matrix
        self.adjacency_matrix[_from, _to]=p

    def n_nodes(self):
        return self.adjacency_matrix.shape[0]

    def p(self, _from, _to):
        return self.adjacency_matrix[_from, _to]

    def get_leafs(self):
        leafs=[]
        all_zeros = lambda a : not np.any(a)
        for i, row in enumerate(self.adjacency_matrix):
            if all_zeros(row):
                leafs.append(i)
        return leafs

    def get_parents(self, node):
        col = self.adjacency_matrix[:,node]
        parents = [j for j in range(0, self.n_nodes()) if col[j]!=0]
        return parents

    def get_paths(self):
        leafs = self.get_leafs()
        paths=[]
        for l in leafs:
            paths.append(self.get_path(l))
        return paths

    def get_path(self, l):
        path = []
        current = l
        parents = self.get_parents(current)
        while len(parents) == 1:
            parent = parents[0]
            p = self.p(parent, current)
            path.append((current, p))
            current = parent
            parents = self.get_parents(current)
        path.append((0, 1.0))
        path.reverse()
        return path

    def get_cycles(self):
        return [1]