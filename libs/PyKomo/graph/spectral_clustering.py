import numpy as np
import networkx as nx
from sklearn.cluster import SpectralClustering
from sklearn import metrics
np.random.seed(1)
import matplotlib.pyplot as plt

COLORS=['r', 'g', 'b', 'c', 'y', 'm']

def build_from_file(filepath):
    nzs = list()
    n = 0
    with open(filepath, "r") as file:
        line = file.readline()
        while line != '':  # The EOF char is an empty string
            #print(line)
            elements = line.split(" ")

            p = int(elements[1])
            q = int(elements[2])
            n = max(n, p, q)
            nzs.append((p, q))

            line = file.readline()

    #size = 200
    size = n+1
    A = np.zeros((size, size))
    for (i, j) in nzs:
        if i < size and j < size:
            A[i, j] = 1
            A[j, i] = 1

    G = nx.from_numpy_matrix(A)
    return G

def build_graph_1():
    G = nx.Graph()
    G.add_edge(1, 0)
    G.add_edge(3, 2)
    G.add_edge(4, 0)
    G.add_edge(4, 1)
    G.add_edge(5, 0)
    G.add_edge(5, 1)
    G.add_edge(5, 4)
    G.add_edge(6, 2)
    G.add_edge(6, 3)
    G.add_edge(6, 5)
    G.add_edge(7, 2)
    G.add_edge(7, 3)
    G.add_edge(7, 6)
    G.add_edge(8, 4)
    G.add_edge(8, 5)
    G.add_edge(9, 4)
    G.add_edge(9, 5)
    G.add_edge(9, 8)
    G.add_edge(10, 6)
    G.add_edge(10, 7)
    G.add_edge(11, 6)
    G.add_edge(11, 7)
    G.add_edge(11, 10)
    return G

def build_graph_2():
    G = nx.Graph()
    G.add_edge(0, 1)
    G.add_edge(1, 2)
    G.add_edge(2, 3)
    G.add_edge(1, 4)
    G.add_edge(4, 5)
    G.add_edge(1, 6)
    G.add_edge(6, 7)
    G.add_edge(7, 8)
    G.add_edge(8, 9)
    G.add_edge(9, 10)
    return G

#G = build_graph_1()
G = build_from_file('data/H_data_car_2w')
#G = build_from_file('data/H_data_2w')

# Number of connected components
nc = nx.number_connected_components(G)
components = sorted(nx.connected_components(G), key=len, reverse=True)
for comp in components:
    print(sorted(comp))
print('number of components:{}'.format(nc))
print('size of principal component: {}/{} = {}'.format(len(components[0]), len(G.nodes()), len(components[0])/ len(G.nodes())))

G = nx.subgraph(G, components[0])

# Get adjacency-matrix as numpy-array

A = nx.to_numpy_matrix(G)
nodes = list(G.nodes())
#print(A)


# Cluster
sc = SpectralClustering(2, affinity='precomputed', n_init=100)
sc.fit(A)
print('spectral clustering')
print(sc.labels_)

# Deduce sub-problems
xmasks=[]
for i, k in enumerate(sc.labels_):
    while len(xmasks) <= k:
        xmasks.append(set())
    xmasks[k].add(nodes[i])

# extend xmasks with connected nodes (ADMM overlap)
for k, xmask in enumerate(xmasks):
    to_add=set()
    for i in xmask:
        for j in G.neighbors(i):
            if j not in xmask:
                to_add.add(j)
    xmask.update(to_add)
    print("size of xmask[{}]={}, overlap:{}".format(k, len(xmask), len(to_add)))


# draw
if len(G.nodes()) < 200:
    colors = [COLORS[sc.labels_[i]] for i, n in enumerate(G.nodes())]
    labels = {n:n for n in G.nodes()}

    nx.draw(G, labels=labels, node_color=colors)
    plt.show()