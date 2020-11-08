import nose.tools as nt
import sys
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
import numpy as np
import scipy as sc
from scipy.linalg import solve_banded

import time

def build_banded_matrix_problem(k, n):
    m = n - k

    a = np.random.rand(n,n)
    for i in range(m):
        for j in range(n-m+i, n):
            a[i][j] = 0

    for i in range(n-m, n):
        for j in range(i-(n-m)+1):
            a[i][j] = 0

    #print(a)

    x = np.random.rand(n, 1)
    b = a.dot(x)

    ab = np.zeros((k+1, n))
    u = k-1
    l = k - u
    for i in range(n):
        for j in range(n):
            try:
                ab[u + i - j][j] = a[i][j]
            except:
                pass
    #print(x)
    #print(ab)

    return a, ab, b, u, l

def test_foo():
    n_actions = 15
    n_steps = 10
    n_deg_freedom = 3
    n = n_actions * 10 * n_deg_freedom
    k = (1 + 2) * n_deg_freedom
    a, ab, b, u, l = build_banded_matrix_problem(k, n)

    start = time.clock()
    for i in range(100):
        x1 = np.linalg.solve(a, b)
    print("normal:{}".format(time.clock() - start))

    start = time.clock()
    for i in range(100):
        x2 = sc.linalg.solve_banded((l, u), ab, b)
    print("banded_solving:{}".format(time.clock() - start))

