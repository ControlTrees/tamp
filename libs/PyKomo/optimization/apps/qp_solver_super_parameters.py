import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
sys.path.append(str(Path('../test').absolute()))

from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem
from qp_solver import UnconstrainedQPSolver, ConstrainedQPSolver
from observers import *
import pickle

def get_mus():
    mus = list()
    for i in range(10):
        mus.append(pow(10.0, i - 2))
    return mus

def make_sym(A):
    W = np.tril(A) + np.triu(A.T, 1)
    return W

def get_pbs(n):
    max_elem = 1
    d0 = 2

    pbs = list()

    for i in range(n):
        q = (np.random.rand(d0, d0) * max_elem).astype(np.float)
        Q = make_sym(q)

        c = ((np.random.rand(d0) - 0.5) * max_elem).astype(np.float)
        A = ((np.random.rand(d0, d0) - 0.5) * max_elem).astype(np.float)
        u = ((np.random.rand(d0) - 0.5) * max_elem).astype(np.float)
        x0 = ((np.random.rand(d0) - 0.5) * max_elem).astype(np.float)

        qp = ConstrainedQP(Q=Q, c=c, A=A, u=u)

        pbs.append((qp, x0))

    return pbs


def test_constrained_2d_qp():
    data = dict()
    mus = get_mus()

    def on_hessian_inversion(n):
        if n > 100:
            raise("Too many inversion")

    for i, (qp, x0) in enumerate(get_pbs(1000)):
        print("------{}------".format(i))

        newton_its = dict()

        for mu in mus:
            p = Plotter2D("2d qp", on_hessian_inversion=on_hessian_inversion)
            p.add_point(x0)
            solver = ConstrainedQPSolver(qp, mu=mu)

            try:
                x = solver.run(x0, observer=p)
                print(x)
            except:
                print("interrupt solving")

            newton_its.update({mu: p.n_hessian_inversion})

        data.update({pickle.dumps((qp, x0)) : newton_its})

    # write to file
    file = open('data.pkl', 'wb')
    pickle.dump(data, file)
    file.close()