import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem
from qp_solver import UnconstrainedQPSolver, ConstrainedQPSolver, F, G
from optimization_problems import ADMMProblem_Newton, ADMMProblemN, ADMMProblem, ConstrainedProblem
from decentralized_aula import DecentralizedAugmentedLagrangianSolver, DecentralizedAugmentedLagrangianSolverN
from qp_decomposition import *
from observers import *

def test_constrained_2d_qp():
    # s.t. x + y >= 3
    def background(ax):
        ax.set_aspect(aspect='equal')
        ax.set_xlim(-1, 10.0)
        ax.set_xticks(np.arange(-1, 10, step=1))
        ax.set_ylim(-1, 2.0)
        ax.set_yticks(np.arange(-1, 2, step=1))
        ax.grid(True)

        delta = 0.1
        x = np.arange(-1.0, 10.0, delta)
        y = np.arange(-4.0, 2.0, delta)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros((y.shape[0], x.shape[0]))

        for i, xx in enumerate(x):
            for j, yy in enumerate(y):
                xxx = np.array([xx, yy])
                Z[j,i] = (0.5 * np.dot(xxx.T, np.dot(qp.Q, xxx)) + np.dot(qp.c.T, xxx))

        CS = ax.contour(X, Y, Z, [1.0, 2.0, 3.0])
        ax.clabel(CS, inline=1, fontsize=10)

    x0 = np.array([10.0, 1.5])

    p = Plotter2DSimple("2d decomposed", background=background)

    qp = ConstrainedQP(Q=np.array([[1.0, 0.0], [0.0, 1.0]]), c=np.array([-1.0, 0.5]), A=np.array([[-1.0, -1.0]]), u=np.array([-3.0])) # 1 constraints over 2 variables

    # decompose
    qps = decompose(qp)
    nt.assert_equals(len(qps), 3)

    # extract pbs
    pb1 = ConstrainedProblem(f=F(qps[0]), g=G(qps[0]) if qps[0].A is not None else None)
    pb2 = ConstrainedProblem(f=F(qps[1]), g=G(qps[1]) if qps[1].A is not None else None)
    pb3 = ConstrainedProblem(f=F(qps[2]), g=G(qps[2]) if qps[2].A is not None else None)

    # give to solver
    pb = ADMMProblemN(pbs=[pb1, pb2, pb3])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    p.report(plot=True)

    print("***")
    print("x={}".format(x)) # x=[2.24957629 0.74957629]
    nt.assert_true(np.allclose(x, np.array([2.24957629, 0.74957629]), rtol=1e-03, atol=1e-03))


def test_constrained_3d_qp():
    # s.t. x + y >= 3
    def background(ax):
        pass

    x0 = np.array([10.0, 1.5, 2.0])

    p = Plotter2DSimple("3d decomposed", background=background)

    qp = ConstrainedQP(Q=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), c=np.array([-1.0, 0.5, 1.0]), A=np.array([[-1.0, -1.0, -1.0]]), u=np.array([-3.0])) # 1 constraints over 2 variables

    # decompose
    qps = decompose(qp)
    nt.assert_equals(len(qps), 3)

    # extract pbs
    pb1 = ConstrainedProblem(f=F(qps[0]), g=G(qps[0]) if qps[0].A is not None else None)
    pb2 = ConstrainedProblem(f=F(qps[1]), g=G(qps[1]) if qps[1].A is not None else None)
    pb3 = ConstrainedProblem(f=F(qps[2]), g=G(qps[2]) if qps[2].A is not None else None)

    # give to solver
    pb = ADMMProblemN(pbs=[pb1, pb2, pb3])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    p.report(plot=False)

    print("***")
    print("x={}".format(x)) # x=[2.24957629 0.74957629]
    #nt.assert_true(np.allclose(x, np.array([2.24957629, 0.74957629]), rtol=1e-03, atol=1e-03))