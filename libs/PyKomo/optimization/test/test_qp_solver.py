import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem
from qp_solver import UnconstrainedQPSolver, ConstrainedQPSolver
from observers import *

def test_unconstrained_qp():
    # min x^2 - x
    qp = UnconstrainedQP(Q=np.array([[2.0]]), c=np.array([-1.0]))
    solver = UnconstrainedQPSolver(qp)
    x = solver.run()

    nt.assert_almost_equals(x, 0.5, delta=0.001)

def test_constrained_qp():
    # min x^2 - x
    # s.t. x <= 0
    qp = ConstrainedQP(Q=np.array([[2.0]]), c=np.array([-1.0]), A=np.array([[1.0]]), u=np.array([0.2]))
    solver = ConstrainedQPSolver(qp)
    x = solver.run(np.array([-1.0]))

    nt.assert_almost_equals(x, 0.2, delta=0.001)

def test_constrained_2d_qp():
    # s.t. x(0) >= 3
    #      x(1) >= 1
    x0 = np.array([10.0, 1.5])

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

    p = Plotter2D("2d qp", background=background)
    p.add_point(x0)

    qp = ConstrainedQP(Q=np.array([[1.0, 0.0], [0.0, 20.0]]), c=np.array([0.0, 0.0]), A=np.array([[-1.0, 0.0], [0.0, -1.0]]), u=np.array([-3.0, -1.0]))
    solver = ConstrainedQPSolver(qp)
    x = solver.run(x0, observer=p)

    p.report(plot=True)

    nt.assert_true(np.allclose(x, np.array([3.0, 1.0]), rtol=1e-03, atol=1e-03))