import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import ADMMProblem_Newton, ADMMProblem, ConstrainedProblem
from admm_solver import ADMMSolver_Newton, ADMMSolver
from augmented_lagrangian_solver import AugmentedLagrangianSolver
from functions import *
from observers import *

def test_distance_3d():
    f = SquareDistance3D(1, 1, 1)
    v = f.value(np.array([0, 0, 0]))
    nt.assert_almost_equals(v, 3, 0.0001)

def test_constrained_dist_3d_no_decomp():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("aula (h:x=0)")
    p.add_point(x0)

    pb = ConstrainedProblem(f=SquareDistance3D(1, 1), h=ProjX())
    solver = AugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)
    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

    p.report()

def test_distance_3d_decomp():
    f0 = SquareDistance3DDecomp0(1, 1)
    f1 = SquareDistance3DDecomp1(1, 1)
    v0 = f0.value(np.array([0, 0, 0]))
    v1 = f1.value(np.array([0, 0, 0]))

    nt.assert_true(f0.checkGradients(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f1.checkGradients(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f0.checkHessian(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f1.checkHessian(np.array([0.0, 0.0, 0.0])))

    nt.assert_almost_equals(v0 + v1, 3, 0.0001)

def test_dist_3d_minimization():
    f0 = SquareDistance3DDecomp0(1, 1)
    f1 = SquareDistance3DDecomp1(1, 1)
    pb = ADMMProblem_Newton(f0=f0, f1=f1)
    solver = ADMMSolver_Newton(pb)
    x = solver.run(np.array([0.0, 0.0, 0.0]))
    npt.assert_almost_equal(x, np.array([1.0, 1.0, 1.0]), decimal=1)

def test_constrained_dist_3d():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("admm (h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=ProjX())
    pb1 = ConstrainedProblem(f= SquareDistance3DDecomp1(1, 1), h=ProjX())
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = ADMMSolver(pb)
    x = solver.run(x0, observer=p)
    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

    p.report()

def test_constrained_dist_3d_sphere():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("admm (h:x=0)")
    p.add_point(x0)

    h = SphereConstraint3D(cx=0, cy=0.5, cz=0.5, radius=0.5)
    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=h)
    pb1 = ConstrainedProblem(f= SquareDistance3DDecomp1(1, 1), h=h)
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = ADMMSolver(pb)
    x = solver.run(x0, observer=p)

    nt.assert_almost_equals(h.value(x), 0, delta=0.001)

    p.report()