import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from augmented_lagrangian_solver import AugmentedLagrangianSolver, Lagrangian
from optimization_problems import ConstrainedProblem
from functions import SquareDistance, ProjX, ProjY, ProjXY
from observers import *

def test_gradients_lagrangian_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    lagrangian = Lagrangian(pb, mu=1.0, lambda_h=1.0)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_constrained_aula_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

def test_constrained_aula_eq_2_constraints():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjXY(), g=ProjXY())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 0.0]), decimal=1)

def test_constrained_aula_eq_no_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, 2.0]), decimal=1)

def test_gradients_aula_ineq_active_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    lagrangian = Lagrangian(pb, mu=1.0, lambda_g=1.0)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_constrained_aula_ineq_active_constraint():
    p = Plotter2D("constrained_aula_ineq_active_constraint")

    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, 0.0]), decimal=1)
    nt.assert_almost_equals(x[1], 0, delta=0.001)

    #p.plot()

def test_gradients_aula_ineq_inactive_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0), g=ProjY())
    lagrangian = Lagrangian(pb, mu=1.0, lambda_g=1.0)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_gradients_aula_ineq_no_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0))
    lagrangian = Lagrangian(pb, mu=1.0, lambda_g=1.0)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_constrained_aula_ineq_inactive_constraint():
    x0 = np.array([1.0, -1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, -1.0]), decimal=1)
    nt.assert_almost_equals(x[1], -1.0, delta=0.001)
