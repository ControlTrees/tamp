import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import ConstrainedProblem, SquarePenaltySolver, NewtonFunction, SquareCostFunction, AugmentedLagrangianSolver

class ProjX(NewtonFunction):
    def value(self, x):
        return x[0] #np.asarray([x[0]])

    def gradient(self, x):
        return np.asarray([1.0, 0.0]) #np.asarray([[1.0, 0.0]])

class SquareDistance(SquareCostFunction):
    def phi(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0] - 10
        dy = x[1] - 2
        return np.asarray([dx, dy])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0], [0.0, 1.0]])

def test_gradients_square_penalty():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    sq = SquarePenaltySolver(pb)
    unconstrained = sq.convert(sq.constrainedProblem, sq.mu)

    nt.assert_true(unconstrained.checkGradients(x0))

def test_constrained_squared_penalty():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    sq = SquarePenaltySolver(pb)
    x = sq.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), 0.0001)

def test_gradients_aula():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolver(pb)
    al.lambda_ = 1.0
    unconstrained = al.convert(al.constrainedProblem, al.mu, al.lambda_)

    nt.assert_true(unconstrained.checkGradients(x0))

def test_constrained_aula():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), 0.0001)

if __name__ == "__main__":
     test_constrained_squared_penalty()
     test_constrained_aula()