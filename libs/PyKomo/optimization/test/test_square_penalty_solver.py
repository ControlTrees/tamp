import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import ConstrainedProblem
from square_penalty_solver import SquarePenaltySolver
from functions import SquareDistance, ProjX

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

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), decimal=0)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

if __name__ == "__main__":
     test_constrained_squared_penalty()