import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import Newton, SquareCostFunction

class Parabol(SquareCostFunction):
    def phi(self, x):
        return np.asarray([x[0]-10.0])

    def gradientPhi(self, x):
        return np.asarray([[1.0]])

def test_Parabol():
    # minimize (x - 10)^2
    f = Parabol()
    gn = Newton(f)
    x = gn.run(np.asarray([1.0]))
    npt.assert_almost_equal(x, np.asarray([10.0]), 0.0001)

class SquareDistance(SquareCostFunction):
    def phi(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0] - 10
        dy = x[1] - 5
        return np.asarray([dx, dy])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0], [0.0, 1.0]])

def test_SquareDistance():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    f = SquareDistance()
    gn = Newton(f)
    x0 = np.array([1.0, 1.0])
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([10.0, 5.0]), 0.0001)

class Parabol2D(SquareCostFunction):
    def phi(self, x):
    # one input two outputs
        f1 = x[0] -10
        f2 = x[0] - 5
        return np.asarray([f1, f2])

    def gradientPhi(self, x):
        return np.asarray([[1.0], [1.0]])

def test_Parabol2D():
    f = Parabol2D()
    gn = Newton(f)
    x0 = np.array([1.0])
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([7.5]), 0.0001)

if __name__ == "__main__":
     test_Parabol2D()