import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import Newton, NewtonFunction

SHOW_PLOTS = True

class Parabol(NewtonFunction):
    def value(self, x):
        return x*x

    def gradient(self, x):
        return 2*x

    def hessian(self, x):
        return 2.0

class Hyperbol(NewtonFunction):
    def value(self, x):
        return x*x*x

    def gradient(self, x):
        return 3*x*x

    def hessian(self, x):
        return 6*x

def test_Parabol():
    f = Parabol()
    n = Newton(f)
    x0 = np.array([10.0])
    x = n.run(x0)

    npt.assert_almost_equal(x, np.asarray([0.0]), 0.0001)

def test_Hyperbol():
    f = Hyperbol()
    n = Newton(f)
    x0 = np.array([10.0])
    x = n.run(x0)

    npt.assert_almost_equal(x, np.asarray([0.0]), 0.0001)

def test_checkGradients():
    f = Parabol()
    nt.assert_true(f.checkGradients(np.array([10.0])))

if __name__ == "__main__":
     test_checkGradients()