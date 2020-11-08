import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from newton import Newton
from functions import PureParabol, Hyperbol

def test_Parabol():
    f = PureParabol()
    n = Newton(f)
    x0 = np.array([10.0])
    x = n.run(x0)

    npt.assert_almost_equal(x, np.asarray([0.0]), decimal=1)

def test_Hyperbol():
    f = Hyperbol()
    n = Newton(f)
    x0 = np.array([10.0])
    x = n.run(x0)

    npt.assert_almost_equal(x, np.asarray([0.0]), decimal=1)

def test_checkGradients():
    f = PureParabol()

    nt.assert_true(f.checkGradients(np.array([10.0])))

def test_checkHessian():
    f = PureParabol()

    nt.assert_true(f.checkHessian(np.array([10.0])))
