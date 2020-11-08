import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from newton import Newton
from functions import Parabol2D, Parabol, SquareDistance

def test_Parabol():
    # minimize (x - 10)^2
    f = Parabol()
    gn = Newton(f)
    x = gn.run(np.asarray([1.0]))
    npt.assert_almost_equal(x, np.asarray([10.0]), decimal=1)

def test_SquareDistance():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    f = SquareDistance()
    gn = Newton(f)
    x0 = np.array([1.0, 1.0])
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([10.0, 2.0]), decimal=1)

def test_Parabol2D():
    f = Parabol2D()
    gn = Newton(f)
    x0 = np.array([1.0])
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([7.5]), decimal=1)