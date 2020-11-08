import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from functions import *

def test_SphereConstraint3D():
    h = SphereConstraint3D(cx=0, cy=0.5, cz=0.5, radius=0.5)

    on_sphere = [
        np.array([0, 0.5, 0.0]),
        np.array([0, 0.5, 1.0]),
        np.array([0.5, 0.5, 0.5])
    ]

    for p in on_sphere:
        d2 = h.value(p) # on sphere
        nt.assert_almost_equals(d2, 0.0)
        nt.assert_true(h.checkGradients(p))

    d2 = h.value(np.array([0, 0.5, 0.5])) # sphere center
    nt.assert_almost_equals(d2, -0.25)