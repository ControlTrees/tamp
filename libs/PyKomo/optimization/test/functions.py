import sys
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from newton import NewtonFunction, VectorizedNewtonFunction
from gauss_newton import SquareCostFunction

class ProjX(NewtonFunction):
    def value(self, x):
        return x[0] #np.asarray([x[0]])

    def gradient(self, x):
        g = np.zeros(x.shape)
        g[0] = 1.0
        return g

    def hessian(self, x):
        return np.zeros((x.shape[0], x.shape[0]))

class ProjY(NewtonFunction):
    def value(self, x):
        return x[1] #np.asarray([x[0]])

    def gradient(self, x):
        return np.asarray([0.0, 1.0])

    def hessian(self, x):
        return np.zeros((2, 2))

class ProjXY(VectorizedNewtonFunction):
    def value(self, x):
        return x[:2]

    def gradient(self, x):
        gs = []
        for i in range(self.dim()):
            g = np.zeros(x.shape)
            g[i] = 1.0
            gs.append(g)
        return np.array(gs)

    def hessian(self, x):
        hs = []
        for i in range(self.dim()):
            h = np.zeros((x.shape[0], x.shape[0]))
            hs.append(h)
        return hs

    def dim(self):
        return 2

class SquareDistance(SquareCostFunction):
    def __init__(self, cx=10, cy=2):
        self.cx = cx
        self.cy = cy

    def phi(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0] - self.cx
        dy = x[1] - self.cy
        return np.asarray([dx, dy])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0], [0.0, 1.0]])


class Parabol(SquareCostFunction):
    def phi(self, x):
        return np.asarray([x[0]-10.0])

    def gradientPhi(self, x):
        return np.asarray([[1.0]])

class Parabol2D(SquareCostFunction):
    def phi(self, x):
    # one input two outputs
        f1 = x[0] -10
        f2 = x[0] - 5
        return np.asarray([f1, f2])

    def gradientPhi(self, x):
        return np.asarray([[1.0], [1.0]])


class PureParabol(NewtonFunction):
    def value(self, x):
        return np.dot(x.T, x)

    def gradient(self, x):
        return np.array([2*x[0]])

    def hessian(self, x):
        return np.array([[2.0]])

class Hyperbol(NewtonFunction):
    def value(self, x):
        return x*x*x

    def gradient(self, x):
        return 3*x*x

    def hessian(self, x):
        return 6*x

class SquareDistanceND(SquareCostFunction):
    def __init__(self, center):
        self.center = center

    def phi(self, x):
        return x - self.center

    def gradientPhi(self, _):
        return np.identity(self.center.shape[0])

class SquareDistance3D(SquareCostFunction):
    def __init__(self, cx=10, cy=2, cz=1, sx=1.0, sy=1.0, sz=1.0):
        self.cx = cx
        self.cy = cy
        self.cz = cz
        # scales
        self.sx = sx
        self.sy = sy
        self.sz = sz

    def phi(self, x):
        dx = self.sx* (x[0] - self.cx)
        dy = self.sy* (x[1] - self.cy)
        dz = self.sz* (x[2] - self.cz)
        return np.asarray([dx, dy, dz])

    def gradientPhi(self, x):
        return np.asarray([[self.sx, 0.0, 0.0], [0.0, self.sy, 0.0], [0.0, 0.0, self.sz]])

class SphereConstraint3D(SquareCostFunction):
    def __init__(self, cx, cy, cz, radius):
        self.sqdist = SquareDistance3D(cx, cy, cz)
        self.sqradius = radius * radius

    def value(self, x):
        return self.sqdist.value(x) - self.sqradius

    def gradient(self, x):
        return self.sqdist.gradient(x)

    def hessian(self, x):
        return self.sqdist.hessian(x)