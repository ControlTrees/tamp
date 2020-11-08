import numpy as np
from newton import NewtonFunction

class SquareCostFunction(NewtonFunction):
    def value(self, x):
        phi = self.phi(x)
        return np.dot(phi.T, phi)

    def gradient(self, x):
        phi = self.phi(x)
        Jphi = self.gradientPhi(x)
        return 2 * np.dot(Jphi.T, phi)

    def hessian(self, x): #pseudo hessian (neglects 2 phi.T * hessian(phi)
        Jphi = self.gradientPhi(x)
        return 2 * np.dot(Jphi.T, Jphi)

    def phi(self, x):
        pass

    def gradientPhi(self, x):
        pass

