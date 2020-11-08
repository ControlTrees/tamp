import numpy as np
from newton import NewtonFunction, Newton

class SquarePenaltySolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps_h = 0.001 #max constraint violation
        self.mu = 1.0
        self.rho = 10 # how much we increase the square penalty at each cycle (works also with one)

    @staticmethod
    def convert(pb, mu):
        class Augmented(NewtonFunction):
            def value(self, x):
                f = pb.f.value(x)
                b = pb.h.value(x) ** 2  # barrier
                return f + mu * b

            def gradient(self, x):
                jf = pb.f.gradient(x)
                h = pb.h.value(x)
                Jh = pb.h.gradient(x)
                jb = 2 * np.dot(Jh.T, h)
                return jf + mu * jb

            def hessian(self, x):
                Hf = pb.f.hessian(x)
                Jh = pb.h.gradient(x)
                Hb = 2 * np.dot(Jh.T, Jh)
                return Hf + mu * Hb

        return Augmented()

    def run(self, x):
        print("mu={}".format(self.mu))

        unconstrained = self.convert(self.constrainedProblem, self.mu)
        gn = Newton(unconstrained)
        x = gn.run(x)
        h = self.constrainedProblem.h.value(x)

        while np.abs(h) > self.eps_h:
            self.mu *= self.rho

            print("mu={}".format(self.mu))

            unconstrained = self.convert(self.constrainedProblem, self.mu)
            gn = Newton(unconstrained)
            x = gn.run(x)
            h = self.constrainedProblem.h.value(x)

        return x