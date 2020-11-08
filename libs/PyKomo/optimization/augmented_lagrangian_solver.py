import numpy as np
from newton import NewtonFunction, Newton

def eval(f, x):
    v = f.value(x)
    J = f.gradient(x)
    H = f.hessian(x)

    if len(J.shape) == 1:
        assert len(H.shape) == 2
        v = np.array([v])
        J = np.array([J])
        H = np.array([H])

    return v, J, H

class Lagrangian(NewtonFunction):
    def __init__(self, pb, mu=1.0, lambda_h=0.0, lambda_g=0.0):
        self.pb = pb
        self.mu = mu
        self.lambda_h = np.ones(pb.h.dim()) * lambda_h if pb.h else None
        self.lambda_g = np.ones(pb.g.dim()) * lambda_g if pb.g else None

    def value(self, x):
        f = self.pb.f.value(x)

        if self.pb.h:
            h = self.pb.h.value(x)
            f += self.mu * np.dot(h.T, h)     # square penalty
            f += np.dot(self.lambda_h.T, h)   # lagrange

        if self.pb.g:
            g = self.pb.g.value(x)
            g = np.atleast_1d(g)

            for i in range(g.shape[0]):
                activity = g[i] >= 0 or self.lambda_g[i] > 0
                if not activity:
                    g[i] = 0.0

            f += self.mu * np.dot(g.T, g)       # square penalty
            f += np.dot(self.lambda_g.T, g)     # lagrange

        return f

    def gradient(self, x):
        J = self.pb.f.gradient(x)

        if self.pb.h:
            h, Jh, _ = eval(self.pb.h, x)

            for i in range(self.pb.h.dim()):
                Jb = 2 * np.dot(Jh[i].T, h[i])

                assert J.shape == Jb.shape, "wrong hessian shapes"
                assert J.shape == Jh[i].shape, "wrong hessian shapes"

                J += self.mu * Jb             # barrier
                J += self.lambda_h[i] * Jh[i] # lagrange

        if self.pb.g:
            g, Jg, _ = eval(self.pb.g, x)

            for i in range(self.pb.g.dim()):
                activity = g[i] >= 0 or self.lambda_g[i] > 0
                if activity:
                    Jb = 2 * np.dot(Jg[i].T, g[i])

                    assert J.shape == Jb.shape, "wrong hessian shapes"
                    assert J.shape == Jg[i].shape, "wrong hessian shapes"

                    J += self.mu * Jb + self.lambda_g[i] * Jg[i]

        return J

    def hessian(self, x):
        H = self.pb.f.hessian(x).copy()  # hessian of f

        if self.pb.h:
            _, Jh, Hh = eval(self.pb.h, x)

            for i in range(self.pb.h.dim()):
                _Jh = np.array([Jh[i]])
                Hb = 2 * _Jh.T * _Jh  # pseudo hessian of the barrier

                assert H.shape == Hb.shape, "wrong hessian shapes"
                assert H.shape == Hh[i].shape, "wrong hessian shapes"

                H += self.mu * Hb                   # barrier
                H += self.lambda_h[i] * Hh[i]       # lagrange

        if self.pb.g:
            g, Jg, Hg = eval(self.pb.g, x)

            for i in range(self.pb.g.dim()):
                activity = g[i] >= 0 or self.lambda_g[i] > 0
                if activity:
                    # Hb = 2 * np.dot(Jg.T, Jg) # pseudo hessian of the barrier
                    _Jg = np.array([Jg[i]])
                    Hb = 2 * _Jg.T * _Jg  # pseudo hessian of the barrier

                    assert H.shape == Hb.shape, "wrong hessian shapes"
                    assert H.shape == Hg[i].shape, "wrong hessian shapes"

                    H += self.mu * Hb + self.lambda_g[i] * Hg[i]

        return H

class AugmentedLagrangianSolver:
    def __init__(self, pb, mu=1.0):
        self.constrainedProblem = pb
        self.eps = 0.001 #max constraint violation
        self.mu = mu
        self.rho = 1.0 # how much we increase the square penalty at each cycle
        self.lambda_h = 0.0
        self.lambda_g = 0.0

    def run(self, x, observer=None):
        if observer:
            observer.on_aula_start(x)

        lagrangian = Lagrangian(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
        gn = Newton(lagrangian)
        x = gn.run(x, observer=observer)
        h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else 0
        g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else 0

        i = 0
        while True:
            print("it={}, lambda_h={}, h={}, lambda_g={}, g={}".format(i, self.lambda_h, h, self.lambda_g, g))

            lagrangian = Lagrangian(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
            solver = Newton(lagrangian)
            x = solver.run(x, observer=observer)
            h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else np.array([0.0])
            g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else np.array([0.0])

            self.lambda_h = self.lambda_h + 2 * self.mu * h
            self.lambda_g = self.lambda_g + 2 * self.mu * g
            self.mu *= self.rho

            if np.abs(h).max() < self.eps and g.max() < self.eps:
                break

            i += 1

        if observer:
            observer.on_aula_end(x)

        return x