import numpy as np
from augmented_lagrangian_solver import AugmentedLagrangianSolver
from newton import NewtonFunction
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem

class UnconstrainedQPSolver:
    def __init__(self, qp):
        self.qp = qp

    def run(self):
        A = self.qp.Q
        B = -self.qp.c

        x = np.linalg.solve(A, B)

        return x

class F(NewtonFunction):
    def __init__(self, qp):
        self.qp = qp

    def value(self, x):
        return 0.5 * np.dot(x.T, np.dot(self.qp.Q, x)) + np.dot(self.qp.c.T, x)

    def gradient(self, x):
        return np.dot(self.qp.Q, x) + self.qp.c

    def hessian(self, x):
        return self.qp.Q

class G(NewtonFunction):
    def __init__(self, qp):
        self.qp = qp

    def value(self, x):
        return np.dot(self.qp.A, x) - self.qp.u

    def gradient(self, x):
        return self.qp.A

    def hessian(self, x):
        return np.zeros((self.qp.A.shape[0], self.qp.A.shape[0], x.shape[0]))

    def dim(self):
        return self.qp.A.shape[0]

class ConstrainedQPSolver:
    def __init__(self, qp, mu=1.0):
        self.qp = qp
        self.mu = mu

        assert self.check_symmetric(self.qp.Q), "Q should be symetric"

    def check_symmetric(self, Q, rtol=1e-05, atol=1e-08):
        return np.allclose(Q, Q.T, rtol=rtol, atol=atol)

    def run(self, x, observer=None):
        f = F(self.qp)
        g = G(self.qp)

        pb = ConstrainedProblem(f=f, g=g)

        al = AugmentedLagrangianSolver(pb, mu=self.mu)
        x = al.run(x, observer=observer)

        return x