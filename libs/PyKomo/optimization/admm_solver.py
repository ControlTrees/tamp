import numpy as np
from newton import NewtonFunction, Newton
from optimization_problems import ConstrainedProblem
from augmented_lagrangian_solver import AugmentedLagrangianSolver

class ADMMLagrangian0(NewtonFunction):
    def __init__(self, f, xk, y, mu):
        self.f = f
        self.xk = xk
        self.y = y
        self.mu = mu

    def value(self, x):
        delta = x - self.xk
        return self.f.value(x) + np.dot(self.y.T, delta) + 0.5 * self.mu * np.dot(delta.T, delta)

    def gradient(self, x):
        delta = x - self.xk
        return self.f.gradient(x) + self.y.T + self.mu * delta

    def hessian(self, x):
        h = self.f.hessian(x)
        Hb = np.identity(x.shape[0])
        return h + self.mu * Hb


class ADMMLagrangian1(NewtonFunction):
    def __init__(self, f, xk, y, mu):
        self.f = f
        self.xk = xk
        self.y = y
        self.mu = mu

    def value(self, x):
        delta = self.xk - x
        return self.f.value(x) + np.dot(self.y.T, delta) + 0.5 * self.mu * np.dot(delta.T, delta)

    def gradient(self, x):
        delta = self.xk - x
        return self.f.gradient(x) - self.y.T - self.mu * delta

    def hessian(self, x):
        h = self.f.hessian(x)
        Hb = np.identity(x.shape[0])
        return h + self.mu * Hb

class ADMMSolver:
    def __init__(self, pb):
        self.pb = pb
        self.y = 0
        self.rho = 1
        self.eps = 0.001 #max constraint violation

    def run(self, x, observer=None):
        self.y = np.zeros(x.shape)

        i = 0
        x0 = x
        x1 = x
        while True:
            print("IT={}".format(i))

            f0 = ADMMLagrangian0(self.pb.pb0.f, x1, self.y, self.rho)
            pb0 = ConstrainedProblem(f=f0, h=self.pb.pb0.h, g=self.pb.pb0.g)
            assert f0.checkGradients(x1) and f0.checkHessian(x1)
            x0 = AugmentedLagrangianSolver(pb0).run(x1, observer=observer)

            f1 = ADMMLagrangian1(self.pb.pb1.f, x0, self.y, self.rho)
            pb1 = ConstrainedProblem(f=f1, h=self.pb.pb1.h, g=self.pb.pb1.g)
            assert f1.checkGradients(x0) and f1.checkHessian(x1)
            x1 = AugmentedLagrangianSolver(pb1).run(x0, observer=observer)

            delta = x0 - x1
            self.y += self.rho * delta

            if np.abs(delta).max() < self.eps:
                break

            i+=1

        return x1

class ADMMSolver_Newton:
    def __init__(self, pb, solver_class=Newton):
        self.pb = pb
        self.solver_class = solver_class
        self.y = 0
        self.rho = 1
        self.eps = 0.001 #max constraint violation

    @staticmethod
    def to_0(pb, xk, y, rho):
        class F(NewtonFunction):
            def value(self, x):
                delta = x - xk
                return pb.f0.value(x) + pb.f1.value(xk) + np.dot(y.T, delta) + 0.5 * rho * np.dot(delta.T, delta)

            def gradient(self, x):
                delta = x - xk
                return pb.f0.gradient(x) + y.T + rho * delta

            def hessian(self, x):
                h = pb.f0.hessian(x)
                Hb = np.identity(x.shape[0])
                return h + rho * Hb

        return F()

    @staticmethod
    def to_1(pb, x_fixed, y, rho):
        class F(NewtonFunction):
            def value(self, x):
                delta = x_fixed - x
                return pb.f0.value(x_fixed) + pb.f1.value(x) + np.dot(y.T, delta) + 0.5 * rho * np.dot(delta.T,
                                                                                                  delta)

            def gradient(self, x):
                delta = x_fixed - x
                return pb.f1.gradient(x) - y.T - rho * delta

            def hessian(self, x):
                h = pb.f1.hessian(x)
                Hb = np.identity(x.shape[0])
                return h + rho * Hb

        return F()

    def run(self, x):
        self.y = np.zeros(x.shape)

        x0 = x
        x1 = x
        i = 0
        while True:
            pb0 = self.to_0(self.pb, x1, self.y, self.rho)
            assert pb0.checkGradients(x1) and pb0.checkHessian(x1)
            x0 = self.solver_class(pb0).run(x1)

            pb1 = self.to_1(self.pb, x0, self.y, self.rho)
            assert pb1.checkGradients(x0) and pb1.checkHessian(x0)
            x1 = self.solver_class(pb1).run(x0)

            delta = x0 - x1
            self.y += self.rho * delta

            if np.abs(delta).max() < self.eps:
                break

        return x1