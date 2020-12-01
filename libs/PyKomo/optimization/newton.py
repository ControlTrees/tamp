import numpy as np
import copy

class NewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

    def hessian(self, x):
        pass

    def checkGradients(self, x):
        dx = 0.001
        y = self.value(x)
        j = self.gradient(x)
        diff = np.zeros(x.shape[0])

        for i in range(0, x.shape[0]):
            x_ = copy.copy(x)
            x_[i] += dx
            y_ = self.value(x_)
            dy = y_ - y
            ji = dy / dx
            diff[i] = j[i] - ji

        close = np.all(np.abs(diff) < 0.01)
        if not close:
            print("gradient problem, diff={}".format(diff))

        return close

    def checkHessian(self, x):
        dx = 0.001
        h  = self.hessian(x)

        close = True

        y = self.value(x)
        for i in range(0, x.shape[0]):
            xp = x.copy()
            xp[i] += dx
            xm = x.copy()
            xm[i] -= dx
            yp = self.value(xp)
            ym = self.value(xm)
            hii = (ym - 2*y + yp) / (dx * dx)
            close = close and np.abs(h[i, i] - hii) < 0.01

        return close

    def dim(self):
        return 1

class VectorizedNewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

    def hessian(self, x):
        pass

    def dim(self):
        pass

class Newton: # sum of square problems
    def __init__(self, function):
        self.function = function
        self.lambda_0 = 0.1
        self.rho = 0.1  # wolfe - minimum acceptaded decrease
        self.eps = 0.01  # update size

        assert issubclass(type(function), NewtonFunction), "wrong function type"

    def run(self, x, observer=None):
        _lambda = self.lambda_0
        _alpha = 1.0

        if observer:
            observer.on_newton_start(x)

        I = np.identity(x.shape[0])
        while True:
            if observer:
                observer.on_newton_step(x)

            hessian = self.function.hessian(x)
            A = hessian + _lambda * I # damping
            B = -self.function.gradient(x)

            d = np.linalg.solve(A, B)

            # print("x:\n{}".format(x))
            # print("A:\n{}".format(A))
            # print("g:\n{}".format(-B))
            # print("Delta:\n{}".format(d))
            if observer:
                observer.on_newton_line_search(x)

            v = self.function.value(x)
            w = self.function.value(x + _alpha * d)   # line search
            w2 = v + self.rho * np.matmul(np.transpose(B), _alpha * d)

            while w > w2:
                _alpha = _alpha * 0.5
                w = self.function.value(x + _alpha * d)
                w2 = v + self.rho * np.matmul(np.transpose(B), _alpha * d)

                if observer:
                    observer.on_newton_line_search(x)

            x = x + _alpha * d

            _alpha = min(1.2 * _alpha, 1.0)

            if np.linalg.norm(_alpha * d) < self.eps:
                break

        if observer:
            observer.on_newton_end(x)

        return x
