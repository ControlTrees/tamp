import numpy as np

class ConstrainedProblem:
    def __init__(self, f, h=None, g=None, m=None):
        self.f = f
        self.h = h
        self.g = g
        self.m = m # mask, tells wether problem contributes to a given index i of x


class UnconstrainedQP:
    def __init__(self, Q, c):
        self.Q = Q  # hessian
        self.c = c
        self.Q.flags.writeable = False
        self.c.flags.writeable = False

class ConstrainedQP: # Ax <= u
    def __init__(self, Q, c, A, u):
        self.Q = Q  # hessian
        self.c = c
        self.A = A
        self.u = u
        self.Q.flags.writeable = False
        self.c.flags.writeable = False
        if self.A is not None:
            self.A.flags.writeable = False
        if self.u is not None:
            self.u.flags.writeable = False

class ADMMProblemN:
    def __init__(self, pbs):
        self.pbs = pbs

class ADMMProblem:
    def __init__(self, pb0, pb1):
        self.pb0 = pb0
        self.pb1 = pb1

class ADMMProblem_Newton:
    def __init__(self, f0, f1):
        self.f0 = f0
        self.f1 = f1

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)

