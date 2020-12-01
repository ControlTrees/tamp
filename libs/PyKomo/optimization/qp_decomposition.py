import numpy as np
import math
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem

def decompose(qp):
    # tentative decompsition generale de QPs
    # resultats preliminaires montrent qu'il est difficile de trouver un decomposition generale
    # les contraintes font que même si il est possible de decomposer les coûts, la hessienne sera probablement dense
    # une decomposition "informée" semble la meilleure manière
    kQ = math.ceil(qp.Q.shape[0] / 2)
    Q1 = qp.Q[:kQ, :kQ]
    Q2 = qp.Q[:kQ, kQ:]
    Q3 = qp.Q[kQ:, :kQ]
    Q4 = qp.Q[kQ:, kQ:]

    c1 = qp.c[:kQ]
    c4 = qp.c[kQ:]

    #kA = math.ceil(qp.A.shape[0]/2)

    #A2 = qp.A[:kA, :]
    #A3 = qp.A[kA:, :]

    #u2 = qp.u[:kA]
    #u3 = qp.u[kA:]

    _Q1 = np.zeros(qp.Q.shape)
    _Q1[:kQ, :kQ] = Q1
    _c1 = np.zeros(qp.c.shape)
    _c1[:kQ] = c1
    qp1 = ConstrainedQP(Q=_Q1, c=_c1, A=None, u=None)

    _Q4 = np.zeros(qp.Q.shape)
    _Q4[kQ:, kQ:] = Q4
    _c4 = np.zeros(qp.c.shape)
    _c4[kQ:] = c4
    qp2 = ConstrainedQP(Q=_Q4, c=_c4, A=None, u=None)

    _Q2 = np.zeros(qp.Q.shape)
    _Q2[:kQ, kQ:] = Q2
    _Q2[kQ:, :kQ] = Q3
    qp3 = ConstrainedQP(Q=_Q2, c=np.zeros(qp.c.shape), A=qp.A, u=qp.u)

    return [qp1, qp2, qp3]