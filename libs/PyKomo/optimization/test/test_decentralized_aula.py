import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import ADMMProblem_Newton, ADMMProblemN, ADMMProblem, ConstrainedProblem
from decentralized_aula import DecentralizedAugmentedLagrangianSolver, DecentralizedAugmentedLagrangianSolverN
from functions import *
from observers import *

def test_unconstrained_dec_aula_3d():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula (unconstrained)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=1, sz=0.0))
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=0, sz=1.0))
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = DecentralizedAugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([1.0, 1.0, 1.0]), decimal=1)

    p.report()


def test_constrained_dec_aula_3d():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=1, sz=0.0), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=0, sz=1.0), h=ProjX())
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = DecentralizedAugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)

    p.report(plot=True)

def test_constrained_dec_aula_3d_n():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=1, sz=0.0), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=0, sz=1.0), h=ProjX())
    pb = ADMMProblemN(pbs=[pb0, pb1])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)

    p.report(plot=True)

def test_constrained_dec_aula_3d_n_battling_over_y():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1.5, 1), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 0.5, 1), h=ProjX())
    pb = ADMMProblemN(pbs=[pb0, pb1])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)

    p.report(plot=True)

def test_constrained_dec_aula_3d_n_battling_over_y_scaling_in_one_dir():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1.5, 1, sy=1.5), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 0.5, 1, sy=0.5), h=ProjX())
    pb = ADMMProblemN(pbs=[pb0, pb1])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    nt.assert_almost_equals(x[0], 0.00, delta=0.001)
    nt.assert_true(x[1] > 1.00)
    nt.assert_almost_equals(x[2], 1.00, delta=0.01)

    p.report(plot=True)


def test_constrained_dec_aula_3d_sphere():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:sphere)")
    p.add_point(x0)

    h = SphereConstraint3D(cx=0, cy=0.5, cz=0.5, radius=0.5)
    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=1, sz=0.0), h=h)
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=0, sz=1.0), h=h)
    pb = ADMMProblemN(pbs=[pb0, pb1])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    nt.assert_almost_equals(h.value(x), 0, delta=0.001)

    p.report(plot=True)

def test_constrained_dec_aula_3d_use_of_support():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula use support(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=1, sz=0.0), h=ProjX(), m=np.array([1.0, 1.0, 0.0]))
    pb1 = ConstrainedProblem(f=SquareDistance3D(1, 1, 1, sx=np.sqrt(0.5), sy=0, sz=1.0), h=ProjX(), m=np.array([1.0, 0.0, 1.0]))
    pb = ADMMProblemN(pbs=[pb0, pb1])
    solver = DecentralizedAugmentedLagrangianSolverN(pb)
    x = solver.run(x0, observer=p)

    p.report(plot=True)