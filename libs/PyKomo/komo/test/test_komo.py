import nose.tools as nt
import sys
import time
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from komo import PyKOMO
from tree_builder import TreeBuilder
from task_map import *
from kinematic_engine import KinematicWorld, Circle, Car, Joint

SHOW_PLOTS = True

def build_linear_traj():
    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3)
    pb.add_edge(3, 4)
    pb.add_edge(4, 5)
    pb.add_edge(5, 6)

    path = pb.get_paths()

    return path, pb.n_nodes()

def build_2_branchs_tree(p_1):
    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3, p_1)
    pb.add_edge(3, 4, p_1)
    pb.add_edge(4, 5, p_1)
    pb.add_edge(5, 6, p_1)

    pb.add_edge(2, 7, 1 - p_1)
    pb.add_edge(7, 8, 1 - p_1)
    pb.add_edge(8, 9, 1 - p_1)
    pb.add_edge(9, 10, 1 - p_1)

    path_1, path_2 = pb.get_paths()

    return path_1, path_2, pb.n_nodes()

def build_3_branchs_tree(p_1, p_2):
    p_3 = 1 - p_2 - p_1

    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3, p_1)
    pb.add_edge(3, 4, p_1)
    pb.add_edge(4, 5, p_1)
    pb.add_edge(5, 6, p_1)

    pb.add_edge(2, 7, p_2)
    pb.add_edge(7, 8, p_2)
    pb.add_edge(8, 9, p_2)
    pb.add_edge(9, 10, p_2)

    pb.add_edge(2, 11, p_3)
    pb.add_edge(11, 12, p_3)
    pb.add_edge(12, 13, p_3)
    pb.add_edge(13, 14, p_3)

    path_1, path_2, path_3 = pb.get_paths()

    return path_1, path_2, path_3, pb.n_nodes()

def build_mille_pattes():
    p_1 = 0.2
    p_2 = 0.2
    p_3 = 0.2
    p_4 = 0.2
    p_5 = 1 - p_1 - p_2 - p_3 - p_4

    pb = TreeBuilder()
    pb.add_edge(0, 1, 1.0)
    pb.add_edge(1, 2, 1.0)
    pb.add_edge(2, 3, 1.0 - p_1)
    pb.add_edge(3, 4, 1.0 - p_1)
    pb.add_edge(4, 5, 1.0 - p_1 - p_2)
    pb.add_edge(5, 6, 1.0 - p_1 - p_2)
    pb.add_edge(6, 7, 1.0 - p_1 - p_2)
    pb.add_edge(7, 8, 1.0 - p_1 - p_2 - p_3)
    pb.add_edge(8, 9, 1.0 - p_1 - p_2 - p_3 - p_4)
    pb.add_edge(9, 10, 1.0 - p_1 - p_2 - p_3 - p_4)

    pb.add_edge(2, 11, p_1)
    pb.add_edge(11, 12, p_1)

    pb.add_edge(4, 13, p_2)
    pb.add_edge(13, 14, p_2)

    pb.add_edge(6, 15, p_3)
    pb.add_edge(15, 16, p_3)

    pb.add_edge(8, 17, p_4)
    pb.add_edge(17, 18, p_4)

    path_1, path_2, path_3, path_4, path_5 = pb.get_paths()
    n_steps = pb.n_nodes()

    return path_1, path_2, path_3, path_4, path_5, n_steps

def test_linear_traj():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    path, n_steps = build_linear_traj()

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)
    komo.add_task(AccelerationPenalty())
    komo.add_task(TargetPosition(goal=[100, -30]), start=4, end=-1)
    komo.add_task(CarOrientation())

    start = time.clock()
    x = komo.run(x0)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_tree_2_branches_pos():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    path_1, path_2, n_steps = build_2_branchs_tree(0.5)

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)

    # pos
    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetPosition(goal=[100, 30]), wpath=path_1, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetPosition(goal=[100, -30]), wpath=path_2, start=4, end=-1)

    # vel
    # komo.add_task(AccelerationPenalty(), wpath=path_1)
    # komo.add_task(TargetVelocity(goal=[15, 5]), wpath=path_1, start=4, end=-1)
    #
    # komo.add_task(AccelerationPenalty(), wpath=path_2)
    # komo.add_task(TargetVelocity(goal=[15, -5]), wpath=path_2, start=4, end=-1)

    komo.add_task(CarOrientation(), wpath=path_1)
    komo.add_task(CarOrientation(), wpath=path_2)

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_tree_2_branches_vel():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    path_1, path_2, n_steps = build_2_branchs_tree(0.5)

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)

    # vel
    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetVelocity(goal=[15, 5]), wpath=path_1, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetVelocity(goal=[15, -5]), wpath=path_2, start=4, end=-1)

    komo.add_task(CarOrientation(), wpath=path_1)
    komo.add_task(CarOrientation(), wpath=path_2)

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_tree_3_branches_pos():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    path_1, path_2, path_3, n_steps = build_3_branchs_tree(0.333, 0.333)

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)

    # pos
    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetPosition(goal=[100, 30]), wpath=path_1, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetPosition(goal=[130, 0]), wpath=path_2, start=4, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_3)
    komo.add_task(TargetPosition(goal=[100, -30]), wpath=path_3, start=4, end=-1)

    # vel
    # komo.add_task(AccelerationPenalty(), wpath=path_1)
    # komo.add_task(TargetVelocity(goal=[15, 5]), wpath=path_1, start=4, end=-1)
    #
    # komo.add_task(AccelerationPenalty(), wpath=path_2)
    # komo.add_task(TargetVelocity(goal=[15, -5]), wpath=path_2, start=4, end=-1)

    komo.add_task(CarOrientation(), wpath=path_1)
    komo.add_task(CarOrientation(), wpath=path_2)
    komo.add_task(CarOrientation(), wpath=path_3)

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_mille_pattes():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    path_1, path_2, path_3, path_4, path_5, n_steps = build_mille_pattes()

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)

    # pos
    komo.add_task(TargetPosition(goal=[100, 0]), wpath=path_1, start=8, end=-1)
    komo.add_task(TargetPosition(goal=[30, 20]), wpath=path_2, start=3, end=-1)
    komo.add_task(TargetPosition(goal=[50, -20]), wpath=path_3, start=5, end=-1)
    komo.add_task(TargetPosition(goal=[70, 20]), wpath=path_4, start=7, end=-1)
    komo.add_task(TargetPosition(goal=[90, -20]), wpath=path_5, start=9, end=-1)

    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(AccelerationPenalty(), wpath=path_3)
    komo.add_task(AccelerationPenalty(), wpath=path_4)
    komo.add_task(AccelerationPenalty(), wpath=path_5)

    komo.add_task(CarOrientation(), wpath=path_1)
    komo.add_task(CarOrientation(), wpath=path_2)
    komo.add_task(CarOrientation(), wpath=path_3)
    komo.add_task(CarOrientation(), wpath=path_4)
    komo.add_task(CarOrientation(), wpath=path_5)

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_linear_traj_among_two_circles():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))
    kin.add_shape(Circle(name="circle_1", radius=10.0))
    kin.set_fixed_pose("circle_1", (40, 10, 0))
    kin.add_shape(Circle(name="circle_10", radius=10.0))
    kin.set_fixed_pose("circle_10", (80, -10, 0))

    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3)
    pb.add_edge(3, 4)
    pb.add_edge(4, 5)
    pb.add_edge(5, 6)

    path = pb.get_paths()
    n_steps = pb.n_nodes()

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(CarOrientation())
    komo.add_task(AccelerationPenalty())

    komo.add_task(TargetPosition(goal=[0, 0]), start=0, end=1)
    # komo.add_task(TargetPosition(goal=[100, 1]), start=4, end=5)
    komo.add_task(TargetVelocity(goal=[20, 0]), start=4, end=5)
    komo.add_task(CircleAvoidance(circle_name="circle_1"))
    komo.add_task(CircleAvoidance(circle_name="circle_10"))

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

def test_traj_tree_in_font_of_obstacle():
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))
    kin.add_shape(Circle(name="circle_1", radius=12.0))
    kin.add_shape(Circle(name="circle_2", radius=12.0))
    kin.set_fixed_pose("circle_1", (50, 0, 0))
    kin.set_fixed_pose("circle_2", (60, 0, 0))

    pb = TreeBuilder()
    pb.add_edge(0, 1)

    p = 0.7

    pb.add_edge(1, 2, p)
    pb.add_edge(2, 3, p)
    pb.add_edge(3, 4, p)
    pb.add_edge(4, 5, p)
    pb.add_edge(5, 6, p)

    pb.add_edge(1, 7, 1 - p)
    pb.add_edge(7, 8, 1 - p)
    pb.add_edge(8, 9, 1 - p)
    pb.add_edge(9, 10, 1 - p)
    pb.add_edge(10, 11, 1 - p)

    path_1, path_2 = pb.get_paths()
    n_steps = pb.n_nodes()

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(TargetPosition(goal=[0, 0]), wpath=path_1, start=0, end=1)
    komo.add_task(TargetPosition(goal=[0, 0]), wpath=path_2, start=0, end=1)

    komo.add_task(CarOrientation(), wpath=path_1)
    komo.add_task(AccelerationPenalty(), wpath=path_1)
    komo.add_task(TargetPosition(goal=[100, 50]), wpath=path_1, start=5, end=-1)
    komo.add_task(CircleAvoidance(circle_name="circle_1"), wpath=path_1)
    komo.add_task(CircleAvoidance(circle_name="circle_2"), wpath=path_1)

    komo.add_task(CarOrientation(), wpath=path_2)
    komo.add_task(AccelerationPenalty(), wpath=path_2)
    komo.add_task(TargetPosition(goal=[100, -50]), wpath=path_2, start=5, end=-1)
    komo.add_task(CircleAvoidance(circle_name="circle_1"), wpath=path_2)
    komo.add_task(CircleAvoidance(circle_name="circle_2"), wpath=path_2)

    start = time.clock()
    x = komo.run(x0)  # , initial=initial)
    end = time.clock()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x),
                                                                          komo.equality_constraint(x), end - start))

    # plot traj
    if SHOW_PLOTS:
        kin.draw(x)

if __name__ == "__main__":
     test_traj_tree_in_font_of_obstacle()