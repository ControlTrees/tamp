import time
from task_map import *
from komo import PyKOMO
from tree_builder import TreeBuilder
from kinematic_engine import KinematicWorld, Circle, Car, Joint

if __name__ == "__main__":
    x0 = np.array([0, 0, 0])
    print("x0:{}".format(x0))

    kin = KinematicWorld()
    kin.add_shape(Car(name="ego_car"))
    kin.add_joint(Joint(shape_name="ego_car"))

    pb = TreeBuilder()
    pb.add_edge(0, 1)
    pb.add_edge(1, 2)
    pb.add_edge(2, 3)
    pb.add_edge(3, 4)
    pb.add_edge(4, 5)
    pb.add_edge(5, 6)
    pb.add_edge(6, 7)
    pb.add_edge(7, 8)
    pb.add_edge(8, 9)
    pb.add_edge(9, 10)
    pb.add_edge(10, 11)


    [path] = pb.get_paths()
    n_steps = pb.n_nodes()

    #
    path.append((0, 1.0))
    path.append((1, 1.0))
    n_steps+=2
    #

    komo = PyKOMO()
    komo.set_kinematic_world(kin)
    komo.set_n_phases(n_steps)

    komo.add_task(AccelerationPenalty(), wpath=path)
    komo.add_task(TargetPosition(goal=[0, 0]), wpath=path, start=0, end=1)
    komo.add_task(TargetPosition(goal=[30, 0]), wpath=path, start=3, end=4)
    komo.add_task(TargetPosition(goal=[30, 30]), wpath=path, start=6, end=7)
    komo.add_task(TargetPosition(goal=[0, 30]), wpath=path, start=9, end=10)
    komo.add_task(CarOrientation(), wpath=path)

    start = time.time()
    x = komo.run(x0)  # , initial=initial)
    end = time.time()

    print("final x:{}\ncost(x):{} \neq constraint:{}\nopt-time:{}".format(x, komo.traj_cost(x), komo.equality_constraint(x), end - start))

    kin.draw(x)
