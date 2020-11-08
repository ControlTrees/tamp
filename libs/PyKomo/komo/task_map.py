import numpy as np
import math
from enum import Enum

class TaskMapType(Enum):
    COST = 1
    EQ = 2

class TaskMap:
    def __init__(self, name = '', order = 0, dim = 1, type = TaskMapType.COST):
        self.order = order
        self.name = name
        self.dim = dim
        self.type = type

    def phi(self, _, context, kinematic_world):
        pass

class TargetPosition(TaskMap):
    def __init__(self, goal=np.array([0, 0]), type = TaskMapType.COST):
        super(TargetPosition, self).__init__(name="target_position", order=0, dim=2, type=type)
        self.goal = goal
    def phi(self, x, context, _):
        cost = x[:2] - self.goal
        Jcost = np.array([[1, 0, 0], [0, 1, 0]])
        return cost, Jcost

class CircleAvoidance(TaskMap):
    def __init__(self, circle_name=""):
        super(CircleAvoidance, self).__init__(name="circle_avoidance", order=0, dim=1, type=TaskMapType.EQ)
        self.circle_name = circle_name
    def phi(self, x, context, kinematic_world):
        pos = kinematic_world.get_pose(self.circle_name, x)[:2]
        circle = kinematic_world.get_shape(self.circle_name)
        d = x[:2]-pos
        norm = np.linalg.norm(x[:2]-pos)
        ego_radius = 4

        if norm > circle.radius + ego_radius:
            cost = np.array([0])
            Jcost = np.zeros(shape=(1, 3))
        else:
            cost = np.array([norm-circle.radius-ego_radius])
            Jcost = np.zeros(shape=(1, 3))
            if norm > 0.0000001:
                for i in range(2):
                    Jcost[0, i] = d[i] / norm
        return cost, Jcost

class TargetVelocity(TaskMap):
    def __init__(self, goal=np.array([0, 0]), type = TaskMapType.COST):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1, dim=2, type=type)
        self.goal = goal
    def phi(self, v, context, _):
        v_cost = v[:2] - self.goal
        Jv_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2, dim=2)
    def phi(self, a, context, _):
        a_cost = np.array([a[0], a[1]])
        Ja_cost = np.array([[1, 0, 0], [0, 1, 0]])
        return a_cost, Ja_cost

class CarOrientation(TaskMap):
    def __init__(self):
        super(CarOrientation, self).__init__(name="car_orientation", order=0, dim=2, type=TaskMapType.EQ)
    def phi(self, x, context, _):
        if context[0] is None:
            cost = np.array([0, 0])
            J_cost = np.array([[0, 0, 0], [0, 0, 0]]) #np.array([[0, 0, -math.sin(x[2])], [0, 0, math.cos(x[2])]])
        else:
            v = (context[1]-context[0])[0:2]
            normV = np.linalg.norm(v)
            if normV == 0:
                cost = np.array([0, 0])
                J_cost = np.array([[0, 0, 0], [0, 0, 0]])
            else:
                v /= normV
                theta = x[2]
                heading = np.array([math.cos(theta), math.sin(theta)])
                cost = heading - v
                J_cost = np.array([[0, 0, -math.sin(theta)], [0, 0, math.cos(theta)]])
        return cost, J_cost