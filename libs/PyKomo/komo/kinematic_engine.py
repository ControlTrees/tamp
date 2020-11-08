import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Polygon

def rotate(point, radians, origin=(0, 0)):
    """Rotate a point around a given point.

    I call this the "low performance" version since it's recalculating
    the same values more than once [cos(radians), sin(radians), x-ox, y-oy).
    It's more readable than the next function, though.
    """
    x, y = point
    ox, oy = origin

    qx = ox + math.cos(radians) * (x - ox) + math.sin(radians) * (y - oy)
    qy = oy + -math.sin(radians) * (x - ox) + math.cos(radians) * (y - oy)

    return qx, qy

class Joint:
    def __init__(self, shape_name=""):
        self.shape_name = shape_name
        self.dim = 3

    def compute_pose(self, q, q_id):
        return q[q_id: q_id + self.dim]

class Shape:
    def __init__(self, name=""):
        self.name = name

    # abstract
    def draw(self, patches):
        pass

class Circle(Shape):
    def __init__(self, name="", radius=1.0):
        super(Circle, self).__init__(name=name)
        self.radius = radius

    def draw(self, pose, patches):
        circle = plt.Circle(pose[:2], self.radius, color='r')
        patches.append(circle)

class Car(Shape):
    def __init__(self, name="", theta=1.0):
        super(Car, self).__init__(name=name)
        self.L = 5.8
        self.W = 1.9
        self.D = 1

    def draw(self, pose, patches):
        p = np.zeros(shape=(4, 2))
        pos = pose[:2]
        theta = - pose[2]
        p[0, :] = np.array(pos) + rotate([-self.D, - 0.5 * self.W], theta)
        p[1, :] = np.array(pos) + rotate([-self.D, 0.5 * self.W], theta)
        p[2, :] = np.array(pos) + rotate([(self.L - self.D), 0.5 * self.W], theta)
        p[3, :] = np.array(pos) + rotate([(self.L - self.D), - 0.5 * self.W], theta)
        car = Polygon(p, closed=True)
        patches.append(car)

class KinematicWorld:
    def __init__(self):
        self.shapes = []
        self.names_to_shapes={}
        self.joints = []
        self.names_to_joints={}
        self.names_to_q_id={} #start of joint in q vector
        self.fixed_poses={} #only for unoptimized objects (ie no joints attached to them)

    def add_shape(self, shape):
        self.shapes.append(shape)
        self.names_to_shapes[shape.name] = shape

    def add_joint(self, joint):
        old_dim = self.get_dim()
        self.joints.append(joint)
        self.names_to_joints[joint.shape_name] = joint
        self.names_to_q_id[joint.shape_name] = old_dim

    def get_shape(self, name):
        return self.names_to_shapes[name]

    def get_dim(self):
        dim = 0
        for j in self.joints:
            dim+=j.dim
        return dim

    def get_pose(self, shape_name, q):
        if shape_name in self.fixed_poses:
            return self.get_fixed_pose(shape_name)
        elif shape_name in self.names_to_joints:
            return get_joint_pose(shape_name, q)

    def set_fixed_pose(self, shape_name, pose):
        self.fixed_poses[shape_name]=pose

    def get_fixed_pose(self, shape_name):
        return self.fixed_poses[shape_name]

    def get_joint_pose(self, shape_name, q):
        joint = self.names_to_joints[shape_name]
        q_id = self.names_to_q_id[shape_name]
        pose = joint.compute_pose(q, q_id)
        return pose

    def draw(self, x):
        import matplotlib.pyplot as plt
        from matplotlib.collections import PatchCollection

        fig, ax = plt.subplots()
        patches = []
        self.draw_on_patches(x, patches)
        p = PatchCollection(patches, alpha=0.4)
        ax.add_collection(p)
        ax.set_aspect(1.0)
        plt.autoscale(tight=True)
        plt.show()

    def draw_on_patches(self, x, patches):
        assert x.shape[1] == self.get_dim(), "wrong trajectory dimension:{} vs {}".format(x.shape[1], self.get_dim())

        for s in self.shapes:
            if s.name in self.fixed_poses:
                s.draw(self.get_fixed_pose(s.name), patches)
                continue
            elif s.name in self.names_to_joints:
                for i in range(x.shape[0]):
                    s.draw(self.get_joint_pose(s.name, x[i]), patches)
