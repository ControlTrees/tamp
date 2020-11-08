import nose.tools as nt
import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from kinematic_engine import KinematicWorld, Shape, Circle, Car, Joint
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

DRAW = True

def build_drawer():
    fig, ax = plt.subplots()
    patches = []

    return ax, patches

def draw(ax, patches):
    if not DRAW:
        return

    p = PatchCollection(patches, alpha=0.4)
    ax.add_collection(p)
    ax.set_aspect(1.0)
    plt.autoscale(tight=True)
    plt.show()

def test_shape_creation():
    s = Shape()
    nt.assert_true(hasattr(s, "name"))

def test_build_circle():
    c = Circle(radius=2.0)
    nt.assert_equal(2.0, c.radius)

def test_draw_circle():
    c = Circle(radius=2.0)
    ax, patches = build_drawer()
    c.draw((0, 0, 0), patches)
    nt.assert_equal(1, len(patches))
    draw(ax, patches)

def test_build_car():
    c = Car()
    nt.assert_equals(1.0, c.D)

def test_draw_car():
    c = Car()
    ax, patches = build_drawer()
    c.draw((0, 0, 1), patches)
    nt.assert_equal(1, len(patches))
    draw(ax, patches)

def test_build_kinematic_world():
    k = KinematicWorld()
    nt.assert_true(hasattr(k, "shapes"))

def test_add_shapes_kinematic_world():
    k = KinematicWorld()
    k.add_shape(Circle(radius=2.0))
    k.add_shape(Car())
    nt.assert_equal(2, len(k.shapes))

def test_get_shape_kinematic_world():
    k = KinematicWorld()
    k.add_shape(Circle(name="circle", radius=2.0))
    k.add_shape(Car(name="car"))
    car = k.get_shape("car")
    nt.assert_true(car is not None)

def test_create_joint():
    j = Joint(shape_name="circle")
    nt.assert_true(j is not None)
    nt.assert_equals(3, j.dim)
    nt.assert_equal("circle", j.shape_name)

def test_add_joint_to_kin_engine():
    k = KinematicWorld()
    k.add_shape(Circle(name="circle", radius=2.0))
    k.add_joint(Joint(shape_name="circle"))
    nt.assert_equal(1, len(k.joints))

def test_add_joint_to_kin_engine():
    k = KinematicWorld()
    k.add_shape(Circle(name="circle", radius=2.0))
    k.add_joint(Joint(shape_name="circle"))
    nt.assert_equal(3, k.get_dim())

def test_set_pose_to_kin_engine():
    k = KinematicWorld()
    k.add_shape(Circle(name="circle", radius=2.0))
    k.set_fixed_pose("circle", (1, 0, 0))
    nt.assert_equal( (1, 0, 0), k.get_fixed_pose("circle") )

def test_draw_kinematic_world():
    k = KinematicWorld()
    k.add_shape(Circle(name="circle", radius=1.0))
    k.set_fixed_pose("circle", (0,0,0))
    k.add_shape(Car(name="car"))
    k.add_joint(Joint(shape_name="car"))
    k.draw(np.array([[0, 0, -1]]))
