import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

L = 5.8
W = 1.9
D = 1

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


def draw_car(x):
    fig, ax = plt.subplots()
    patches = []
    for i, _x in enumerate(x):
        color = 1.0 - i / 2 / len(x)
        p = np.zeros(shape=(4, 2))
        theta = -_x[2]
        p[0, :] = _x[:2] + rotate([-D, - 0.5 * W], theta)
        p[1, :] = _x[:2] + rotate([-D, 0.5 * W], theta)
        p[2, :] = _x[:2] + rotate([(L-D), 0.5 * W], theta)
        p[3, :] = _x[:2] + rotate([(L-D), - 0.5 * W], theta)
        polygon = Polygon(p, closed=True)
        patches.append(polygon)
    p = PatchCollection(patches, alpha=0.4)
    ax.add_collection(p)
    ax.set_aspect(1.0)
    plt.autoscale(tight=True)
    plt.show()