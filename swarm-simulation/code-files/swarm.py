import numpy as np
import time
import pybullet as p
import time
from env import SearchAreaAviary


def generate_drone_positions(num_drones, radius, home_xy):
    positions = []
    for i in range(num_drones):
        angle = (2 * np.pi / num_drones) * i
        x = home_xy[0] + radius * np.cos(angle)
        y = home_xy[1] + radius * np.sin(angle)
        z = 0.03
        positions.append([x, y, z])
    return np.array(positions)
