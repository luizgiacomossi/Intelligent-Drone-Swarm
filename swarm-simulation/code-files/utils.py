# utils.py
import numpy as np
from config import FLY_HEIGHT, SECTION_SWEEP_STEPS, LAWNMOWER_MARGIN_FACTOR

def generate_drone_positions(num_agents, home_xy):
    # Adaptive radius: increases slowly with sqrt(N)
    radius = 0.8 + 0.25 * np.sqrt(num_agents)
    positions = []
    for i in range(num_agents):
        angle = (2 * np.pi / num_agents) * i
        x = home_xy[0] + radius * np.cos(angle)
        y = home_xy[1] + radius * np.sin(angle)
        z = 0.03
        positions.append([x, y, z])
    return np.array(positions), radius

def generate_lawnmower_points(center, section_size, steps):
    # Generate lawn-mower pattern fully inside each section.
    cx, cy = center
    half = section_size / 2
    margin = LAWNMOWER_MARGIN_FACTOR * section_size   # a margin of 30% so that the drones cover most of the section without hitting borders.
    x1, x2 = cx - half + margin, cx + half - margin
    y1, y2 = cy - half + margin, cy + half - margin
    ys = np.linspace(y1, y2, steps)
    pts = []
    flip = False # acts like a switch for the drone to go from (x1 -> x2) when False and then (x2 -> x1) when True and etc.
    for y in ys:
        if not flip:
            pts += [(x1, y, FLY_HEIGHT), (x2, y, FLY_HEIGHT)]
        else:
            pts += [(x2, y, FLY_HEIGHT), (x1, y, FLY_HEIGHT)]
        flip = not flip
    return pts

def rebuild_tasks_from_market(drone_tasks, market, area, swarm, num_agents):
    for drone_id in range(num_agents):
        owned = [s for s in market.sections if s["owner"] == drone_id and not s["searched"]]
        if not owned:
            drone_tasks[drone_id] = iter([])
            continue

        drone_xy = np.array(swarm[drone_id].position)[:2]
        owned.sort(key=lambda s: np.linalg.norm(drone_xy - s["pos"][:2]))

        paths = []
        for sec in owned:
            path = generate_lawnmower_points(sec["pos"], area.section_size, SECTION_SWEEP_STEPS)
            paths.append((sec["id"], path))
        drone_tasks[drone_id] = iter(paths)
