import numpy as np
import time
import pybullet as p
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
from tables import HEALTH_CODES
from config import FLY_HEIGHT, RETURN_HOME_DIST_THRESHOLD, RETURN_HOME_SPEED_DIVISOR, RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX, MAX_SPEED, RETURN_HOME_STEP_MULTIPLIER, LAWNMOWER_MARGIN_FACTOR

DEBUG = False

class Drone:
    """Drone agent with selective communication behavior."""

    def __init__(self, drone_id, env, init_position):
        self.id = drone_id
        self.env = env
        self.position = np.array(init_position, dtype=float)
        self.waypoints = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.velocity = np.array([0, 0, 0], dtype=float)
        self.last_velocity = np.array([0, 0, 0], dtype=float)
        self.acceleration = np.array([0, 0, 0], dtype=float)

        # --- mission section ---
        self.mission_section = None

        # --- Drone state ---
        self.health_status = "OK"
        self.last_health_status = "OK"
        self.health_code = [k for k, v in HEALTH_CODES.items() if v == "OK"][0]

        self.current_command = "IDLE"
        self.last_command = "IDLE"

        self.subject_found = 0
        self.timer_start = time.time()

        # --- Controller ---
        self.controller = DSLPIDControl(drone_model=DroneModel.CF2X)

    def update(self):
        state = self.env._getDroneStateVector(self.id)
        self.position = np.array(state[0:3])

        # Physics updates
        vel, _ = p.getBaseVelocity(self.env.DRONE_IDS[self.id], physicsClientId=self.env.CLIENT)
             
        self.velocity = np.array(vel)
        self.acceleration = (self.velocity - self.last_velocity) * self.env.CTRL_FREQ
        self.last_velocity = self.velocity
        
        if self.id == 0 and DEBUG:
            print(f"DEBUG: Drone ID: {id(self)}")
            print(f"DEBUG: last_vel={self.last_velocity}, new_vel={vel}")
            print("-" * 20)

        return state

    def step_toward(self, target_pos):
        # Do not call self.update() here to avoid double-updating physics history
        state = self.env._getDroneStateVector(self.id)
        rpm, _, _ = self.controller.computeControlFromState(
            control_timestep=1 / self.env.CTRL_FREQ,
            state=state,
            target_pos=np.array(target_pos)
        )
        return rpm

    def set_health(self, status):
        """Change health status."""
        valid_values = HEALTH_CODES.values()
        if status not in valid_values:
            raise ValueError(f"Unknown health status: {status}")
        self.last_health_status = self.health_status
        self.health_status = status
        for code, name in HEALTH_CODES.items():
            if name == status:
                self.health_code = code
                break

    def set_command(self, command):
        """Set current drone command."""
        self.last_command = self.current_command
        self.current_command = command

    def broadcast(self):
        """Broadcast basic info and health if needed."""
        pos = np.round(self.position, 2).tolist()
        msg = {
            "ID": int(self.id),
            "Pos": tuple(pos),
            "Timer": round(time.time() - self.timer_start, 2),
        }

        # Broadcast health if changed or not OK
        if self.health_status != "OK" or self.health_status != self.last_health_status:
            msg["Health"] = {
                "Status": self.health_status,
                "Code": int(self.health_code)
            }
            msg["Command"] = {"Command": self.current_command}

        if self.subject_found:
            msg["SubjectFound"] = True
            self.subject_found = 0

        self.last_health_status = self.health_status
        self.last_command = self.current_command

        return msg

    def get_state(self):
        return {
            "ID": int(self.id),
            "Pos": tuple(np.round(self.position, 2).tolist()),
            "Timer": round(time.time() - self.timer_start, 2),
            "Health": {
                "Status": self.health_status,
                "Code": int(self.health_code)
            },
            "Command": {"Command": self.current_command}
        }
        
    def set_waypoint(self, waypoint):
        self.waypoint.append(waypoint)
    
    def return_home(self):
        target_pos = np.array([self.home_targets[i][0], self.home_targets[i][1], FLY_HEIGHT])
        diff = target_pos - self.position
        dist = np.linalg.norm(diff)
        
        if dist > RETURN_HOME_DIST_THRESHOLD:
            direction = diff / (dist + 1e-6)
            speed_scale = np.clip(dist / RETURN_HOME_SPEED_DIVISOR, RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX)
            move_dist = min(dist, MAX_SPEED * speed_scale / self.ctrl_freq * RETURN_HOME_STEP_MULTIPLIER)
            next_pos = self.position + direction * move_dist
        else:
            next_pos = target_pos
            
        next_pos[2] = FLY_HEIGHT
        return self.step_toward(next_pos)

    def generate_lawnmower_points(self, center, section_size, steps):
        """
        Generates a lawnmower pattern with intermediate waypoints along scan lines.
        
        Args:
            center (tuple): (x, y) center of the section.
            section_size (float): Width/Height of the square section.
            steps (int): Number of horizontal scan lines (rows).
        """
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