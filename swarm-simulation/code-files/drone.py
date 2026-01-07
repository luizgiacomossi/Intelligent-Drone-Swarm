import numpy as np
import time
import pybullet as p
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
from tables import HEALTH_CODES
from guidance import Guidance
from config import (
    FLY_HEIGHT, SECTION_SWEEP_STEPS, RETURN_HOME_DIST_THRESHOLD,
    RETURN_HOME_SPEED_DIVISOR, RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX,
    MAX_SPEED, RETURN_HOME_STEP_MULTIPLIER, LAWNMOWER_MARGIN_FACTOR,
    NAV_STD_SPEED_DIVISOR, NAV_STD_SPEED_MIN, NAV_STD_SPEED_MAX,
    NAV_AGGRESSIVE_SPEED_SCALE, NAV_GLOBAL_STEP_MULTIPLIER
)

DEBUG = False

class Drone:
    """Drone agent with selective communication behavior."""

    def __init__(self, drone_id: int, env, init_position: list):
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
        self.assigned_section_center = None
        self.assigned_section_size = None

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

    def assign_section(self, section_id: int, center: tuple, section_size: float) -> None:
        """
        Assigns a section to the drone and generates the scan path using Guidance logic.

        Args:
            section_id (int): ID of the section.
            center (tuple): (x, y) center of the section.
            section_size (float): Size of the section.
        """
        self.mission_section = section_id
        self.assigned_section_center = center
        self.assigned_section_size = section_size
        
        # Calculate waypoints using Guidance module
        # Note: Using the original generate_lawnmower_points as it was used before.
        # Use generate_lawnmower_points_new if denser points are needed.
        self.waypoints = Guidance.generate_lawnmower_points(center, section_size, SECTION_SWEEP_STEPS)
        self.waypoint_index = 0

    def compute_return_home_step(self, home_pos: np.ndarray, fly_height: float, ctrl_freq: float) -> np.ndarray:
        """
        Calculates the RPM action to return to home position.

        Args:
            home_pos (np.ndarray): Target home position (x, y).
            fly_height (float): Target flying height.
            ctrl_freq (float): Control frequency.

        Returns:
            np.ndarray: RPM action.
        """
        target_pos = np.array([home_pos[0], home_pos[1], fly_height])
        diff = target_pos - self.position
        dist = np.linalg.norm(diff)
        
        if dist > RETURN_HOME_DIST_THRESHOLD:
            direction = diff / (dist + 1e-6)
            speed_scale = np.clip(dist / RETURN_HOME_SPEED_DIVISOR, RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX)
            move_dist = min(dist, MAX_SPEED * speed_scale / ctrl_freq * RETURN_HOME_STEP_MULTIPLIER)
            next_pos = self.position + direction * move_dist
        else:
            next_pos = target_pos
            
        next_pos[2] = fly_height
        return self.step_toward(next_pos)

    def compute_search_step(self, target_pos: np.ndarray, flight_mode: str, avoidance_vec: np.ndarray, ctrl_freq: float) -> np.ndarray:
        """
        Calculates the RPM action to move towards a search target with avoidance.

        Args:
            target_pos (np.ndarray): Target waypoint position.
            flight_mode (str): Flight mode ('standard' or 'aggressive').
            avoidance_vec (np.ndarray): Calculated avoidance vector.
            ctrl_freq (float): Control frequency.

        Returns:
            np.ndarray: RPM action.
        """
        diff = np.array(target_pos) - self.position
        dist = np.linalg.norm(diff)
        
        speed_scale = 1.0
        if flight_mode == "standard":
             speed_scale = np.clip(dist / NAV_STD_SPEED_DIVISOR, NAV_STD_SPEED_MIN, NAV_STD_SPEED_MAX)
        elif flight_mode == "aggressive":
             speed_scale = NAV_AGGRESSIVE_SPEED_SCALE

        move_dist = min(dist * speed_scale, MAX_SPEED * speed_scale / ctrl_freq * NAV_GLOBAL_STEP_MULTIPLIER)
        direction = diff / (dist + 1e-6)
        next_pos = self.position + direction * move_dist

        # Apply avoidance
        next_pos += avoidance_vec
        
        next_pos[2] = FLY_HEIGHT
        return self.step_toward(next_pos)
    
    def check_waypoint_reached(self, target_pos: np.ndarray, tolerance: float) -> bool:
        """
        Checks if the drone has reached the target waypoint.

        Args:
            target_pos (np.ndarray): Target position.
            tolerance (float): Distance tolerance.

        Returns:
            bool: True if reached, False otherwise.
        """
        diff = np.array(target_pos) - self.position
        dist = np.linalg.norm(diff)
        return dist < tolerance

    def detect_subject(self, subject_pos: np.ndarray, detection_dist: float) -> bool:
        """
        Checks if the subject is within detection range.

        Args:
            subject_pos (np.ndarray): Position of the subject (x, y).
            detection_dist (float): Maximum detection distance.

        Returns:
            bool: True if detected.
        """
        drone_xy = self.position[:2]
        subject_xy = np.array(subject_pos[:2])
        dist_to_subject = np.linalg.norm(drone_xy - subject_xy)
        return dist_to_subject < detection_dist