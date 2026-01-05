import numpy as np
import time
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
from tables import HEALTH_CODES


class Drone:
    """Drone agent with selective communication behavior."""

    def __init__(self, drone_id, env, init_position):
        self.id = drone_id
        self.env = env
        self.position = np.array(init_position, dtype=float)

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
        return state

    def step_toward(self, target_pos):
        state = self.update()
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
