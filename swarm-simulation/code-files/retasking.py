import numpy as np
from tables import get_health_name

class RetaskingSystem:
    def __init__(self, home_positions):
        self.home_positions = np.array(home_positions)
        self.actions = {
            "LOW_BATTERY": self._low_battery,
            "BAD_BATTERY": self._bad_battery,
            "SENSOR_FAILURE": self._sensor_fail,
            "COMM_FAILURE": self._comm_fail,
            "GPS_FAILURE": self._gps_fail,
            "MOTOR_FAILURE": self._motor_fail,
            "UNKNOWN": self._unknown,
        }

    def handle(self, drone_id: int, health_code: int):
        name = get_health_name(health_code)
        if name == "OK":
            return None
        func = self.actions.get(name, self._unknown)
        return func(drone_id, name)

    def _low_battery(self, i, _):
        #the value being returned is dictionary 
        return {"drone_id": i, "action": "RETURN_HOME",
                "message": f"Drone {i}: Low battery – preparing to return soon."}

    def _bad_battery(self, i, _):
        home = self.home_positions[i]
        return {"drone_id": i, "action": "RETURN_HOME", "target": home.tolist(),
                "message": f"Drone {i}: Bad battery – returning home {home}."}

    def _sensor_fail(self, i, _):
        return {"drone_id": i, "action": "REDUCED_ROLE",
                "message": f"Drone {i}: Sensor failure – reducing role."}

    def _comm_fail(self, i, _):
        return {"drone_id": i, "action": "LAND_NOW",
               "message": f"Drone {i}: communication failure – emergency landing!."}

    def _gps_fail(self, i, _):
        return {"drone_id": i, "action": "LAND_NOW",
                "message": f"Drone {i}: GPS failure – emergency landing!."}

    def _motor_fail(self, i, _):
        return {"drone_id": i, "action": "LAND_NOW",
                "message": f"Drone {i}: Motor failure – emergency landing!"}

    def _unknown(self, i, name):
        return {"drone_id": i, "action": "IDLE",
                "message": f"Drone {i}: Unknown status '{name}' – emergency landing!."}
