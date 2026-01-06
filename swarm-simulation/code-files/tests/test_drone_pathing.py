
import unittest
import numpy as np
from drone import Drone
# Mock env since we don't need pybullet for pathing logic tests
class MockEnv:
    def __init__(self):
        self.CTRL_FREQ = 60
        self.PYB_FREQ = 240
    def _getDroneStateVector(self, id):
        return [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0] # Mock state

class TestDronePathing(unittest.TestCase):
    def setUp(self):
        self.env = MockEnv()
        self.drone = Drone(0, self.env, [0,0,0])

    def test_add_task(self):
        path = [(1,1,1), (2,2,2)]
        self.drone.add_task(101, path)
        # Assuming internal queue implementation. 
        # Since I haven't written the code yet, I'll adapt this if the internal structure differs,
        # but the plan is to use a queue or iterator.
        self.assertTrue(hasattr(self.drone, "tasks")) 
        
    def test_standard_flight_mode(self):
        self.drone.set_flight_mode("standard")
        self.drone.position = np.array([0,0,0])
        path = [(10,0,1)]
        self.drone.add_task(1, path)
        self.drone.pop_task() # Activate first task
        
        # Check desired position
        # In standard mode, it should be a point towards the target
        target = self.drone.get_desired_position(current_time=0)
        
        # Direction should be towards (10,0,1)
        direction = target - self.drone.position
        direction = direction / np.linalg.norm(direction)
        expected_dir = np.array([1, 0, 0]) # rough check
        
        self.assertAlmostEqual(direction[0], expected_dir[0], delta=0.1)

    def test_waypoint_arrival(self):
        self.drone.position = np.array([0,0,0])
        path = [(0,0,0), (10,10,1)] # First point is current pos
        self.drone.add_task(1, path)
        self.drone.pop_task()
        
        # Should detect arrival and move to next
        # Fake time passing for hover
        self.drone.last_reach_time = 0
        current_time = 5.0 # > HOVER_TIME (assume 2s)
        
        self.drone.check_waypoint_arrival(current_time, tolerance=0.1)
        
        # Should now be targeting index 1
        self.assertEqual(self.drone.path_index, 1)

if __name__ == '__main__':
    unittest.main()
