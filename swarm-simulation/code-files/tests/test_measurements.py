
import unittest
import time
from measurements import MeasurementManager

class TestMeasurementManager(unittest.TestCase):
    def setUp(self):
        self.mgr = MeasurementManager(num_agents=4)

    def test_initialization(self):
        self.assertEqual(len(self.mgr.metrics["sections_searched_per_drone"]), 4)
        self.assertFalse(self.mgr.metrics["success"])
        self.assertEqual(self.mgr.metrics["total_distance"], 0.0)

    def test_updates(self):
        self.mgr.update_distance(10.5)
        self.assertEqual(self.mgr.metrics["total_distance"], 10.5)
        
        self.mgr.increment_sections_searched(2)
        self.assertEqual(self.mgr.metrics["sections_searched_per_drone"][2], 1)
        
    def test_end_mission(self):
        self.mgr.end_mission(success=True)
        self.assertTrue(self.mgr.metrics["success"])
        self.assertIsNotNone(self.mgr.metrics["end_time"])

    def test_error_metrics(self):
        crashed = [False, True, False, False]
        health = [0, 0, 1, 0] # Agent 2 has fault
        late = {3} # Agent 3 is late
        
        self.mgr.update_error_metrics(crashed, health, late)
        self.assertEqual(self.mgr.metrics["crashed_count"], 1)
        # Agent 2 (health) and Agent 3 (late) are failures
        self.assertEqual(self.mgr.metrics["failed_count"], 2)

if __name__ == '__main__':
    unittest.main()
