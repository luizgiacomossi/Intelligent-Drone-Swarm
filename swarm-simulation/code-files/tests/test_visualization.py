
import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock pybullet before importing visualization
sys.modules['pybullet'] = MagicMock()
from visualization import VisualizationManager

class TestVisualizationManager(unittest.TestCase):
    def setUp(self):
        self.mock_client = 1
        self.vis = VisualizationManager(num_agents=3, env_client=self.mock_client, headless=False)
        
    def test_colors_generated(self):
        self.assertEqual(len(self.vis.drone_colors), 3)
        # Check integrity of color (r,g,b)
        self.assertEqual(len(self.vis.drone_colors[0]), 3)

    @patch('pybullet.changeVisualShape')
    def test_apply_colors(self, mock_change_shape):
        drone_ids = [10, 11, 12]
        self.vis.apply_drone_colors(drone_ids)
        self.assertEqual(mock_change_shape.call_count, 3)

    @patch('pybullet.addUserDebugLine')
    def test_draw_path(self, mock_add_line):
        points = [(0,0,1), (1,1,1), (2,2,1)]
        self.vis.draw_full_path(0, points, enabled=True)
        # Should draw 2 lines for 3 points
        self.assertEqual(mock_add_line.call_count, 2)
        
    @patch('pybullet.removeUserDebugItem')
    def test_clear_lines(self, mock_remove):
        # Fake some lines
        self.vis.section_path_lines[0] = [99, 100]
        self.vis.clear_full_path(0)
        self.assertEqual(mock_remove.call_count, 2)
        self.assertEqual(self.vis.section_path_lines[0], [])

if __name__ == '__main__':
    unittest.main()
