import numpy as np
from typing import List, Tuple
from config import LAWNMOWER_MARGIN_FACTOR, FLY_HEIGHT

class Guidance:
    """
    Handles path generation and guidance logic for drones.
    """

    @staticmethod
    def generate_lawnmower_points(center: Tuple[float, float], section_size: float, steps: int) -> List[Tuple[float, float, float]]:
        """
        Generates a lawnmower pattern with intermediate waypoints along scan lines.

        Args:
            center (Tuple[float, float]): (x, y) center of the section.
            section_size (float): Width/Height of the square section.
            steps (int): Number of horizontal scan lines (rows).

        Returns:
            List[Tuple[float, float, float]]: List of waypoints (x, y, z).
        """
        cx, cy = center
        half = section_size / 2
        margin = LAWNMOWER_MARGIN_FACTOR * section_size   # a margin so that the drones cover most of the section without hitting borders.
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

    @staticmethod
    def generate_lawnmower_points_new(center: Tuple[float, float], section_size: float, steps: int, samples_per_line: int = 5) -> List[Tuple[float, float, float]]:
        """
        Generates a lawnmower pattern with intermediate waypoints along scan lines.

        Args:
            center (Tuple[float, float]): (x, y) center of the section.
            section_size (float): Width/Height of the square section.
            steps (int): Number of horizontal scan lines (rows).
            samples_per_line (int): Number of points to generate per horizontal line.

        Returns:
            List[Tuple[float, float, float]]: List of waypoints (x, y, z).
        """
        cx, cy = center
        half = section_size / 2
        margin = LAWNMOWER_MARGIN_FACTOR * section_size 
        
        # Define horizontal and vertical limits
        x1, x2 = cx - half + margin, cx + half - margin
        y1, y2 = cy - half + margin, cy + half - margin
        
        # Generate Y coordinates (scan lines)
        ys = np.linspace(y1, y2, steps)
        pts = []
        
        flip = False 

        for y in ys:
            if not flip:
                # Generate from left to right (x1 -> x2)
                xs = np.linspace(x1, x2, samples_per_line)
            else:
                # Generate from right to left (x2 -> x1)
                xs = np.linspace(x2, x1, samples_per_line)
                
            # Combine generated Xs with current Y and flight height
            row_points = [(x, y, FLY_HEIGHT) for x in xs]
            pts.extend(row_points)
            
            flip = not flip
            
        return pts
