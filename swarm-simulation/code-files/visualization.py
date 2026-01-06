
import pybullet as p
import colorsys
import numpy as np

class VisualizationManager:
    def __init__(self, num_agents, env_client, headless=False):
        self.num_agents = num_agents
        self.client = env_client
        self.headless = headless
        
        self.drone_colors = []
        self.debug_lines = [None] * num_agents
        self.section_path_lines = [[] for _ in range(num_agents)]
        
        self._generate_colors()
        
    def _generate_colors(self):
        for i in range(self.num_agents):
            hue = (i * 0.618033988749895) % 1.0 # Golden ratio
            sat = 0.8 + (i % 2) * 0.1
            val = 0.9
            r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
            self.drone_colors.append([r, g, b])
            
    def apply_drone_colors(self, drone_ids):
        if self.headless: return
        for i, drone_id in enumerate(drone_ids):
            if i < len(self.drone_colors):
                try:
                    p.changeVisualShape(drone_id, -1, rgbaColor=self.drone_colors[i] + [1], physicsClientId=self.client)
                except Exception:
                    pass

    def clear_debug_lines(self):
        for i, line_id in enumerate(self.debug_lines):
            if line_id is not None:
                try:
                    p.removeUserDebugItem(line_id, physicsClientId=self.client)
                except Exception:
                    pass
                self.debug_lines[i] = None

    def clear_full_path(self, drone_id):
        if 0 <= drone_id < len(self.section_path_lines):
            for line_id in self.section_path_lines[drone_id]:
                try:
                    p.removeUserDebugItem(line_id, physicsClientId=self.client)
                except Exception:
                    pass
            self.section_path_lines[drone_id] = []

    def draw_full_path(self, drone_id, points, enabled=True):
        if self.headless or not enabled:
            return
            
        self.clear_full_path(drone_id)
        
        if points is None or len(points) < 2:
            return

        color = self.drone_colors[drone_id]
        for k in range(len(points) - 1):
            p1 = points[k]
            p2 = points[k+1]
            try:
                line_id = p.addUserDebugLine(p1, p2, lineColorRGB=color, lineWidth=1.5, lifeTime=0, physicsClientId=self.client)
                self.section_path_lines[drone_id].append(line_id)
            except Exception:
                pass
                
    def update_waypoint_line(self, drone_id, current_pos, target_pos, show_lines=True):
        if self.headless: return
        
        if show_lines:
            line_id = self.debug_lines[drone_id]
            color = self.drone_colors[drone_id]
            try:
                if line_id is None:
                    self.debug_lines[drone_id] = p.addUserDebugLine(
                        current_pos, target_pos, lineColorRGB=color, lifeTime=0, 
                        physicsClientId=self.client
                    )
                else:
                    self.debug_lines[drone_id] = p.addUserDebugLine(
                        current_pos, target_pos, lineColorRGB=color, lifeTime=0, 
                        replaceItemUniqueId=line_id, physicsClientId=self.client
                    )
            except Exception:
                pass
        elif self.debug_lines[drone_id] is not None:
            # If toggled off but lines exist, remove them
            try:
                p.removeUserDebugItem(self.debug_lines[drone_id], physicsClientId=self.client)
            except Exception:
                pass
            self.debug_lines[drone_id] = None
