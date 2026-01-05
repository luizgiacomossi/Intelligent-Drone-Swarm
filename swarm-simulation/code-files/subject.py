import numpy as np
import pybullet as p
import time
import os
import json
import random
from searchArea import SearchArea


class SubjectManager:
    def __init__(self, env, area: SearchArea, urdf_path: str, save_file="subject_location.json"):
        """
        Manages spawning a random subject object into one of the grid sections.
        Args:
            env: The active PyBullet environment (SearchAreaAviary instance)
            area: The SearchArea object (contains section positions)
            urdf_path: Path to the .urdf file for the subject to spawn
            save_file: JSON file where subject location will be stored
        """
        self.env = env
        self.area = area
        self.urdf_path = urdf_path
        self.save_file = save_file
        self.subject_id = None
        self.subject_pos = None

    def spawn_random_subject(self):
        """Spawn a subject randomly within a random section."""
        sections = self.area.sections
        if not sections:
            print("[SubjectManager] No sections found in SearchArea.")
            return None

        # Pick a random section
        chosen_section = random.choice(sections)
        center_x, center_y = chosen_section.position
        section_size = self.area.section_size

        # Random offset within 40% of the section size (to stay well inside)
        offset_x = random.uniform(-0.4, 0.4) * section_size
        offset_y = random.uniform(-0.4, 0.4) * section_size
        pos = [center_x + offset_x, center_y + offset_y, 0.05]  # slightly above ground

        # Load URDF
        if not os.path.exists(self.urdf_path):
            print(f"[SubjectManager] URDF file not found: {self.urdf_path}")
            return None

        self.subject_id = p.loadURDF(
            self.urdf_path,
            pos,
            p.getQuaternionFromEuler([0, 0, np.random.uniform(0, np.pi * 2)]),
            physicsClientId=self.env.CLIENT,
        )
        self.subject_pos = pos
        print(f"[SubjectManager] Spawned subject in section {chosen_section.id} at {pos}")

        # Save to file
        self._save_subject_location(pos, chosen_section.id)
        return pos

    def _save_subject_location(self, pos, section_id):
        """Save subject location to JSON for later retrieval."""
        data = {"position": pos, "section_id": section_id, "timestamp": time.time()}
        with open(self.save_file, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[SubjectManager] Subject location saved to {self.save_file}")

    def get_saved_subject(self):
        """Load previously saved subject location (if exists)."""
        if not os.path.exists(self.save_file):
            print("[SubjectManager] No saved subject location file found.")
            return None
        with open(self.save_file, "r") as f:
            data = json.load(f)
        return data
