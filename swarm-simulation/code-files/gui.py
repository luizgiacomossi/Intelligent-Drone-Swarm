# gui.py
import sys
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, 
    QHBoxLayout, QSpinBox, QLabel, QTextEdit, QComboBox, QCheckBox, QButtonGroup, QRadioButton
)
from PyQt5.QtCore import QTimer
import controller
from tables import get_all_health_codes

class DroneControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # --- Grid size control ---
        grid_layout = QHBoxLayout()
        grid_layout.addWidget(QLabel("Grid Size (NxN):"))
        self.grid_size_spin = QSpinBox()
        self.grid_size_spin.setRange(2, 10)
        self.grid_size_spin.setValue(4)
        self.grid_size_spin.valueChanged.connect(self.update_drone_limit)
        grid_layout.addWidget(self.grid_size_spin)
        layout.addLayout(grid_layout)

        # --- Agent count control ---
        agent_layout = QHBoxLayout()
        agent_layout.addWidget(QLabel("Number of Agents:"))
        self.drone_count = QSpinBox()
        self.drone_count.setRange(4, 16)
        self.drone_count.setValue(4)
        agent_layout.addWidget(self.drone_count)
        layout.addLayout(agent_layout)

        # --- Buttons ---
        self.sim_btn = QPushButton("Run Simulation")
        self.sim_btn.clicked.connect(self.run_simulation_with_count)
        layout.addWidget(self.sim_btn)

        self.start_btn = QPushButton("Start Search")
        self.start_btn.clicked.connect(controller.start_search)
        layout.addWidget(self.start_btn)

        self.abort_btn = QPushButton("Abort Mission!")
        self.abort_btn.clicked.connect(controller.abort_mission)
        layout.addWidget(self.abort_btn)

        # --- Text displays ---
        info_layout = QHBoxLayout()

        # Left column (broadcast)
        left_layout = QVBoxLayout()
        self.broadcast_label = QLabel("Agent Broadcast Information")
        self.broadcast_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #0080ff;")
        self.broadcast_display = QTextEdit()
        self.broadcast_display.setReadOnly(True)
        self.broadcast_display.setPlaceholderText("Agent Broadcast Information...")
        left_layout.addWidget(self.broadcast_label)
        left_layout.addWidget(self.broadcast_display)

        # Middle column: Health reports + fault injection controls
        middle_layout = QVBoxLayout()
        self.health_label = QLabel("Agent Health & Retasking")
        self.health_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #cc0000;")
        middle_layout.addWidget(self.health_label)

        # Text area for displaying alerts
        self.health_display = QTextEdit()
        self.health_display.setReadOnly(True)
        self.health_display.setPlaceholderText("Health alerts and retasking actions will appear here...")
        middle_layout.addWidget(self.health_display)

        # --- Fault Injection Controls ---
        inject_label = QLabel("Inject Agent Fault")
        inject_label.setStyleSheet("font-weight: bold; color: #444;")
        middle_layout.addWidget(inject_label)

        fault_row = QHBoxLayout()
        
        # Charge button
        self.charge_btn = QPushButton("Charge Drone")
        self.charge_btn.setEnabled(False)  # disabled by default
        self.charge_btn.setStyleSheet("background-color: gray; color: white;")
        self.charge_btn.clicked.connect(self.charge_drone)
        middle_layout.addWidget(self.charge_btn)

        # Drone selector
        self.drone_selector = QComboBox()
        self.drone_selector.setToolTip("Select which agent to inject fault into")
        self.drone_selector.addItems([str(i) for i in range(0, 100)])  # Adjust max if needed
        fault_row.addWidget(QLabel("Drone ID:"))
        fault_row.addWidget(self.drone_selector)

        # Search Are size selector
        self.searchArea_selector = QComboBox()
        self.searchArea_selector.setToolTip("Select how big the search are shall be.")
        self.searchArea_selector.addItems([str(i) for i in range(0, 9)])  # Adjust max if needed
        fault_row.addWidget(self.searchArea_selector)

        # Health selector (codes and names)
        self.health_selector = QComboBox()
        self.health_selector.setToolTip("Select fault/health condition")
        codes = get_all_health_codes()
        for code, name in codes.items():
            self.health_selector.addItem(f"{code} - {name}", code)
        fault_row.addWidget(QLabel("Health Code:"))
        fault_row.addWidget(self.health_selector)

        middle_layout.addLayout(fault_row)

        # Inject button
        self.inject_btn = QPushButton("Inject Fault")
        self.inject_btn.clicked.connect(self.send_health_status)
        middle_layout.addWidget(self.inject_btn)

        # --- Camera Control ---
        camera_layout = QHBoxLayout()
        camera_label = QLabel("Camera Follow:")
        camera_label.setStyleSheet("font-weight: bold; color: #444;")
        
        self.camera_selector = QComboBox()
        self.camera_selector.addItem("Free Cam", -1)
        self.camera_selector.addItems([f"Drone {i}" for i in range(16)]) # Pre-populate enough slots
        # Note: We should dynamically update this based on num_drones, but static is easier for now to avoid complexity in initialization order.
        
        self.set_cam_btn = QPushButton("Set Camera")
        self.set_cam_btn.clicked.connect(self.update_camera_target)
        
        # Toggle Debug Lines
        self.debug_lines_chk = QCheckBox("Show Path Lines")
        self.debug_lines_chk.setChecked(True)
        self.debug_lines_chk.toggled.connect(self.toggle_debug_lines)
        self.debug_lines_chk.setToolTip("Show red lines pointing to next waypoint")
        
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_selector)
        camera_layout.addWidget(self.set_cam_btn)
        camera_layout.addWidget(self.debug_lines_chk)
        
        
        # Flight Mode Selector
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        self.mode_selector = QComboBox()
        self.mode_selector.addItems(["Standard (Ramp)", "Aggressive (Max)", "Path Follow", "Smooth Follow"])
        
        # Sync with default config
        try:
             default_modes = ["standard", "aggressive", "path_follow", "smooth_follow"]
             if controller.flight_mode in default_modes:
                 self.mode_selector.setCurrentIndex(default_modes.index(controller.flight_mode))
        except:
             pass

        self.mode_selector.currentIndexChanged.connect(self.change_flight_mode)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_selector)
        camera_layout.addLayout(mode_layout)

        # Full Path Toggle
        self.full_path_chk = QCheckBox("Full Path")
        self.full_path_chk.setChecked(True)
        self.full_path_chk.toggled.connect(self.toggle_full_paths)
        self.full_path_chk.setToolTip("Show entire section search path")
        camera_layout.addWidget(self.full_path_chk)
        
        middle_layout.addLayout(camera_layout)

        # Right column (assignments)
        right_layout = QVBoxLayout()
        self.assignment_label = QLabel("Agent–Section Assignments")
        self.assignment_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #0080ff;")
        self.assignment_display = QTextEdit()
        self.assignment_display.setReadOnly(True)
        self.assignment_display.setPlaceholderText("Agent Section Assignments...")
        right_layout.addWidget(self.assignment_label)
        right_layout.addWidget(self.assignment_display)

        # Extra column (searched sections)
        searched_layout = QVBoxLayout()
        self.searched_label = QLabel("Searched Sections")
        self.searched_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #008000;")
        self.searched_display = QTextEdit()
        self.searched_display.setReadOnly(True)
        self.searched_display.setPlaceholderText("Searched sections will appear here...")
        searched_layout.addWidget(self.searched_label)
        searched_layout.addWidget(self.searched_display)

        # Market display
        market_layout = QVBoxLayout()
        self.market_label = QLabel("Market")
        self.market_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #aa6600;")
        self.market_display = QTextEdit()
        self.market_display.setReadOnly(True)
        self.market_display.setPlaceholderText("Market activity and ownership...")
        market_layout.addWidget(self.market_label)
        market_layout.addWidget(self.market_display)

        # Voting display (generic consensus / verification log)
        voting_layout = QVBoxLayout()
        self.voting_label = QLabel("Voting")
        self.voting_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #7733ff;")
        self.voting_display = QTextEdit()
        self.voting_display.setReadOnly(True)
        self.voting_display.setPlaceholderText("Voting events and consensus results will appear here...")
        voting_layout.addWidget(self.voting_label)
        voting_layout.addWidget(self.voting_display)

        # Add all columns
        info_layout.addLayout(left_layout)
        info_layout.addLayout(middle_layout)
        info_layout.addLayout(right_layout)
        info_layout.addLayout(searched_layout)
        info_layout.addLayout(market_layout)
        info_layout.addLayout(voting_layout)

        layout.addLayout(info_layout)

        self.setLayout(layout)
        self.setWindowTitle("Agent Swarm Control Panel")

        # --- Timer to refresh info from controller ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(1000)  # update every 1 second

        self.show()

    def run_simulation_with_count(self):
        num = self.drone_count.value()
        grid_size = self.grid_size_spin.value()
        
        # Initialize simulation structure
        controller.run_simulation(num, grid_size)
        
        # Start a fast timer for the physics loop (e.g. 60Hz or faster)
        # Note: We already have a slow timer (1s) for update_displays.
        # We need a new one for physics.
        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(controller.update_simulation)
        self.sim_timer.start(1) # Poll as fast as possible (idle priority)
        
        print(f"Starting simulation with {num} agents on a {grid_size}x{grid_size} grid...")
    
    def charge_drone(self):
        """User manually 'charges' a drone that's at home."""
        import controller
        drone_id = int(self.drone_selector.currentText())
        controller.mark_drone_charged(drone_id)
        print(f"[GUI] Drone {drone_id} charged and ready to rejoin mission.")
        self.charge_btn.setEnabled(False)
        self.charge_btn.setStyleSheet("background-color: gray; color: white;")


    def update_drone_limit(self):
        """Update the maximum number of agents based on grid size."""
        grid_size = self.grid_size_spin.value()
        max_agents = grid_size * grid_size
        self.drone_count.setMaximum(max_agents)
        self.drone_count.setMinimum(4)
        if self.drone_count.value() > max_agents:
            self.drone_count.setValue(max_agents)

    def update_displays(self):
        """Pull live text data from controller."""
        self.broadcast_display.setPlainText(controller.broadcast_text)
        self.assignment_display.setPlainText(controller.assignment_text)
        # --- Enable charge button if any drone is home waiting ---
        if hasattr(controller, "home_ready") and any(controller.home_ready):
            self.charge_btn.setEnabled(True)
            self.charge_btn.setStyleSheet("background-color: green; color: white;")
        else:
            self.charge_btn.setEnabled(False)
            self.charge_btn.setStyleSheet("background-color: gray; color: white;")

        if hasattr(controller, "searched_text"):
            self.searched_display.setPlainText(controller.searched_text)

        if hasattr(controller, "market_text"):
            self.market_display.setPlainText(controller.market_text)

        if hasattr(controller, "voting_text"):
            self.voting_display.setPlainText(controller.voting_text)

        # Show latest health/retasking info
        if hasattr(controller, "middle_text"):
            current = self.health_display.toPlainText()
            new_msg = controller.middle_text
            if new_msg and (new_msg not in current):  # prevent duplicates
                self.health_display.append(new_msg)


    def send_health_status(self):
        """Send a fault injection command to the controller (and thus to test.py)."""
        import controller

        agent_id = int(self.drone_selector.currentText())
        # Get the selected code from the data role
        health_code = self.health_selector.currentData()
        controller.injected_fault = (agent_id, health_code)
        name = self.health_selector.currentText()
        print(f"[GUI] Injected fault {name} for Agent {agent_id}")
        self.health_display.append(f"[GUI] Injected fault {name} for Agent {agent_id}")

    def update_camera_target(self):
        import controller
        # Extract ID from text "Drone X" or use user data if I set it cleanly. 
        # I set "Free Cam" as -1 userData. For "Drone X", I didn't set userData loop.
        # Let's trust the text parsing or index.
        text = self.camera_selector.currentText()
        if "Free" in text:
            target_id = -1
        else:
            try:
                target_id = int(text.split(" ")[1])
            except:
                target_id = -1
        
        controller.set_camera_target(target_id)

    def toggle_debug_lines(self):
        import controller
        checked = self.debug_lines_chk.isChecked()
        controller.toggle_debug_lines(checked)

    def change_flight_mode(self, index):
        import controller
        modes = ["standard", "aggressive", "path_follow", "smooth_follow"]
        if 0 <= index < len(modes):
            controller.set_flight_mode(modes[index])

    def toggle_full_paths(self):
        import controller
        checked = self.full_path_chk.isChecked()
        controller.toggle_full_paths(checked)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = DroneControlUI()
    sys.exit(app.exec_())
