import time
import numpy as np
import pybullet as p
import controller
import sys
from env import SearchAreaAviary
from searchArea import SearchArea
from drone import Drone
from avoidance import avoidance_from_drones
from avoidance import avoidance_from_borders
from avoidance import get_all_positions
from retasking import RetaskingSystem
from tables import get_health_name
from market import MarketSystem
from subject import SubjectManager
from PyQt5.QtWidgets import QMessageBox, QApplication

# Constants
from config import (
    HOME_POSITION, FLY_HEIGHT, SECTION_SWEEP_STEPS, SEARCH_OFFSET,
    WAYPOINT_TOLERANCE, HOVER_TIME, MAX_SPEED, BROADCAST_PERIOD,
    RETURN_TIMEOUT, CTRL_FREQ, SECTION_SIZE, 
    CRASH_HEIGHT_THRESHOLD, CRASH_TIMEOUT, BATTERY_CHANGE_DURATION,
    SUBJECT_DETECTION_DIST, VERIFICATION_DIST, VOTING_RADIUS, VOTING_ANGLES,
    MAX_FRAME_TIME, VERIFICATION_SPEED_FACTOR,
    AVOIDANCE_FACTOR, AVOID_DRONE_RADIUS, AVOID_DRONE_MAX_PUSH, AVOID_DRONE_GAIN_NAV, AVOID_DRONE_GAIN_VERIFY,
    AVOID_BORDER_GAIN, AVOID_BORDER_MAX_PUSH, AVOID_BORDER_MARGIN_NAV, AVOID_BORDER_MARGIN_VERIFY,
    NAV_STD_SPEED_DIVISOR, NAV_STD_SPEED_MIN, NAV_STD_SPEED_MAX, NAV_AGGRESSIVE_SPEED_SCALE,
    NAV_GLOBAL_STEP_MULTIPLIER, RETURN_HOME_DIST_THRESHOLD, RETURN_HOME_SPEED_DIVISOR,
    RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX, RETURN_HOME_STEP_MULTIPLIER
)
from utils import (
    generate_drone_positions, generate_lawnmower_points,
    rebuild_tasks_from_market
)
from measurements import MeasurementManager
from visualization import VisualizationManager

class SimulationManager:
    def __init__(self, num_agents=4, grid_size=4, headless=False):
        self.num_agents = num_agents
        self.grid_size = grid_size
        self.ctrl_freq = CTRL_FREQ
        self.headless = headless
        
        # Managers
        self.measurements = MeasurementManager(num_agents)
        self.measurements.start_mission()
        
        print("Initializing environment...")
        self.initial_xyz_agent, self.formation_radius = generate_drone_positions(num_agents, HOME_POSITION)

        offset_x = 4 + 1.0 * self.formation_radius + 0.75 * grid_size 
        offset_y = 2 + 0.5 * self.formation_radius + 0.5 * grid_size
        self.search_offset = (offset_x, offset_y)

        self.env = SearchAreaAviary(
            num_drones=num_agents,
            initial_xyzs=self.initial_xyz_agent,
            gui=not headless,
            user_debug_gui=False,
            grid_size=(grid_size, grid_size),
            section_size=SECTION_SIZE,
            home_position=(0, 0),
            search_area_offset=(offset_x, offset_y),
            helipad_radius=self.formation_radius 
        )

        self.visualizer = VisualizationManager(num_agents, self.env.CLIENT, headless=headless)
        self.charged_complete = [False] * num_agents
        self.area = SearchArea(grid_size=(grid_size, grid_size), section_size=SECTION_SIZE, home=self.search_offset)

        self.subject_mgr = SubjectManager(self.env, self.area, urdf_path=r"/Users/lgr03/Documents/MDU_PhD/Papers/Intelligent Replanning/Intelligent-Drone-Swarm/swarm-simulation/objects/human_urdf/unnamed/urdf/unnamed.urdf")
        self.subject_pos = self.subject_mgr.spawn_random_subject()
        
        self.subject_section_id = min(
            range(len(self.area.sections)),
            key=lambda sid: np.linalg.norm(
                np.array(self.area.sections[sid].position) - np.array(self.subject_pos[:2])
            )
        )

        self.subject_found = False
        self.voting_active = False
        self.detecting_drone_id = None
        self.verification_targets = {}
        self.votes = []
        self.helpers = []

        self.return_reason = [""] * num_agents
        self.battery_return_start = {}
        self.battery_late = set()

        self.swarm = [Drone(i, self.env, self.initial_xyz_agent[i]) for i in range(num_agents)]
        self.market = MarketSystem(num_agents, self.area)
        self.market_initialized = False
        self.drone_tasks = {i: iter([]) for i in range(num_agents)}
        self.return_timer = [0.0] * num_agents
        self.return_active = [False] * num_agents

        print("Mission started. Drones searching assigned sections...")
        self.current_targets = [None] * num_agents
        self.path_progress = [0] * num_agents
        self.last_reach_time = [0.0] * num_agents
        self.current_section = [None] * num_agents
        self.last_broadcast = [0.0] * num_agents
        self.last_positions = np.array([d.position for d in self.swarm]) # For distance calc

        self.returning_home = False
        self.home_targets, _ = generate_drone_positions(num_agents, HOME_POSITION)

        self.crashed = [False] * num_agents
        self.crash_timer = [0.0] * num_agents

        # Drone colors managed by VisualizationManager
        self.visualizer.apply_drone_colors(self.env.DRONE_IDS if not self.headless else [])

        self.retasker = RetaskingSystem(self.home_targets)
        self.health_status = [0] * num_agents

        if not hasattr(controller, "injected_fault"):
            controller.injected_fault = None
            
        self.descent_timer = 0
        self.is_descending = False
        self.mission_complete = False
        
        # Camera tracking
        self.camera_target_id = None

    # =========================================================================
    # PUBLIC INTERFACE
    # =========================================================================

    def set_camera_target(self, drone_id):
        self.camera_target_id = drone_id

    @property
    def metrics(self):
        """Backward compatibility for external scripts accessing metrics directly."""
        return self.measurements.metrics

    def step(self):
        """
        Orchestrates a single simulation step.
        Returns: True if simulation should continue, False otherwise.
        """
        # 1. Validation & Early Exits
        if self.mission_complete:
            self._handle_mission_completion()
            return False
        
        if not self._is_env_valid():
            return False

        # 2. Global State & Inputs
        t = self.env.step_counter / self.env.PYB_FREQ
        self._handle_injected_faults()
        self._check_mission_abort()
        
        # 3. Critical Failure Checks
        if self._check_critical_swarm_failure():
            return False

        # 4. Paused State Handling
        if not self._handle_pause_state():
            return True # Paused, but keep sim running

        # 5. Market & Task Management
        self._manage_market_and_penalties(t)

        # 6. Swarm Update Loop
        actions = np.zeros((self.num_agents, 4))
        all_positions = get_all_positions(self.env, self.num_agents) 
        
        # Determine if we are in a global descent phase
        if self.returning_home and self._check_ready_for_descent():
             self.is_descending = True

        if self.is_descending:
            actions = self._handle_global_descent()
        else:
            # Standard Drone Updates
            for i, drone in enumerate(self.swarm):
                actions[i, :] = self._update_single_drone(i, drone, t, all_positions)

        # 7. GUI & Visualization Updates
        self._update_gui_and_camera()

        # 8. Physics Step
        self.env.step(actions)
        
        # 9. Post-Step Logic (Mission Success Check)
        if not self.is_descending and not self.voting_active and not self.returning_home:
             self._check_mission_success_condition()

        return True

    def show_summary(self):
        subject_section = None
        for sec in self.area.sections:
            if np.linalg.norm(np.array(sec.position) - np.array(self.subject_pos[:2])) < 1.0: 
                 subject_section = sec
                 break

        summary = self.measurements.get_metrics_summary()
        summary_msg = f"""
        Position: {np.round(self.subject_pos[:2], 2)}
        Section: {subject_section.id if subject_section else 'Unknown'}
        Time to find: {summary['duration']:.2f} seconds
        """
        if not self.headless:
            try:
                 app = QApplication.instance()
                 if app is None:
                     app = QApplication(sys.argv)
                 msg_box = QMessageBox()
                 msg_box.setWindowTitle("Mission Summary")
                 msg_box.setText(summary_msg)
                 msg_box.setStandardButtons(QMessageBox.Ok)
                 msg_box.exec_()
            except Exception as e:
                 print(f"[GUI] Could not open summary window: {e}")
        else:
            print(f"[Summary] Mission Complete. Time to find: {summary['duration']:.2f}s")

    def close(self):
        if hasattr(self, 'env'):
             try:
                 p.disconnect(physicsClientId=self.env.CLIENT)
             except:
                 pass
             self.env.CLIENT = -1

    # =========================================================================
    # GLOBAL STATE & HELPERS
    # =========================================================================

    def _is_env_valid(self):
        return hasattr(self, 'env') and self.env.CLIENT >= 0

    def _handle_mission_completion(self):
        self.measurements.update_error_metrics(self.crashed, self.health_status, self.battery_late)

    def _check_mission_abort(self):
        if controller.mission_aborted:
            self.returning_home = True
            controller.mission_aborted = False
            print("GUI: Mission abort detected, returning home!")

    def _handle_pause_state(self):
        if not controller.search_active and not self.returning_home:
            if self.headless:
                controller.search_active = True
                return True
            else:
                return False
        return True

    def _check_critical_swarm_failure(self):
        # 1. All crashed
        if all(self.crashed) and not self.is_descending and not self.measurements.metrics.get("success", False):
            print("All drones crashed! Ending simulation.")
            self.measurements.end_mission(success=False, reason="All Drones Crashed")
            self._handle_mission_completion()
            return True
            
        # 2. Incapacitated (crashed or grounded)
        drones_incapacitated = []
        for i in range(self.num_agents):
            is_down = self.crashed[i] or (i < len(controller.home_ready) and controller.home_ready[i])
            drones_incapacitated.append(is_down)
        
        if all(drones_incapacitated) and not self.mission_complete:
            print("All drones are crashed or grounded. Mission failed (Swarm Depleted).")
            self.measurements.end_mission(success=False, reason="Swarm Depleted")
            self._handle_mission_completion()
            return True
            
        return False

    def _handle_injected_faults(self):
        if controller.injected_fault:
            fault_drone, fault_code = controller.injected_fault
            self.health_status[fault_drone] = fault_code
            controller.injected_fault = None
            fault_name = get_health_name(fault_code)
            print(f"[GUI Inject] agent {fault_drone} fault set to {fault_name}")

    def _initialize_market(self):
        drone_positions = [drone.position for drone in self.swarm]
        self.market.open_market(drone_positions)
        controller.market_text = self.market.get_market_status()
        rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)
        self.market_initialized = True

    def _manage_market_and_penalties(self, t):
        if not self.market_initialized:
            self._initialize_market()
            
        for j in range(self.num_agents):
            if self.return_reason[j] == "battery" and not self.charged_complete[j]:
                if j in self.battery_return_start and (t - self.battery_return_start[j]) > BATTERY_CHANGE_DURATION:
                    if j not in self.battery_late:
                        self._apply_battery_penalty(j, t)

    def _apply_battery_penalty(self, drone_id, t):
        print(f"[Battery Timeout] Drone {drone_id} took too long. Releasing sections.")
        self.battery_late.add(drone_id)
        self.market.release_drone_sections(drone_id, current_time=t)
        self._refresh_market_tasks(t)

    def _refresh_market_tasks(self, t):
        controller.market_text = self.market.get_market_status()
        drone_positions = [d.position for d in self.swarm]
        self.market.dynamic_update(drone_positions, current_time=t)
        controller.market_text = self.market.get_market_status()
        rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)

    def _check_ready_for_descent(self):
        """Returns True if all functional drones are close to home."""
        if self.is_descending: return False
        
        distances = []
        for i in range(self.num_agents):
             if self.crashed[i]: continue
             pos = np.array(self.env._getDroneStateVector(i)[0:3])
             dist = np.linalg.norm(pos[:2] - self.home_targets[i][:2])
             distances.append(dist)
        
        if len(distances) == 0 or all(d < 1.8 for d in distances):
             print("All drones are returning home. Starting descent.")
             return True
        return False

    def _check_mission_success_condition(self):
         if all(c.searched for c in self.area.sections):
             print("The search area is searched! Drones will return home together...")
             self.returning_home = True

    # =========================================================================
    # SINGLE AGENT LOOP
    # =========================================================================

    def _update_single_drone(self, i, drone, t, all_positions):
        """Calculates the RPM actions for a single drone."""
        
        # 1. Update State & Metrics
        state = drone.update()
        current_pos = np.array(state[0:3])
        self._update_drone_metrics(i, current_pos)

        # 2. Check Recharged Status
        if i in controller.charged_drones:
            self._handle_recharged_drone(i, t)
            # Hover momentarily after reset
            return drone.step_toward(np.array([current_pos[0], current_pos[1], FLY_HEIGHT]))

        # 3. Handle Faults
        if self.health_status[i] != 0 and not self.returning_home:
            fault_action = self._process_fault_logic(i, drone, current_pos, t)
            if fault_action is not None:
                return fault_action

        # 4. Crash Handling
        if self.crashed[i]: 
            return np.zeros(4) 
        if self._check_crash_condition(i, current_pos, t):
             # Just crashed this frame
             self.market.release_drone_sections(i)
             self._refresh_market_tasks(t)
             return np.zeros(4)

        # 5. Periodic Broadcast
        if t - self.last_broadcast[i] >= BROADCAST_PERIOD:
            drone.broadcast()
            self.last_broadcast[i] = t

        # 6. Global Return Home
        if self.returning_home:
            return self._calculate_return_home_action(i, drone, current_pos)

        # 7. Subject Detection
        if self._should_check_for_subject(i):
             self._check_for_subject(i, drone, current_pos)

        # 8. Voting
        if self.voting_active:
            action = self._handle_voting_behavior(i, drone, current_pos, t, all_positions)
            if action is not None: return action

        # 9. Standard Search Navigation
        return self._execute_search_navigation(i, drone, current_pos, t, all_positions)

    def _update_drone_metrics(self, i, current_pos):
        dist = np.linalg.norm(current_pos - self.last_positions[i])
        self.measurements.update_distance(dist)
        self.last_positions[i] = current_pos

    def _handle_recharged_drone(self, i, t):
        print(f"[MAIN] Drone {i} recharged — rejoining mission.")
        controller.charged_drones.remove(i)
        self.health_status[i] = 0
        controller.home_ready[i] = False
        self.charged_complete[i] = True
        self.battery_return_start.pop(i, None)
        self.return_reason[i] = ""

        if i in self.battery_late:
            print(f"[Penalty] Drone {i} took too long to charge. Must buy section (cost 2 points).")
            self.market.force_buy_section(i, cost=2)
            self.battery_late.remove(i)
        
        self._refresh_market_tasks(t)

    # =========================================================================
    # FAULT & CRASH HANDLING
    # =========================================================================

    def _check_crash_condition(self, i, current_pos, t):
        if current_pos[2] <= CRASH_HEIGHT_THRESHOLD:
            if self.crash_timer[i] == 0.0:
                self.crash_timer[i] = t
            elif t - self.crash_timer[i] > CRASH_TIMEOUT:
                self.crashed[i] = True
                print(f"Drone {i} has crashed! Altitude={current_pos[2]:.2f}")
                p.addUserDebugText("CRASHED", [current_pos[0], current_pos[1], 0.1], 
                                   textColorRGB=[1, 0, 0], textSize=2, lifeTime=0, 
                                   physicsClientId=self.env.CLIENT)
                return True
        else:
            self.crash_timer[i] = 0.0
        return False

    def _process_fault_logic(self, i, drone, current_pos, t):
        decision = self.retasker.handle(i, self.health_status[i])
        if not decision: return None
        
        action = decision["action"]
        
        if action == "RETURN_HOME":
            return self._navigate_fault_return(i, drone, current_pos, t)
        elif action == "LAND_NOW":
            return self._emergency_land(i, drone, current_pos, t)
        elif action in ("HOVER", "REDUCED_ROLE", "HOVER_AND_RECONNECT"):
            return self._hover_relay(i, drone, current_pos, t)
        return None

    def _navigate_fault_return(self, i, drone, current_pos, t):
        target_pos = np.array(self.home_targets[i])
        diff = target_pos - current_pos
        dist = np.linalg.norm(diff)
        
        if dist > 0.05:
            next_pos = current_pos + 0.3 * diff
        else:
            next_pos = target_pos
            
        if dist < 0.2:
            controller.home_ready[i] = True
        else:
            controller.home_ready[i] = False
            
        if not self.return_active[i]:
            self.return_active[i] = True
            self.return_timer[i] = t
            self.charged_complete[i] = False
            self.return_reason[i] = "battery"
            self.battery_return_start[i] = t
            print(f"Agent {i} returning home for battery change.")
            
        return drone.step_toward(next_pos)

    def _emergency_land(self, i, drone, current_pos, t):
        target_pos = np.array([current_pos[0], current_pos[1], 0.05])
        next_pos = current_pos + 0.2 * (target_pos - current_pos)
        
        # Only release once
        if not self.crashed[i]: # Use crashed flag or separate flag to know if handled
             self.market.release_drone_sections(i)
             self._refresh_market_tasks(t)
             controller.middle_text = f"Agent {i} emergency landed — sections returned."
             
        return drone.step_toward(next_pos)

    def _hover_relay(self, i, drone, current_pos, t):
        target_pos = np.array([current_pos[0], current_pos[1], 2])
        next_pos = current_pos + 0.2 * (target_pos - current_pos)
        
        self.market.release_drone_sections(i)
        self._refresh_market_tasks(t)
        controller.middle_text = f"Agent {i} is being used as a relay, sections returned."
        
        return drone.step_toward(next_pos)

    # =========================================================================
    # SUBJECT & VOTING
    # =========================================================================

    def _should_check_for_subject(self, i):
        return (not self.subject_found and not self.voting_active 
                and not self.returning_home and controller.search_active 
                and self.current_section[i] is not None 
                and self.subject_pos is not None)

    def _check_for_subject(self, i, drone, current_pos):
        drone_xy = np.array(current_pos[:2])
        subject_xy = np.array(self.subject_pos[:2])
        dist_to_subject = np.linalg.norm(drone_xy - subject_xy)

        is_assigned = (self.current_section[i] == self.subject_section_id and
                       self.area.sections[self.subject_section_id].assigned_drone == i)

        if is_assigned and dist_to_subject < SUBJECT_DETECTION_DIST:
            self._trigger_voting(i)

    def _trigger_voting(self, detecting_id):
        self.subject_found = True
        self.detecting_drone_id = detecting_id
        self.swarm[detecting_id].subject_found = 1
        
        controller.middle_text = f"Drone {detecting_id} detected subject at {np.round(self.subject_pos[:2], 2)}!"
        print(controller.middle_text)

        # Select helpers
        dists = [
            (j, np.linalg.norm(np.array(self.swarm[j].position[:2]) - np.array(self.subject_pos[:2])))
            for j in range(self.num_agents) if j != detecting_id
        ]
        dists.sort(key=lambda x: x[1])
        self.helpers = [idx for idx, _ in dists[:3]]
        print(f"[Subject] Verifiers assigned: {self.helpers}")

        controller.voting_text = (
            "Subject verification started\n"
            f"Candidate position: {np.round(self.subject_pos[:2], 2)}\n"
            f"Verifiers: {self.helpers}\n"
        )

        self.verification_targets = {}
        angles = VOTING_ANGLES
        for k, drone_id in enumerate(self.helpers):
            offx = np.cos(np.radians(angles[k])) * VOTING_RADIUS
            offy = np.sin(np.radians(angles[k])) * VOTING_RADIUS
            self.verification_targets[drone_id] = np.array([self.subject_pos[0] + offx,
                                                            self.subject_pos[1] + offy,
                                                            FLY_HEIGHT])
        
        self.visualizer.draw_verification_targets(self.verification_targets)
        self.voting_active = True
        self.votes = []

    def _handle_voting_behavior(self, i, drone, current_pos, t, all_positions):
        # 1. Check for crashed verifiers
        crashed_verifiers = [j for j in self.verification_targets.keys() if self.crashed[j]]
        if crashed_verifiers:
             for cv in crashed_verifiers: del self.verification_targets[cv]
             if not self.verification_targets:
                 self._abort_voting()
                 return None

        # 2. Logic if I am a verifier
        if i in self.verification_targets:
            target = self.verification_targets[i]
            diff_xy = target[:2] - current_pos[:2]
            dist = np.linalg.norm(diff_xy)
            
            if dist > VERIFICATION_DIST:
                direction = diff_xy / (dist + 1e-6)
                move_dist = min(dist, MAX_SPEED / self.ctrl_freq * VERIFICATION_SPEED_FACTOR)
                next_xy = current_pos[:2] + direction * move_dist
                next_pos = np.array([next_xy[0], next_xy[1], FLY_HEIGHT])
                return drone.step_toward(next_pos)
            else:
                if i not in [v["drone_id"] for v in self.votes]:
                    self.votes.append({"drone_id": i, "vote": "YES"})
                    msg = f"Drone {i} votes YES at {np.round(current_pos[:2], 2)}"
                    print(msg)
                    controller.voting_text += msg + "\n"
                    
                self._check_voting_completion()
                return drone.step_toward(np.array([current_pos[0], current_pos[1], FLY_HEIGHT]))

        # 3. Logic if I am NOT a verifier (avoidance)
        if self.current_targets[i] is not None:
             return self._execute_search_navigation(i, drone, current_pos, t, all_positions)
        
        return drone.step_toward(np.array([current_pos[0], current_pos[1], FLY_HEIGHT]))

    def _abort_voting(self):
         print("[Subject] Voting aborted (verifiers crashed).")
         self.voting_active = False
         self.votes = []
         self.subject_found = False
         self.visualizer.clear_verification_targets()
         controller.voting_text = "Voting aborted."

    def _check_voting_completion(self):
        if len(self.votes) >= len(self.verification_targets):
            print("[Subject] Voting complete!")
            controller.middle_text = "✅ Subject confirmed! All drones returning home."
            controller.voting_text += "✅ Confirmed — returning home.\n"
            
            total_time = round(time.time() - self.measurements.metrics["start_time"], 1)
            controller.middle_text += f"\nSubject confirmed at {np.round(self.subject_pos[:2], 2)} | Time: {total_time}s"
            
            # Map Coverage
            total_sections = self.grid_size * self.grid_size
            searched_count = sum(1 for s in self.market.sections if s["searched"])
            coverage_pct = (searched_count / total_sections) * 100.0
            self.measurements.record_map_coverage(coverage_pct)
            
            self.measurements.end_mission(success=True)
            self.measurements.merge_market_metrics(self.market.get_metrics())
            summary = self.measurements.get_metrics_summary()
            print(f"Metrics: Success! Time={summary['duration']:.2f}s | Coverage={summary['map_coverage']:.1f}%")
            
            if self.headless:
                self.mission_complete = True
                return

            self.returning_home = True
            self.voting_active = False
            self.visualizer.clear_verification_targets()
            
            # Highlight Found Section
            min_dist = float("inf")
            subject_section = None
            for sec in self.area.sections:
                dist_to_section = np.linalg.norm(np.array(sec.position) - np.array(self.subject_pos[:2]))
                if dist_to_section < min_dist:
                    min_dist = dist_to_section
                    subject_section = sec
            if subject_section:
                self.env.mark_section_as_searched(subject_section.position, color=(1, 0.84, 0))

    # =========================================================================
    # NAVIGATION
    # =========================================================================

    def _calculate_return_home_action(self, i, drone, current_pos):
        target_pos = np.array([self.home_targets[i][0], self.home_targets[i][1], FLY_HEIGHT])
        diff = target_pos - current_pos
        dist = np.linalg.norm(diff)
        
        if dist > RETURN_HOME_DIST_THRESHOLD:
            direction = diff / (dist + 1e-6)
            speed_scale = np.clip(dist / RETURN_HOME_SPEED_DIVISOR, RETURN_HOME_SPEED_MIN, RETURN_HOME_SPEED_MAX)
            move_dist = min(dist, MAX_SPEED * speed_scale / self.ctrl_freq * RETURN_HOME_STEP_MULTIPLIER)
            next_pos = current_pos + direction * move_dist
        else:
            next_pos = target_pos
            
        next_pos[2] = FLY_HEIGHT
        return drone.step_toward(next_pos)

    def _execute_search_navigation(self, i, drone, current_pos, t, all_positions):
        # 1. Fetch Task
        if self.current_targets[i] is None:
            try:
                cell_id, path = next(self.drone_tasks[i])
                self.current_targets[i] = path
                self.path_progress[i] = 0
                self.current_section[i] = cell_id
                self.last_reach_time[i] = t
                print(f"Drone {i} → new section {cell_id}")
                
                # Full Path Vis
                if controller.show_full_paths:
                    self.visualizer.draw_full_path(i, path, enabled=True)
            except StopIteration:
                # Hover if no tasks
                return drone.step_toward(np.array([current_pos[0], current_pos[1], FLY_HEIGHT]))

        target_pos = self.current_targets[i][self.path_progress[i]]

        # 2. Visuals
        if not self.headless:
            if self.env.step_counter % 10 == 0:
                 self.visualizer.update_waypoint_line(i, current_pos, target_pos, show_lines=controller.show_debug_lines)
            elif not controller.show_debug_lines:
                 self.visualizer.clear_debug_lines()

        # 3. Physics & Avoidance
        diff = np.array(target_pos) - current_pos
        dist = np.linalg.norm(diff)
        
        speed_scale = 1.0
        if controller.flight_mode == "standard":
             speed_scale = np.clip(dist / NAV_STD_SPEED_DIVISOR, NAV_STD_SPEED_MIN, NAV_STD_SPEED_MAX)
        elif controller.flight_mode == "aggressive":
             speed_scale = NAV_AGGRESSIVE_SPEED_SCALE

        move_dist = min(dist, MAX_SPEED * speed_scale / self.ctrl_freq * NAV_GLOBAL_STEP_MULTIPLIER)
        direction = diff / (dist + 1e-6)
        next_pos = current_pos + direction * move_dist

        push_drones = avoidance_from_drones(current_pos, all_positions, i, 
                                            radius=AVOID_DRONE_RADIUS, 
                                            gain=AVOID_DRONE_GAIN_NAV, 
                                            max_push=AVOID_DRONE_MAX_PUSH)
        push_border = avoidance_from_borders(current_pos, (self.grid_size, self.grid_size), 
                                             SECTION_SIZE, self.search_offset, 
                                             margin=AVOID_BORDER_MARGIN_NAV, 
                                             gain=AVOID_BORDER_GAIN, 
                                             max_push=AVOID_BORDER_MAX_PUSH)
        
        next_pos += AVOIDANCE_FACTOR * (push_drones + push_border)
        
        # Reset internal PID occasionally
        if int(t) % 60 == 0: drone.controller.reset()
        
        next_pos[2] = FLY_HEIGHT
        action = drone.step_toward(next_pos)

        # 4. Waypoint Logic
        if dist < WAYPOINT_TOLERANCE:
            if t - self.last_reach_time[i] > HOVER_TIME:
                self.path_progress[i] += 1
                self.last_reach_time[i] = t
                if self.path_progress[i] >= len(self.current_targets[i]):
                    self._complete_section(i, t)
                    
        return action

    def _complete_section(self, i, t):
        if self.current_section[i] is not None:
            # Mark done
            self.area.mark_searched(self.current_section[i])
            cell_center = self.area.sections[self.current_section[i]].position
            self.env.mark_section_as_searched(cell_center)
            print(f"Drone {i} finished section {self.current_section[i]}")
            
            # Market Update
            self.market.reward_and_remove_section(i, self.current_section[i])
            self.measurements.increment_sections_searched(i)
            
            self._refresh_market_tasks(t)
            
            # Reset
            self.visualizer.clear_full_path(i)
            self.current_targets[i] = None
            self.path_progress[i] = 0
            self.last_reach_time[i] = t

    def _handle_global_descent(self):
        actions = np.zeros((self.num_agents, 4))
        self.descent_timer += 1
        
        if self.descent_timer < self.ctrl_freq * 30: # 30s timeout
            for i, drone in enumerate(self.swarm):
                if self.crashed[i]: continue
                state = drone.update()
                current_pos = np.array(state[0:3])
                target_pos = np.array([self.home_targets[i][0], self.home_targets[i][1], 0.05])
                next_pos = current_pos + 0.2 * (target_pos - current_pos)
                actions[i, :] = drone.step_toward(next_pos)
        else:
            self._finalize_landing()
            
        return actions

    def _finalize_landing(self):
        for i, drone in enumerate(self.swarm):
            if self.crashed[i]: continue
            p.resetBasePositionAndOrientation(
                self.env.DRONE_IDS[i],
                [self.home_targets[i][0], self.home_targets[i][1], 0.1],
                [0, 0, 0, 1],
                physicsClientId=self.env.CLIENT
            )
            p.resetBaseVelocity(self.env.DRONE_IDS[i], [0,0,0], [0,0,0], physicsClientId=self.env.CLIENT)
        
        print("All drones landed and motors shut down. Mission complete!")
        controller.search_active = False
        controller.simulation_running = False
        self.mission_complete = True
        self.show_summary()

    def _update_gui_and_camera(self):
        # 1. Update Broadcast Text
        broadcast_lines = []
        for i, drone in enumerate(self.swarm):
            msg = drone.broadcast()
            broadcast_lines.append(f"Agent {msg['ID']}: Pos={tuple(np.round(msg['Pos'],2))}, Time={msg['Timer']}s")
        controller.broadcast_text = "\n".join(broadcast_lines)

        # 2. Update Assignment Text
        assign_lines = []
        for i in range(self.num_agents):
            sections = [c.id for c in self.area.sections if c.assigned_drone == i]
            assign_lines.append(f"Drone {i}: Sections {sections}")
        controller.assignment_text = "\n".join(assign_lines)

        # 3. Update Searched Text
        searched_lines = []
        for c in self.area.sections:
            status = "✅ Done" if c.searched else "🔲 Searching"
            searched_lines.append(f"Section {c.id}: {status}")
        controller.searched_text = "\n".join(searched_lines)
        
        # 4. Camera Tracking
        if self.camera_target_id is not None:
             if 0 <= self.camera_target_id < self.num_agents:
                 try:
                     pos, _ = p.getBasePositionAndOrientation(self.env.DRONE_IDS[self.camera_target_id], physicsClientId=self.env.CLIENT)
                     p.resetDebugVisualizerCamera(
                         cameraDistance=1.5,
                         cameraYaw=-90,
                         cameraPitch=-40,
                         cameraTargetPosition=[pos[0], pos[1], pos[2]]
                     )
                 except Exception:
                     pass

def main():
    pass