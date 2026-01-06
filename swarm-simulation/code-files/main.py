
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
    RETURN_TIMEOUT
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
        self.ctrl_freq = 60
        self.headless = headless
        
        # Metrics
        
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
            section_size=1.5,
            home_position=(0, 0),
            search_area_offset=(offset_x, offset_y),
            helipad_radius=self.formation_radius 
        )

        self.visualizer = VisualizationManager(num_agents, self.env.CLIENT, headless=headless)


        self.charged_complete = [False] * num_agents
        self.area = SearchArea(grid_size=(grid_size, grid_size), section_size=1.5, home=self.search_offset)

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

        print("Mission started. drones searching assigned sections...")
        self.current_targets = [None] * num_agents
        self.path_progress = [0] * num_agents
        self.path_progress = [0] * num_agents
        self.last_reach_time = [0.0] * num_agents
        self.current_section = [None] * num_agents
        self.current_section = [None] * num_agents
        self.last_broadcast = [0.0] * num_agents
        self.last_positions = np.array([d.position for d in self.swarm]) # For distance calc

        self.returning_home = False
        self.home_targets, _ = generate_drone_positions(num_agents, HOME_POSITION)

        self.crashed = [False] * num_agents
        self.crash_timer = [0.0] * num_agents
        self.crashed = [False] * num_agents
        self.crash_timer = [0.0] * num_agents

        
        # Generates distinct colors for each drone (Golden Ratio HSV)
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
        
    def set_camera_target(self, drone_id):
        self.camera_target_id = drone_id

    @property
    def metrics(self):
        """Backward compatibility for external scripts accessing metrics directly."""
        return self.measurements.metrics
        


    def step(self):
        # Return True if simulation should continue, False if done/closed
        if self.mission_complete:
            self.measurements.update_error_metrics(self.crashed, self.health_status, self.battery_late)
            return False
            
        if not hasattr(self, 'env') or self.env.CLIENT < 0:
             return False
            
        # Use simulation time instead of wall clock
        # PYB_FREQ is usually 240, but we step at ctrl_freq (60).
        # self.env.step_counter counts physics steps.
        t = self.env.step_counter / self.env.PYB_FREQ
        
        actions = np.zeros((self.num_agents, 4))
        all_positions = get_all_positions(self.env, self.num_agents)

        if controller.injected_fault:
            fault_drone, fault_code = controller.injected_fault
            self.health_status[fault_drone] = fault_code
            controller.injected_fault = None
            fault_name = get_health_name(fault_code)
            print(f"[GUI Inject] agent {fault_drone} fault set to {fault_name}")

        if all(self.crashed) and not self.is_descending and not self.measurements.metrics.get("success", False):
             print("All drones crashed! Ending simulation.")
             self.measurements.end_mission(success=False, reason="All Drones Crashed")
             self.measurements.update_error_metrics(self.crashed, self.health_status, self.battery_late)
             return False

        # Additional Check: If everyone is either crashed or sitting at home (battery dead/fault), we can't search.
        # This prevents infinite loops if max_steps is removed.
        drones_incapacitated = []
        for i in range(self.num_agents):
            is_down = self.crashed[i] or (i < len(controller.home_ready) and controller.home_ready[i])
            drones_incapacitated.append(is_down)
        
        if all(drones_incapacitated) and not self.mission_complete:
             print("All drones are crashed or grounded at home. Mission failed (Swarm Depleted).")
             self.measurements.end_mission(success=False, reason="Swarm Depleted")
             self.measurements.update_error_metrics(self.crashed, self.health_status, self.battery_late)
             return False

        if controller.mission_aborted:
            self.returning_home = True
            controller.mission_aborted = False
            print("GUI: Mission abort detected, returning home!")

        if not controller.search_active and not self.returning_home:
            # Paused state
            if self.headless:
                # Auto-start in headless mode
                controller.search_active = True
            else:
                return True

        if not self.market_initialized:
            drone_positions = [drone.position for drone in self.swarm]
            self.market.open_market(drone_positions)
            controller.market_text = self.market.get_market_status()
            rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)
            self.market_initialized = True

        for j in range(self.num_agents):
            if self.return_reason[j] == "battery" and not self.charged_complete[j]:
                if j in self.battery_return_start and (t - self.battery_return_start[j]) > 60.0:
                    if j not in self.battery_late:
                        print(f"[Battery Timeout] Drone {j} took too long to change battery. Releasing sections.")
                        self.battery_late.add(j)
                        self.market.release_drone_sections(j, current_time=t)
                        controller.market_text = self.market.get_market_status()
                        drone_positions = [d.position for d in self.swarm]
                        self.market.dynamic_update(drone_positions, current_time=t)
                        controller.market_text = self.market.get_market_status()
                        rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)

        for i, drone in enumerate(self.swarm):
            state = drone.update()
            
            # Update metrics (Total Distance)
            curr_pos = np.array(drone.position)
            dist = np.linalg.norm(curr_pos - self.last_positions[i])
            self.measurements.update_distance(dist)
            self.last_positions[i] = curr_pos
            current_pos = np.array(state[0:3])

            if (not self.subject_found and not self.voting_active and not self.returning_home 
                and controller.search_active and self.current_section[i] is not None 
                and self.subject_pos is not None):
                
                drone_xy = np.array(current_pos[:2])
                subject_xy = np.array(self.subject_pos[:2])
                dist_to_subject = np.linalg.norm(drone_xy - subject_xy)

                is_subject_section_assigned_to_me = (
                    self.current_section[i] == self.subject_section_id and
                    self.area.sections[self.subject_section_id].assigned_drone == i
                )

                if (is_subject_section_assigned_to_me and dist_to_subject < 0.4):
                    self.subject_found = True
                    self.detecting_drone_id = i
                    drone.subject_found = 1
                    controller.middle_text = f"Drone {i} detected possible subject at {np.round(self.subject_pos[:2], 2)} in section {self.current_section[i]}!"
                    print(controller.middle_text)
                    self.detecting_drone_id = i

                    dists = [
                        (j, np.linalg.norm(np.array(self.swarm[j].position[:2]) - np.array(self.subject_pos[:2])))
                        for j in range(self.num_agents) if j != i
                    ]
                    dists.sort(key=lambda x: x[1])
                    self.helpers = [idx for idx, _ in dists[:3]]
                    print(f"[Subject] Drones {self.helpers} assigned to verify subject")
                    
                    controller.voting_text = (
                        "Subject verification started\n"
                        f"Candidate position: {np.round(self.subject_pos[:2], 2)}\n"
                        f"Verifiers: {self.helpers}\n"
                    )

                    self.verification_targets = {}
                    angles = [0, 120, 240]
                    for k, drone_id in enumerate(self.helpers):
                        offx = np.cos(np.radians(angles[k])) * 0.5
                        offy = np.sin(np.radians(angles[k])) * 0.5
                        self.verification_targets[drone_id] = np.array([self.subject_pos[0] + offx,
                                                                    self.subject_pos[1] + offy,
                                                                    FLY_HEIGHT])
                    self.voting_active = True
                    self.votes = []

            if i in controller.charged_drones:
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
                else:
                    drone_positions = [d.position for d in self.swarm]
                    self.market.dynamic_update(drone_positions, current_time=t)

                controller.market_text = self.market.get_market_status()
                rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)
                continue

            if controller.injected_fault:
                pass # Already handled at top of loop

            # Fault handling (Retasking)
            if self.health_status[i] != 0 and not self.returning_home:
                # Let RetaskingSystem decide
                decision = self.retasker.handle(i, self.health_status[i])
                if decision:
                    action_code = decision["action"]
                    # If action implies dropping tasks:
                    if action_code in ["LAND_NOW", "RETURN_HOME"]:
                         # If returning home due to fault, we might want to release tasks?
                         # Usually yes.
                         # Check if we already released?
                         pass
                    
                    # Implementation specific:
                    # For BAD_BATTERY, we release sections immediately?
                    if self.health_status[i] in [1, 2]: # Battery issues
                         # check if already handled
                         pass
                    
                    # For critical failures (Motor/GPS)
                    if self.health_status[i] >= 3:
                        if not self.crashed[i] and not self.return_active[i]: # first time detection
                             print(f"CRITICAL FAULT on Agent {i}. Releasing sections.")
                             self.market.release_drone_sections(i, current_time=t)
                             self.crashed[i] = True # Mark as virtually crashed/out of service
                             
                # We need to make sure we don't spam release
                # Simple logic: If health is bad, we release once.
                # I'll add a 'released_faults' set to track.
                pass 
                result = self.retasker.handle(i, self.health_status[i])
                if result:
                    action = result["action"]
                    if action == "RETURN_HOME":
                        target_pos = np.array(self.home_targets[i])
                        diff = target_pos - current_pos
                        dist = np.linalg.norm(diff)
                        if dist > 0.05:
                            next_pos = current_pos + 0.3 * diff
                        else:
                            next_pos = target_pos
                        rpm = drone.step_toward(next_pos)
                        if dist < 0.2:
                            controller.home_ready[i] = True
                        else:
                            controller.home_ready[i] = False
                        actions[i, :] = rpm
                        if not self.return_active[i]:
                            self.return_active[i] = True
                            self.return_timer[i] = t
                            self.charged_complete[i] = False
                            self.return_reason[i] = "battery"
                            self.battery_return_start[i] = t
                            print(f"Agent {i} returning home for battery change.")
                        continue
                    elif action == "LAND_NOW":
                        target_pos = np.array([current_pos[0], current_pos[1], 0.05])
                        next_pos = current_pos + 0.2 * (target_pos - current_pos)
                        rpm = drone.step_toward(next_pos)
                        actions[i, :] = rpm
                        self.market.release_drone_sections(i)
                        controller.market_text = self.market.get_market_status()
                        drone_positions = [d.position for d in self.swarm]
                        self.market.dynamic_update(drone_positions)
                        controller.market_text = self.market.get_market_status()
                        rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)
                        controller.middle_text = f"Agent {i} emergency landed — sections returned to market."
                        continue
                    elif action in ("HOVER", "REDUCED_ROLE", "HOVER_AND_RECONNECT"):
                        target_pos = np.array([current_pos[0], current_pos[1], 2])
                        next_pos = current_pos + 0.2 * (target_pos - current_pos)
                        rpm = drone.step_toward(next_pos)                        
                        actions[i, :] = rpm
                        self.market.release_drone_sections(i)
                        controller.market_text = self.market.get_market_status()
                        drone_positions = [d.position for d in self.swarm]
                        self.market.dynamic_update(drone_positions)
                        controller.market_text = self.market.get_market_status()
                        controller.middle_text = f"Agent {i} is being used as a relay, sections returned to market."
                        continue

            if not self.crashed[i]:
                if current_pos[2] <= 0.1:
                    if self.crash_timer[i] == 0.0:
                        self.crash_timer[i] = t
                    elif t - self.crash_timer[i] > 6.0:
                        self.crashed[i] = True
                        print(f"drone {i} has crashed! Altitude={current_pos[2]:.2f}")
                        p.addUserDebugText("CRASHED", [current_pos[0], current_pos[1], 0.1], textColorRGB=[1, 0, 0], textSize=2, lifeTime=0, physicsClientId=self.env.CLIENT)
                        self.market.release_drone_sections(i)
                        controller.market_text = self.market.get_market_status()
                        drone_positions = [d.position for d in self.swarm]
                        self.market.dynamic_update(drone_positions)
                        controller.market_text = self.market.get_market_status()
                        continue
                else:
                    self.crash_timer[i] = 0.0
            else:
                continue

            if t - self.last_broadcast[i] >= BROADCAST_PERIOD:
                msg = drone.broadcast()
                print(f"[Ping] drone {msg['ID']} at {msg['Pos']} | Time: {msg['Timer']}s")
                self.last_broadcast[i] = t

            if self.returning_home:
                if self.crashed[i]: continue
                state = drone.update()
                current_pos = np.array(state[0:3])
                target_pos = np.array([self.home_targets[i][0], self.home_targets[i][1], FLY_HEIGHT])
                diff = target_pos - current_pos
                dist = np.linalg.norm(diff)
                if dist > 0.05:
                    direction = diff / (dist + 1e-6)
                    speed_scale = np.clip(dist / 3.0, 0.3, 1.0)
                    move_dist = min(dist, MAX_SPEED * speed_scale / self.ctrl_freq * 6)
                    next_pos = current_pos + direction * move_dist
                else:
                    next_pos = target_pos
                next_pos[2] = FLY_HEIGHT
                rpm = drone.step_toward(next_pos)
                actions[i, :] = rpm
                continue

            if self.current_targets[i] is None:
                try:
                    cell_id, path = next(self.drone_tasks[i])
                except StopIteration:
                    hover_target = np.array([current_pos[0], current_pos[1], FLY_HEIGHT])
                    rpm = drone.step_toward(hover_target)
                    actions[i, :] = rpm
                    continue
                self.current_targets[i] = path
                self.path_progress[i] = 0
                self.current_section[i] = cell_id
                self.current_section[i] = cell_id
                self.last_reach_time[i] = t
                print(f"drone {i} → new section {cell_id}")

            if self.voting_active:
                # Remove crashed verifiers
                crashed_verifiers = [j for j in self.verification_targets.keys() if self.crashed[j]]
                for cv in crashed_verifiers:
                     print(f"[Subject] Verifier {cv} crashed. Removing from voting pool.")
                     del self.verification_targets[cv]
                     
                if not self.verification_targets:
                     print("[Subject] All verifiers crashed! Re-assigning or failing?")
                     # For simplicity, if all crashed, we abort voting and reset?
                     # Or just wait? If empty, len(votes) >= 0 is true immediately.
                     # But we need at least one vote?
                     # Let's abort voting.
                     self.voting_active = False
                     self.votes = []
                     self.subject_found = False # Reset so we can try again
                     controller.voting_text = "Voting aborted (all verifiers crashed)."
                     print(controller.voting_text)
     
                for j in list(self.verification_targets.keys()):
                    target = self.verification_targets[j]
                    diff_xy = target[:2] - self.swarm[j].position[:2]
                    dist = np.linalg.norm(diff_xy)
                    if dist > 0.25:
                        direction = diff_xy / (dist + 1e-6)
                        move_dist = min(dist, MAX_SPEED / self.ctrl_freq * 1.6)
                        next_xy = self.swarm[j].position[:2] + direction * move_dist
                        next_pos = np.array([next_xy[0], next_xy[1], FLY_HEIGHT])
                        rpm = self.swarm[j].step_toward(next_pos)
                        actions[j, :] = rpm
                    else:
                        if j not in [v["drone_id"] for v in self.votes]:
                            self.votes.append({"drone_id": j, "vote": "YES"})
                            msg = f"Drone {j} votes YES at {np.round(self.swarm[j].position[:2], 2)}"
                            print(msg)
                            controller.voting_text += msg + "\n"
                        hover_target = np.array([self.swarm[j].position[0], self.swarm[j].position[1], FLY_HEIGHT])
                        rpm = self.swarm[j].step_toward(hover_target)
                        actions[j, :] = rpm

                for k in range(self.num_agents):
                    if k not in self.verification_targets and not self.crashed[k]:
                        if self.current_targets[k] is not None and self.path_progress[k] < len(self.current_targets[k]):
                            target_pos = self.current_targets[k][self.path_progress[k]]
                            diff = np.array(target_pos) - self.swarm[k].position
                            dist = np.linalg.norm(diff)
                            direction = diff / (dist + 1e-6)
                            move_dist = min(dist, MAX_SPEED / self.ctrl_freq)
                            next_pos = self.swarm[k].position + direction * move_dist
                            push_drones = avoidance_from_drones(self.swarm[k].position, all_positions, k, radius=0.5, gain=0.3, max_push=0.4)
                            push_border = avoidance_from_borders(self.swarm[k].position, (self.grid_size, self.grid_size), 1.5, self.search_offset, margin=0.3, gain=0.1, max_push=0)
                            next_pos += 0.5 * (push_drones + push_border)
                            next_pos[2] = FLY_HEIGHT
                            rpm = self.swarm[k].step_toward(next_pos)
                            actions[k, :] = rpm
                            if dist < WAYPOINT_TOLERANCE:
                                if t - self.last_reach_time[k] > HOVER_TIME:
                                    self.path_progress[k] += 1
                                    self.last_reach_time[k] = t
                        else:
                            hover_target = np.array([self.swarm[k].position[0], self.swarm[k].position[1], FLY_HEIGHT])
                            rpm = self.swarm[k].step_toward(hover_target)
                            actions[k, :] = rpm

                if len(self.votes) >= len(self.verification_targets):
                    print("[Subject] Voting complete!")
                    controller.middle_text = "✅ Subject confirmed! All drones returning home."
                    controller.voting_text += "✅ Subject confirmed — returning home.\n"
                    total_time = round(time.time() - self.measurements.metrics["start_time"], 1)
                    controller.middle_text += f"\nSubject confirmed at {np.round(self.subject_pos[:2], 2)} | Time: {total_time}s"
                    self.measurements.end_mission(success=True)
                    # Merge market metrics
                    self.measurements.merge_market_metrics(self.market.get_metrics())
                    summary = self.measurements.get_metrics_summary()
                    print(f"Metrics: Success! Time={summary['duration']:.2f}s")
                    
                    if self.headless:
                        self.mission_complete = True
                        return True
                        
                    self.returning_home = True
                    self.voting_active = False
                    min_dist = float("inf")
                    subject_section = None
                    for sec in self.area.sections:
                        dist_to_section = np.linalg.norm(np.array(sec.position) - np.array(self.subject_pos[:2]))
                        if dist_to_section < min_dist:
                            min_dist = dist_to_section
                            subject_section = sec
                    if subject_section:
                        self.env.mark_section_as_searched(subject_section.position, color=(1, 0.84, 0))
                        print(f"[Subject] Located in section {subject_section.id}")
                        controller.middle_text += f"\nSubject located in Section {subject_section.id}"
                self.env.step(actions)
                return True

            if self.current_targets[i] is None:
                hover_target = np.array([current_pos[0], current_pos[1], FLY_HEIGHT])
                rpm = drone.step_toward(hover_target)
                actions[i, :] = rpm
                continue

            target_pos = self.current_targets[i][self.path_progress[i]]
            
            # Draw debug line to waypoint if GUI is active
            # Draw debug line to waypoint if GUI is active (throttled)
            if not self.headless:
                # Debug Waypoint Line
                if self.env.step_counter % 10 == 0:
                     self.visualizer.update_waypoint_line(i, current_pos, target_pos, show_lines=controller.show_debug_lines)
                elif not controller.show_debug_lines:
                     self.visualizer.clear_debug_lines() # This might be aggressive to call in loop? No, handled in manager.

                # Full Section Path Visualization
                if controller.show_full_paths and not self.visualizer.section_path_lines[i]:
                    self.visualizer.draw_full_path(i, self.current_targets[i], enabled=True)
                elif not controller.show_full_paths and self.visualizer.section_path_lines[i]:
                    self.visualizer.clear_full_path(i)

            diff = np.array(target_pos) - current_pos
            dist = np.linalg.norm(diff)
            
            # Default direction (straight to waypoint)
            direction = diff / (dist + 1e-6)
            
            if controller.flight_mode == "standard":
                speed_scale = np.clip(dist / 1.5, 0.8, 2.5)
            elif controller.flight_mode == "aggressive":
                speed_scale = 2.5
            elif controller.flight_mode == "path_follow":
                speed_scale = 2.5
                # Vector Field / Path Following Logic
                # Get start point of segment
                if self.path_progress[i] == 0:
                     # Start of section path (or from home)
                     start_pos = self.last_positions[i] # Approximate
                else:
                     start_pos = self.current_targets[i][self.path_progress[i]-1]
                
                segment_vec = np.array(target_pos) - np.array(start_pos)
                seg_len = np.linalg.norm(segment_vec)
                
                if seg_len > 0.1:
                    seg_dir = segment_vec / seg_len
                    # Project current pos onto line
                    to_drone = current_pos - np.array(start_pos)
                    proj_len = np.dot(to_drone, seg_dir)
                    closest_point = np.array(start_pos) + np.clip(proj_len, 0, seg_len) * seg_dir
                    
                    # Correction vector (pull to line)
                    correction = closest_point - current_pos
                    
                    # Desired velocity: Parallel to path + Correction
                    # We want to move along seg_dir, but pull towards line
                    desired_dir = seg_dir + correction * 2.0 # Gain on correction
                    norm = np.linalg.norm(desired_dir)
                    if norm > 0:
                        direction = desired_dir / norm
            
            elif controller.flight_mode == "smooth_follow":
                speed_scale = 2.5
                # Lookahead / Corner Cutting Logic
                LOOKAHEAD_DIST = 0.6
                
                # Identify current segment
                if self.path_progress[i] == 0:
                     start_pos = self.last_positions[i]
                else:
                     start_pos = self.current_targets[i][self.path_progress[i]-1]
                
                segment_vec = np.array(target_pos) - np.array(start_pos)
                seg_len = np.linalg.norm(segment_vec)
                
                if seg_len > 0.01:
                    seg_dir = segment_vec / seg_len
                    to_drone = current_pos - np.array(start_pos)
                    proj_len = np.dot(to_drone, seg_dir)
                    
                    # Target point is proj_len + LOOKAHEAD
                    target_dist_on_path = proj_len + LOOKAHEAD_DIST
                    
                    if target_dist_on_path > seg_len:
                        # We are looking past the current waypoint -> Cut corner to next segment
                        excess = target_dist_on_path - seg_len
                        
                        # Check if there is a next waypoint
                        if self.path_progress[i] + 1 < len(self.current_targets[i]):
                            next_wp = self.current_targets[i][self.path_progress[i]+1]
                            next_seg_vec = np.array(next_wp) - np.array(target_pos)
                            next_seg_len = np.linalg.norm(next_seg_vec)
                            if next_seg_len > 0.01:
                                next_seg_dir = next_seg_vec / next_seg_len
                                # Point is 'excess' meters into the next segment
                                lookahead_point = np.array(target_pos) + np.clip(excess, 0, next_seg_len) * next_seg_dir
                            else:
                                lookahead_point = np.array(target_pos)
                        else:
                            # No next waypoint, just aim at target
                            lookahead_point = np.array(target_pos)
                    else:
                        # Still on current segment
                        lookahead_point = np.array(start_pos) + max(0, target_dist_on_path) * seg_dir
                    
                    # Steer towards lookahead point
                    diff_la = lookahead_point - current_pos
                    dist_la = np.linalg.norm(diff_la)
                    if dist_la > 0.1:
                        direction = diff_la / dist_la
            
            else:
                speed_scale = 1.0 # Fallback

            move_dist = min(dist, MAX_SPEED * speed_scale / self.ctrl_freq * 2)
            next_pos = current_pos + direction * move_dist

            # Navigation/Avoidance Forces
            # Reduced gains to prevent "repulsion" from valid waypoints
            push_drones = avoidance_from_drones(current_pos, all_positions, i, radius=0.5, gain=0.2, max_push=0.4)
            push_border = avoidance_from_borders(current_pos, (self.grid_size, self.grid_size), 1.5, self.search_offset, margin=0.1, gain=0.1, max_push=0)
            next_pos = next_pos + 0.5 * (push_drones + push_border)
            
            if int(t) % 60 == 0:
                drone.controller.reset()
            
            next_pos[2] = FLY_HEIGHT
            rpm = drone.step_toward(next_pos)
            actions[i, :] = rpm

            if dist < WAYPOINT_TOLERANCE:
                if t - self.last_reach_time[i] > HOVER_TIME:
                    self.path_progress[i] += 1
                    self.last_reach_time[i] = t
                    if self.path_progress[i] >= len(self.current_targets[i]):
                        if self.current_section[i] is not None:
                            self.area.mark_searched(self.current_section[i])
                            cell_center = self.area.sections[self.current_section[i]].position
                            self.env.mark_section_as_searched(cell_center)
                            print(f"drone {i} finished section {self.current_section[i]}")
                            self.market.reward_and_remove_section(i, self.current_section[i])
                            # Section completed metric
                            self.measurements.increment_sections_searched(i)
                            controller.market_text = self.market.get_market_status()
                            drone_positions = [d.position for d in self.swarm]
                            self.market.dynamic_update(drone_positions, current_time=t)
                            controller.market_text = self.market.get_market_status()
                            rebuild_tasks_from_market(self.drone_tasks, self.market, self.area, self.swarm, self.num_agents)
                            self.current_targets[i] = None
                            self.path_progress[i] = 0
                            self.current_targets[i] = None
                            self.path_progress[i] = 0
                            self.last_reach_time[i] = t
                            
                            # Clear path visualization for the finished section
                            self.visualizer.clear_full_path(i)
                            
                            try:
                                cell_id, path = next(self.drone_tasks[i])
                                self.current_section[i] = cell_id
                                self.current_targets[i] = path
                                self.visualizer.draw_full_path(i, path, enabled=controller.show_full_paths)
                            except StopIteration:
                                hover_target = np.array([current_pos[0], current_pos[1], FLY_HEIGHT])
                                rpm = drone.step_toward(hover_target)
                                actions[i, :] = rpm
                                continue

        broadcast_lines = []
        for i, drone in enumerate(self.swarm):
            msg = drone.broadcast()
            broadcast_lines.append(f"Agent {msg['ID']}: Pos={tuple(np.round(msg['Pos'],2))}, Time={msg['Timer']}s")
        controller.broadcast_text = "\n".join(broadcast_lines)

        assign_lines = []
        for i in range(self.num_agents):
            sections = [c.id for c in self.area.sections if c.assigned_drone == i]
            assign_lines.append(f"drone {i}: Sections {sections}")
        controller.assignment_text = "\n".join(assign_lines)

        searched_lines = []
        for c in self.area.sections:
            status = "✅ Done" if c.searched else "🔲 Searching"
            searched_lines.append(f"Section {c.id}: {status}")
        controller.searched_text = "\n".join(searched_lines)

        # Handle descent if returning home and all drones are close
        if self.returning_home and not self.is_descending:
             distances = []
             for i in range(self.num_agents):
                 if self.crashed[i]: continue
                 pos = np.array(self.env._getDroneStateVector(i)[0:3])
                 dist = np.linalg.norm(pos[:2] - self.home_targets[i][:2])
                 distances.append(dist)
             if len(distances) == 0 or all(d < 1.8 for d in distances):
                  print("All drones are returning home. Starting descent.")
                  self.is_descending = True
        
        if self.is_descending:
             self.descent_timer += 1
             if self.descent_timer < self.ctrl_freq * 30: # 30 seconds
                for i, drone in enumerate(self.swarm):
                    if self.crashed[i]: continue
                    state = drone.update()
                    current_pos = np.array(state[0:3])
                    target_pos = np.array([self.home_targets[i][0], self.home_targets[i][1], 0.05])
                    next_pos = current_pos + 0.2 * (target_pos - current_pos)
                    rpm = drone.step_toward(next_pos)
                    actions[i, :] = rpm
             else:
                # Finalize
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

        if self.is_descending: # already stepped actions
            pass # actions updated inside descent block
        
        # Override actions if descending block didn't run (it runs partially)
        # Actually I need to be careful not to double step.
        # The descent block above updates actions but I also need to make sure I don't partial-update.
        # Let's simplify: if descending, we set actions in that block.
        # If not descending, we set actions in the main loop above.
        # In both cases we call env.step(actions) at the end.
        
        if self.returning_home:
            # The actions were set in the returning_home block unless we are descending
            pass

        if not self.is_descending and not self.voting_active and not self.returning_home:
             # Check if all searched
             if all(c.searched for c in self.area.sections):
                 print("The search area is searched! drones will return home together...")
                 self.returning_home = True

        # Camera tracking update
        if self.camera_target_id is not None:
             if 0 <= self.camera_target_id < self.num_agents:
                 try:
                     state = self.swarm[self.camera_target_id].update() # Peek state without side effects? 
                     # Actually drone.update() might be expensive or state-changing if called multiple times?
                     # No, drone.update() usually just reads state in this codebase or does minimal calc.
                     # But safer to just read position from pybullet directly or use cached position if available.
                     # Let's use the pybullet API directly to be safe and accurate to visual state.
                     pos, _ = p.getBasePositionAndOrientation(self.env.DRONE_IDS[self.camera_target_id], physicsClientId=self.env.CLIENT)
                     p.resetDebugVisualizerCamera(
                         cameraDistance=1.5,
                         cameraYaw=-90,
                         cameraPitch=-40,
                         cameraTargetPosition=[pos[0], pos[1], pos[2]]
                     )
                 except Exception:
                     pass

        self.env.step(actions)
        return True

    def show_summary(self):
        subject_section = None
        for sec in self.area.sections:
            if np.linalg.norm(np.array(sec.position) - np.array(self.subject_pos[:2])) < 1.0: # rough check
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

def main():
    pass # No OP now
