
import main
import pybullet as p
import time
from config import FLIGHT_MODE

# Control flags
simulation_running = False
search_active = False
search_active = False
mission_aborted = False
show_debug_lines = True
use_ramp_down = True

def toggle_debug_lines(state):
    global show_debug_lines
    show_debug_lines = state
    print(f"Debug lines toggled: {state}")

def toggle_ramp_down(state):
    global flight_mode
    flight_mode = "standard" if state else "aggressive"
    print(f"Flight Mode: {flight_mode}")

show_full_paths = True
flight_mode = FLIGHT_MODE # Options: "standard", "aggressive", "path_follow", "smooth_follow"

def set_flight_mode(mode):
    global flight_mode
    flight_mode = mode
    print(f"Flight Mode set to: {flight_mode}")

def toggle_full_paths(state):
    global show_full_paths
    show_full_paths = state
    print(f"Show Full Paths: {state}")

# Shared text for GUI
broadcast_text = ""
assignment_text = ""
voting_text = ""             
# Fault injection & health feedback
injected_fault = None      # (drone_id, health_code)
middle_text = ""           # Text from test.py for GUI display
home_ready = []            # list of booleans, same length as number of drones
charged_drones = set()     # track drones that have been charged
market_text = ""
searched_text = ""

_sim_instance = None
_last_update_time = 0.0
_accumulator = 0.0
_fixed_dt = 1.0 / 60.0  # Target physics step (60Hz)

def start_search():
    global search_active, mission_aborted
    if not _sim_instance:
        print("Simulation not started yet!")
        return
    mission_aborted = False
    search_active = True
    print("Search started!")


def abort_mission():
    global search_active, mission_aborted
    if not _sim_instance:
        print("Simulation not running!")
        return
    search_active = False
    mission_aborted = True
    print("Mission aborted! Returning agents to base.")

def mark_drone_charged(drone_id):
    """Called from GUI when the Charge button is pressed."""
    global home_ready, charged_drones
    if len(home_ready) > drone_id and home_ready[drone_id]:
        charged_drones.add(drone_id)
        home_ready[drone_id] = False  # reset flag
        print(f"[Controller] Drone {drone_id} marked as charged.")

def set_camera_target(drone_id):
    """Called from GUI to set camera target. Pass None or -1 to disable tracking."""
    global _sim_instance
    if _sim_instance:
        if drone_id == -1:
            _sim_instance.set_camera_target(None)
            print("[Controller] Camera tracking disabled (Free Cam).")
        else:
            _sim_instance.set_camera_target(drone_id)
            print(f"[Controller] Camera tracking Drone {drone_id}.")

def run_simulation(num_drones=4, grid_size=4):
    global simulation_running, _sim_instance, home_ready, charged_drones
    global _last_update_time, _accumulator
    
    if _sim_instance is not None:
        print("Simulation already running/initialized.")
        return

    home_ready = [False] * num_drones
    charged_drones = set()
    simulation_running = True
    
    # Instantiate the manager
    _sim_instance = main.SimulationManager(num_drones, grid_size)
    
    # Initialize timing
    _last_update_time = time.time()
    _accumulator = 0.0

def update_simulation():
    """Called by GUI timer on main thread."""
    global simulation_running, _sim_instance
    global _last_update_time, _accumulator
    
    if _sim_instance is not None and simulation_running:
        try:
            current_time = time.time()
            frame_time = current_time - _last_update_time
            _last_update_time = current_time
            
            # Clamp frame time to avoid spiral of death on lag spikes (max 0.25s)
            if frame_time > 0.25:
                frame_time = 0.25
                
            _accumulator += frame_time
            
            # Consume accumulated time in fixed steps
            while _accumulator >= _fixed_dt:
                running = _sim_instance.step()
                if not running:
                    stop_simulation()
                    return
                _accumulator -= _fixed_dt
                
            # Optional: If you want to interpolate rendering, you'd do it here.
            # For now, just stepping physics is enough.
            
        except Exception as e:
            print(f"Simulation Exception: {e}")
            import traceback
            traceback.print_exc()
            stop_simulation()

def stop_simulation():
    global simulation_running, _sim_instance
    simulation_running = False
    if _sim_instance:
        _sim_instance.close()
        _sim_instance = None
    print("Simulation stopped.")