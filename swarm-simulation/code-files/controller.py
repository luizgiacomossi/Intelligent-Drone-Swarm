
import main
import pybullet as p

# Control flags
simulation_running = False
search_active = False
mission_aborted = False

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

def run_simulation(num_drones=4, grid_size=4):
    global simulation_running, _sim_instance, home_ready, charged_drones
    
    if _sim_instance is not None:
        print("Simulation already running/initialized.")
        return

    home_ready = [False] * num_drones
    charged_drones = set()
    simulation_running = True
    
    # Instantiate the manager
    _sim_instance = main.SimulationManager(num_drones, grid_size)

def update_simulation():
    """Called by GUI timer on main thread."""
    global simulation_running, _sim_instance
    if _sim_instance is not None and simulation_running:
        try:
            running = _sim_instance.step()
            if not running:
                stop_simulation()
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