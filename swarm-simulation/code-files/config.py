# config.py

# --- Simulation Constants ---
CTRL_FREQ = 60                       # [Hz] Frequency of the control loop (physics acts every 1/60s)
GRID_SIZE = 4                        # [Integer] Dimension of the grid (GRID_SIZE x GRID_SIZE sections)
SECTION_SIZE = 1.5                   # [Meters] Size of each square section side
SECTION_SWEEP_STEPS = 4              # [Integer] Number of sweeps for the lawnmower pattern generation
LAWNMOWER_MARGIN_FACTOR = 0.25       # [Float] Percentage of section size to wait before sweeping (margin for lawnmower path)

# --- Drone Physics & Logic ---
HOME_POSITION = (0, 0)               # [Meters] X, Y coordinate for the drone starting/home position
FLY_HEIGHT = 1.0                     # [Meters] Target altitude for flight
MAX_SPEED = 8.0                      # [Meters/s] Maximum speed limit for drones
WAYPOINT_TOLERANCE = 0.15            # [Meters] Distance tolerance to consider a waypoint reached
HOVER_TIME = 1                    # [Seconds] Time to hover at a waypoint before moving to next
BROADCAST_PERIOD = 5.0               # [Seconds] Period for drones to broadcast their status
RETURN_TIMEOUT = 10.0                # [Seconds] Timeout before assuming a drone is lost/crashed
CRASH_HEIGHT_THRESHOLD = 0.1         # [Meters] Altitude below which a drone is considered crashed
CRASH_TIMEOUT = 0.5                  # [Seconds] Time to wait before confirming a crash
BATTERY_CHANGE_DURATION = 60.0       # [Seconds] Duration required to recharge battery
MAX_FRAME_TIME = 0.25                # [Seconds] Maximum allowable time per frame (for lag handling)

# --- Search & Detection ---
SUBJECT_DETECTION_DIST = 0.4         # [Meters] Distance to detect the subject
VERIFICATION_DIST = 0.25             # [Meters] Distance tolerance for verification hover position
VOTING_RADIUS = 0.5                  # [Meters] Radius of the voting/verification circle around subject
VOTING_ANGLES = [0, 120, 240]        # [Degrees] Angles for the 3 verifiers around the subject
VERIFICATION_SPEED_FACTOR = 6.5      # [Factor] Speed multiplier during verification phase (relative to base calculation)
SEARCH_OFFSET = (6, 4)               # [Tuple] Offset for the search area center (x, y)

# --- Market Economy ---
STARTING_POINTS = 3.0                # [Points] Initial points allocated to each drone
SECTION_BASE_VALUE = 2.0             # [Points] Base value of searching one section
FORCE_BUY_COST = 2.0                 # [Points] Cost penalty for failing a task/battery timeout
DYNAMIC_PRICE_FACTOR = 10.0          # [Factor] Multiplier for dynamic pricing based on demand/scarcity

# --- Flight Control ---
FLIGHT_MODE = "aggressive"           # [String] Options: "standard", "aggressive"

# --- Collision Avoidance ---
AVOIDANCE_FACTOR = 0.5               # [Factor] Global weight for avoidance forces (0.0 to 1.0 generally)

# Drone-to-Drone Avoidance
AVOID_DRONE_RADIUS = 0.7             # [Meters] Distance at which drones start repelling each other
AVOID_DRONE_MAX_PUSH = 0.8           # [Force] Maximum repulsion force magnitude
AVOID_DRONE_GAIN_NAV = 0.2           # [Gain] Strength of repulsion during normal navigation
AVOID_DRONE_GAIN_VERIFY = 0.3        # [Gain] Strength of repulsion during verification (higher to prevent crowding)

# Border Avoidance
AVOID_BORDER_GAIN = 0.1              # [Gain] Strength of repulsion from search area borders
AVOID_BORDER_MAX_PUSH = 0.0          # [Force] Maximum border repulsion force (0 means disabled/soft limit)
AVOID_BORDER_MARGIN_NAV = 0.1        # [Meters] Safety margin distance from border during navigation
AVOID_BORDER_MARGIN_VERIFY = 0.3     # [Meters] Safety margin distance from border during verification

# --- Navigation Control Parameters ---
NAV_STD_SPEED_DIVISOR = 1.2          # Divisor for distance to speed calculation in standard mode
NAV_STD_SPEED_MIN = 1.8              # Minimum speed scale for standard mode
NAV_STD_SPEED_MAX = 3.5              # Maximum speed scale for standard mode

NAV_AGGRESSIVE_SPEED_SCALE = 2.5     # Speed scale for aggressive mode

NAV_GLOBAL_STEP_MULTIPLIER = 2.0     # Multiplier for final step distance calculation

RETURN_HOME_DIST_THRESHOLD = 0.05    # [Meters] Distance to consider 'at home' target for fine adjustment
RETURN_HOME_SPEED_DIVISOR = 3.0
RETURN_HOME_SPEED_MIN = 0.3
RETURN_HOME_SPEED_MAX = 1.0
RETURN_HOME_STEP_MULTIPLIER = 4.0    # Speed multiplier when returning home
