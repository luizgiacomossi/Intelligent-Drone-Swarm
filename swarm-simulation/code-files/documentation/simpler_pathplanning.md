# Path Planning Architecture

Path planning in the simulation is implemented in a hierarchical 3-level architecture:

## 1. Global Allocation (Task Level)
**Logic**: A **Market-Based Approach** is used where the search area is divided into a grid. Drones "bid" on sections based on distance and their accumulated "wealth" (points).
- **File**: `market.py` (`MarketSystem` class).
- **Key Function**: `dynamic_update()` calculates bids and assigns `GridSection`s to drones.

## 2. Local Planning (Path Generation)
**Logic**: Once a section is assigned, a **Lawnmower Pattern (Zig-Zag)** is generated to cover that specific functionality.
- **File**: `main.py`.
- **Key Function**: `generate_lawnmower_points()` creates a list of coordinate waypoints (Search Path) inside the assigned section.

## 3. Reactive Control (Execution & Avoidance)
**Logic**: The drone follows the generated waypoints but adds a **Artificial Potential Field (APF)** vector to avoid collisions.
- **Files**:
    - `main.py`: The `step()` loop calculates the desired velocity towards the next waypoint.
    - `avoidance.py`: Calculates repulsion vectors from other drones and borders (`avoidance_from_drones`, `avoidance_from_borders`).
    - `drone.py`: The `DSLPIDControl` takes the final adjusted target and converts it to motor RPMs.

**Summary**: **Market** decides *where* to go (Section), **Lawnmower** decides *how* to search it (Waypoints), and **APF** ensures *safety* during flight.
