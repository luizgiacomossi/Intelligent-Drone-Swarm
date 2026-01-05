# Intelligent Drone Swarm for Search and Rescue (SAR)

This repository contains a high-fidelity simulation of a Multi-UAV system designed for resilient Search and Rescue missions. The system implements a **Decentralized Market-Based Consensus** strategy to coordinate a swarm of drones in dynamic environments, featuring fault tolerance, real-time retasking, and collaborative target verification.

## 🚀 Key Features

*   **Market-Based Task Allocation**: Drones bid for search sections based on a dynamic pricing model (Distance + Base Value), ensuring efficient workload distribution.
*   **Resilient Retasking**: Automatic detection of agent failures (e.g., Low Battery, GPS Failure). Failed agents release their tasks back to the market for immediate reallocation.
*   **Consensus Verification**: A spatial voting protocol where neighbor drones are recruited to verify potential target detections, minimizing false positives ($P_{error} < 10^{-4}$).
*   **Reactive Collision Avoidance**: Decentralized navigation using Artificial Potential Fields (APF).
*   **Physics-Based Simulation**: Built on `gym-pybullet-drones` for realistic flight dynamics and sensor simulation.

## 🛠️ Installation & Usage

### Prerequisites
*   Python 3.8+
*   `numpy`, `pybullet`, `gym-pybullet-drones`
*   `PyQt5` (for the control interface)

### Running the Simulation
To start the main interactive simulation with the GUI control panel:
```bash
python main.py
```
*   **GUI Controls**: Use the control panel to Start/Pause, Inject Faults, or abort the mission.
*   **Visualization**: The PyBullet window shows the 3D drone behaviors, while the GUI displays real-time market logs and health status.

### Running Experiments
To run the headless batch experiments (Scalability & Fault Tolerance analysis):
```bash
python experiments.py
```
Results will be saved to `experiment_results.csv`.

## 🏗️ System Architecture

The software is organized into modular components managed by a central `SimulationManager`.

*   **`SimulationManager`**: The orchestration loop synchronizing physics (PyBullet) and logic (60Hz).
*   **`MarketSystem`**: Manages the virtual economy, auctions sections, and handles currency transactions ($W_i$).
*   **`RetaskingSystem`**: A decision logic lookup table mapping Health Codes $\to$ Recovery Actions (e.g., `BAD_BATTERY` $\to$ `RETURN_HOME`).
*   **`Drone`**: Agent class encapsulating PID control, state estimation, and sensor simulation.

## 📂 File Structure

```text
.
├── main.py                     # Entry point for interactive simulation
├── experiments.py              # Batch experiment runner
├── market.py                   # Market mechanism & auction logic
├── retasking.py                # Fault handling state machine
├── drone.py                    # UAV agent class
├── env.py                      # Custom PyBullet environment wrapper
├── avoidance.py                # Potential Field collision avoidance
├── subject.py                  # Target spawning manager
├── tables.py                   # Lookup tables for Health Codes & Commands
└── documentation/              # Detailed Project Documentation
    ├── methodology.md          # Math models, Algorithms & Introduction
    ├── experiments.md          # Experimental setup & Statistical methods
    ├── 01_introduction.md      # (Detailed) Problem statement
    ├── 02_mathematical_model.md# (Detailed) Formal definitions
    ├── 03_algorithm_implementation.md # (Detailed) Code specs
    └── 04_experiments_and_results.md # (Detailed) Metrics
```

## 📚 Documentation

For detailed theoretical/mathematical explanations, please refer to the `documentation/` folder:
*   [Methodology (Unified)](documentation/methodology.md)
*   [Experimental Results (Unified)](documentation/experiments.md)
