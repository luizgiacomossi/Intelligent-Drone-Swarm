
# Intelligent Replanning Drone Swarm Simulation

This project implements an intelligent drone swarm simulation using PyBullet.

## Installation

### 1. Prerequisites
- [Anaconda](https://www.anaconda.com/) or Miniconda
- Git

### 2. Setup Environment

First, create a new conda environment:

```sh
conda create -n drones python=3.10
conda activate drones
```

### 3. Install Dependencies

You need to install `gym-pybullet-drones` and other requirements. 

**Option A (Recommended): Install as a library**
```sh
# Clone gym-pybullet-drones
git clone https://github.com/utiasDSL/gym-pybullet-drones.git

# Install it in editable mode
cd gym-pybullet-drones
pip install -e .

# Install PyQt5 for the GUI
pip install PyQt5
```

## Running the Simulation

1. Navigate to the simulation code directory:
   ```sh
   cd path/to/Intelligent-Drone-Swarm/swarm-simulation/code-files
   ```

2. Activate your environment if not already active:
   ```sh
   conda activate drones
   ```

3. Run the GUI:
   ```sh
   python gui.py
   ```

## Usage
- **Control Panel**: Use the GUI to set the number of agents and grid size.
- **Run Simulation**: Click "Run Simulation" to start the PyBullet visualization.
- **Start Search**: Once running, click "Start Search" to begin the swarm mission.

## Note for macOS Users
The simulation has been updated to run the PyBullet window on the main thread to prevent crashes on macOS. Ensure you use the provided `gui.py` to launch the simulation.
