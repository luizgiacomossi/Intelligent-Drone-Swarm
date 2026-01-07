import sys
import os
import time
import numpy as np

# Ensure code-files is in path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import SimulationManager

def verify_physics():
    print("Initializing simulation...")
    # Initialize simulation in headless mode
    sim = SimulationManager(num_agents=4, grid_size=4, headless=True)
    
    print("Running simulation steps...")
    # Run for 2 seconds (assuming 60Hz -> 120 steps)
    accumulated_vel = np.zeros(3)
    accumulated_acc = np.zeros(3)
    
    for i in range(120):
        sim.step()
        
        # Check drone 0
        drone = sim.swarm[0]
        vel = drone.velocity
        acc = drone.acceleration
        
        accumulated_vel += np.abs(vel)
        accumulated_acc += np.abs(acc)
        
        if i % 20 == 0:
            print(f"Step {i}:")
            print(f"  Pos: {drone.position}")
            print(f"  Vel: {vel}")
            print(f"  Acc: {acc}")
            print(f"  Drone ID: {id(drone)}")
            print("=" * 20)
            
    print("\nVerification Results:")
    print(f"Total accumulated abs vel: {np.sum(accumulated_vel)}")
    print(f"Total accumulated abs acc: {np.sum(accumulated_acc)}")
    
    if np.sum(accumulated_vel) > 0.1 and np.sum(accumulated_acc) > 0.1:
        print("SUCCESS: Velocity and Acceleration are being updated.")
    else:
        print("FAILURE: Velocity or Acceleration seem to be zero or too low.")

    sim.close()

if __name__ == "__main__":
    verify_physics()
