import time
import pandas as pd
import numpy as np
import controller
import main
import sys
import os
import pybullet as p

# Ensure we can import local modules
sys.path.append(os.getcwd())

def run_experiment(num_agents, fault_config, max_steps=100000, grid_size=8, headless=True):
    """
    Runs a single simulation experiment.
    
    Args:
        num_agents (int): Number of drones
        fault_config (list): List of dicts defining faults, e.g., [{"time": 8.0, "agent": 0, "type": 1}]
        max_steps (int): Max validation steps
        grid_size (int): Grid dimension (4x4)
        headless (bool): Run without GUI
    """
    
    # Reset controller state
    controller.search_active = False 
    controller.mission_aborted = False
    controller.injected_fault = None
    controller.home_ready = [False] * num_agents
    controller.charged_drones = set()
    
    # Init Simulation
    sim = main.SimulationManager(num_agents=num_agents, grid_size=grid_size, headless=headless)
    
    # Convert fault timing to steps
    # CTRL_FREQ is usually 60Hz. We can get it from sim.env.CTRL_FREQ if available, or constants.
    # Assuming 60Hz for now, consistent with docs.
    ctrl_freq = 60
    
    scheduled_faults = []
    if fault_config:
        for f in fault_config:
            step = int(f["time"] * ctrl_freq)
            scheduled_faults.append({
                "step": step,
                "agent_id": f["agent"],
                "health": f["type"] # 1 = BAD_BATTERY
            })
    
    # Sort by step to pop efficiently
    scheduled_faults.sort(key=lambda x: x["step"])
    
    step_count = 0
    start_time_real = time.time()
    
    while step_count < max_steps:
        # Check faults
        while scheduled_faults and step_count >= scheduled_faults[0]["step"]:
            f = scheduled_faults.pop(0)
            controller.injected_fault = (f["agent_id"], f["health"])
            print(f"[Exp] Injecting fault {f['health']} to agent {f['agent_id']} at step {step_count} (~{f['step']/ctrl_freq}s)")
        
        running = sim.step()
        if not running: # Mission complete
            break
        step_count += 1
        
    duration = step_count / float(ctrl_freq)
    sim.measurements.metrics["end_time"] = duration

    # Merge market metrics if not already merged
    if "reallocation_time" not in sim.measurements.metrics:
         sim.measurements.metrics.update(sim.market.get_metrics())
         
    # Post-process metrics
    # Gini Coefficient for workload balance
    searched = np.array(sim.measurements.metrics.get("sections_searched_per_drone", [0]*num_agents))
    if np.sum(searched) > 0:
        mean_s = np.mean(searched)
        gini = np.sum(np.abs(searched - mean_s)) / (2 * num_agents * mean_s)
    else:
        gini = 0.0
        
    # Calculate coverage based on unique sections (robust against double counting)
    searched_count_unique = sum(1 for s in sim.market.sections if s["searched"])
    total_sections = sim.grid_size * sim.grid_size
    coverage_pct = (searched_count_unique / total_sections) * 100.0
        
    result = {
        "num_agents": num_agents,
        "fault_count": len(fault_config) if fault_config else 0,
        "success": sim.measurements.metrics.get("success", False),
        "duration": duration,
        "total_distance": sim.measurements.metrics.get("total_distance", 0.0),
        "avg_distance": sim.measurements.metrics.get("total_distance", 0.0) / num_agents if num_agents > 0 else 0,
        "gini_index": gini,
        "reallocation_time": sim.measurements.metrics.get("reallocation_time", 0.0),
        "cost_efficiency": sim.measurements.metrics.get("cost_efficiency", 0.0),
        "recovery_count": sim.measurements.metrics.get("recovery_count", 0),
        "crashed_count": sim.measurements.metrics.get("crashed_count", 0),
        "total_bids": sim.measurements.metrics.get("total_bids", 0),
        "bids_per_auction": (sim.measurements.metrics.get("total_bids", 0) / sim.measurements.metrics.get("total_auctions", 1)) if sim.measurements.metrics.get("total_auctions", 0) > 0 else 0,
        "wasted_attempts": sim.measurements.metrics.get("wasted_attempts", 0),
        "map_coverage": coverage_pct,
        "scenario_tag": "" # To be filled by caller
    }
    
    sim.env.close()
    
    # Ensure PyBullet is cleaned up
    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass
        
    return result

def main_experiment():
    results = []
    
    # --- Configuration ---
    TRIALS = 30
    
    print(f"Starting Experiments for Paper (Trials={TRIALS})...")
    
    # 1. Scalability Analysis (No Faults)
    # N = 2, 4, 8
    for n in [2, 4, 8]:
        print(f"\n--- Scenario: Scalability (N={n}) ---")
        for i in range(TRIALS):
            print(f"  Trial {i+1}/{TRIALS}...", end="", flush=True)
            res = run_experiment(num_agents=n, fault_config=None)
            res["scenario"] = "Scalability"
            res["scenario_tag"] = f"N{n}_NoFault"
            res["trial"] = i
            results.append(res)
            print(f" Done. Success={res['success']}, Dur={res['duration']:.2f}s")

    # 2. Fault Tolerance Analysis (N=8)
    # 1 Fault at 8s
    # 2 Faults at 8s and 15s
    
    fault_scenarios = [
        {
            "name": "Fault_1",
            "config": [{"time": 8.0, "agent": 0, "type": 1}]
        },
        {
            "name": "Fault_2",
            "config": [
                {"time": 8.0, "agent": 0, "type": 1},
                {"time": 15.0, "agent": 1, "type": 1}
            ]
        }
    ]
    
    for scen in fault_scenarios:
        print(f"\n--- Scenario: {scen['name']} (N=8) ---")
        for i in range(TRIALS):
            print(f"  Trial {i+1}/{TRIALS}...", end="", flush=True)
            res = run_experiment(num_agents=8, fault_config=scen["config"])
            res["scenario"] = "FaultTolerance"
            res["scenario_tag"] = scen["name"]
            res["trial"] = i
            results.append(res)
            print(f" Done. Success={res['success']}, Dur={res['duration']:.2f}s, Realloc={res['reallocation_time']:.2f}s")

    # --- Save & Summary ---
    df = pd.DataFrame(results)
    df.to_csv("experiment_results_paper.csv", index=False)
    print("\nExperiments Completed. Saved to 'experiment_results_paper.csv'.")
    
    # Print formatted summary table
    print("\n=== Results Summary ===")
    summary = df.groupby(["scenario", "scenario_tag"]).agg({
        "duration": ["mean", "std"],
        "total_distance": ["mean", "std"],
        "gini_index": ["mean", "std"],
        "reallocation_time": ["mean", "std"],
        "map_coverage": ["mean", "std"],
        "success": "mean",
        "crashed_count": ["mean", "std"],
        "recovery_count": ["mean", "std"],
        "total_bids": ["mean", "std"],
        "bids_per_auction": ["mean", "std"],
        "wasted_attempts": ["mean", "std"]
    })
    print(summary)
    
    # Save summary to CSV for the paper
    summary.to_csv("experiment_results_paper_summary.csv")
    
    # Save text summary (console output version)
    with open("experiment_results_paper_summary.txt", "w") as f:
        f.write(summary.to_string())
        
    print("Summary statistics saved to 'experiment_results_paper_summary.csv' and 'experiment_results_paper_summary.txt'.")

if __name__ == "__main__":
    main_experiment()
