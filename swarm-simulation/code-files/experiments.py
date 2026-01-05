
import time
import pandas as pd
import numpy as np
import controller
import main
import sys
import os

# Ensure we can import local modules
sys.path.append(os.getcwd())

def run_experiment(num_agents, num_faults=0, max_steps=10000000, headless=True):
    """
    Runs a single simulation experiment.
    """
    # ... (existing setup) ...
    

    
    # Reset controller state
    controller.search_active = False # Will be auto-set by headless sim
    controller.mission_aborted = False
    controller.injected_fault = None
    controller.home_ready = [False] * num_agents
    controller.charged_drones = set()
    
    # Init Simulation
    sim = main.SimulationManager(num_agents=num_agents, grid_size=8, headless=headless)
    
    # Inject faults if requested (simple modification to main/controller for pre-planned faults)
    # For now, we will simulate faults by manually injecting them at step X.
    # But since controller.injected_fault is a polling mechanism, we can set it here?
    # No, we need to set it *during* the loop.
    
    fault_schedule = []
    if num_faults > 0 and num_agents >= num_faults:
         # Fault agents 0 to N-1 at step 500
         for k in range(num_faults):
             fault_schedule.append((500 + k*100, k, 1)) # Step, AgentID, HealthCode(1=BAD_BATTERY usually)
    
    step_count = 0
    while step_count < max_steps:
        # Check faults
        if fault_schedule:
            if fault_schedule[0][0] == step_count:
                step, agent_id, health = fault_schedule.pop(0)
                controller.injected_fault = (agent_id, health)
                print(f"[Exp] Injecting fault {health} to agent {agent_id} at step {step}")
        
        running = sim.step()
        if not running: # Mission complete
            break
        step_count += 1
        
        # Fast-forward: In headless mode, step() is blocking and fast. 
        # We don't sleep.
        
        # We don't sleep.
        
    # Calculate duration in simulation seconds (approx)
    # The main loop might exit early, so we rely on step_count.
    # Default frequency is ctrl_freq=60 usually.
    # We can get strict sim time from env if accessible, but step_count is good proxy.
    duration = step_count / 60.0 # sim.ctrl_freq
    sim.metrics["end_time"] = duration

    
    # Merge market metrics if not already merged (e.g. if timeout occurred)
    if "reallocation_time" not in sim.metrics:
         sim.metrics.update(sim.market.get_metrics())
         
    # duration is already calculated above

    
    # Post-process metrics
    # Gini Coefficient for workload balance
    searched = np.array(sim.metrics["sections_searched_per_drone"])
    if np.sum(searched) > 0:
        mean_s = np.mean(searched)
        gini = np.sum(np.abs(searched - mean_s)) / (2 * num_agents * mean_s)
    else:
        gini = 0.0
        
    result = {
        "num_agents": num_agents,
        "num_faults": num_faults,
        "success": sim.metrics.get("success", False),
        "duration": duration,
        "total_distance": sim.metrics["total_distance"],
        "avg_distance": sim.metrics["total_distance"] / num_agents,
        "gini_index": gini,
        "gini_index": gini,
        "sections_per_agent": str(searched.tolist()),
        "reallocation_time": sim.metrics.get("reallocation_time", 0.0),
        "cost_efficiency": sim.metrics.get("cost_efficiency", 0.0),
        "recovery_rate": sim.metrics.get("recovery_count", 0),
        "failure_reason": sim.metrics.get("failure_reason", "Unknown") if not sim.metrics.get("success", False) else None,
        "crashed_count": sim.metrics.get("crashed_count", 0),
        "failed_count": sim.metrics.get("failed_count", 0)
    }
    
    sim.env.close()
    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass
        
    return result

def main_experiment():
    results = []
    
    # Scenario A: Scalability (2 to 8 agents)
    # Using small iteration count for testing (Iter=1). 
    # let's do 3 iterations for demonstration.
    ITERATIONS = 3
    
    print("Starting Scalability Batch...")
    for n in [2, 4, 8]:
        for i in range(ITERATIONS):
            res = run_experiment(num_agents=n, num_faults=0)
            res["scenario"] = "Scalability"
            res["iteration"] = i
            results.append(res)
            print(f"Result: {res}")
            
    # Scenario B: Fault Tolerance
    print("Starting Fault Tolerance Batch...")
    for f in [1, 2]:
        for i in range(ITERATIONS):
            res = run_experiment(num_agents=8, num_faults=f)
            res["scenario"] = "FaultCoverage"
            res["iteration"] = i
            results.append(res)
            print(f"Result: {res}")

    df = pd.DataFrame(results)
    df.to_csv("experiment_results.csv", index=False)
    print("\nExperiments Completed. Saved to 'experiment_results.csv'.")
    print(df.groupby(["scenario", "num_agents", "num_faults"]).mean(numeric_only=True))

if __name__ == "__main__":
    main_experiment()
