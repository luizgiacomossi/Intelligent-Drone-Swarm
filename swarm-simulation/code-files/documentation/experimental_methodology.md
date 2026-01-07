# Experimental Methodology and Performance Metrics

This section details the simulation-based experimental framework designed to rigorously evaluate the proposed market-based consensus mechanism. The experiments are conducted using a high-fidelity physics-based simulation environment to assess system scalability, mission efficiency, and fault tolerance.

## 1. Experimental Setup

The simulation is implemented using `gym-pybullet-drones`, integrated with a custom decentralized market logic.

*   **Simulation Environment**: 3D physics engine (PyBullet) with `Crazyflie 2.x` dynamics.
*   **Control Frequency**: $60 \text{ Hz}$.
*   **Search Domain**: A discretized $4 \times 4$ grid (16 independent search sections).
*   **Statistical Significance**: Each experimental configuration is executed for $K=30$ independent trials to ensure statistical robustness. Results are reported as Mean $\pm$ Standard Deviation ($\mu \pm \sigma$).

## 2. Experimental Scenarios

We define two primary experimental scenarios to validate the system's performance characteristics.

### 2.1 Scenario A: Scalability Analysis
**Objective**: To evaluate the system's ability to scale with increasing swarm size without significant coordination overhead or diminishing returns.
*   **Configurations**: Swarm sizes of $N \in \{2, 4, 8\}$ agents.
*   **Conditions**: Fault-free execution.
*   **Hypothesis**: Search duration should decrease approximately linearly ($T \propto 1/N$), while the Gini coefficient should remain low, indicating effective load balancing.

### 2.2 Scenario B: Fault Tolerance and Resilience
**Objective**: To quantify the system's robustness and self-healing capabilities when subjected to abrupt agent failures.
*   **Configuration**: Fixed swarm size of $N=8$ agents.
*   **Fault Injection**:
    1.  **Single Fault**: Agent $A_0$ incurs a critical `BAD_BATTERY` fault at $t=8.0s$ (step 480).
    2.  **Multi-Fault (Staggered)**: Agent $A_0$ fails at $t=8.0s$, followed by Agent $A_1$ at $t=15.0s$ (step 900).
*   **Fault Mechanism**: Faulty agents immediately cease task execution, release all owned-but-unsearched sections back to the market, and initiate an emergency return-to-home.
*   **Hypothesis**: The market mechanism should dynamically reallocate released tasks to remaining healthy agents with minimal latency, ensuring 100% map coverage despite agent loss.

## 3. Performance Metrics

To provide a comprehensive assessment, we define the following quantitative metrics:

### 3.1 Mission Efficiency
*   **Search Duration ($T_{total}$)**: The total time elapsed from mission start ($t_0$) until all searchable sections are confirmed explored.
*   **Total Distance ($D_{swarm}$)**: The scalar sum of the Euclidean trajectories traversed by all $N$ agents, serving as a proxy for total energy expenditure:
    $$ D_{swarm} = \sum_{i=1}^{N} \int_{t_0}^{T_{total}} \| \mathbf{v}_i(t) \| dt $$
    where $\mathbf{v}_i(t)$ is the instantaneous velocity vector of agent $i$.

### 3.2 Workload Distribution
*   **Gini Coefficient ($G$)**: A measure of statistical dispersion used to quantify the inequality of task distribution among agents. It is defined based on the number of sections searched by each agent, $S_i$:
    $$ G = \frac{\sum_{i=1}^N \sum_{j=1}^N |S_i - S_j|}{2N \sum_{i=1}^N S_i} $$
    A value of $G=0$ represents perfect equality (ideal load balancing), while $G \to 1$ indicates maximal inequality.

### 3.3 Resilience Metrics (Fault Scenarios)
*   **Reallocation Latency ($L_{realloc}$)**: The time interval between a task being released by a faulty agent ($t_{drop}$) and its subsequent re-acquisition by a healthy agent ($t_{acq}$):
    $$ L_{realloc} = t_{acq} - t_{drop} $$
    This metric directly measures the responsiveness of the market mechanism.
*   **Recovery Count ($C_{rec}$)**: The absolute count of tasks that were successfully recovered (released by a faulty agent and completed by another).
*   **Crashed Count ($N_{crash}$)**: The total number of agents that entered a permanent failure state during the mission.
*   **Map Coverage ($\eta$)**: The percentage of the total grid area successfully searched by the conclusion of the experiment.
    $$ \eta = \frac{N_{searched}}{N_{total}} \times 100\% $$
    For a successful mission, $\eta$ must be $100\%$. Values $<100\%$ indicate catastrophic mission failure.
