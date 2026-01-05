# 2. Mathematical Model

This section details the mathematical formulations governing the market-based task allocation and the distributed consensus mechanism.

## 2.1 System Definitions

Let the search area $\mathcal{A} \subset \mathbb{R}^2$ be discretized into a set of $M$ non-overlapping rectangular sections $\mathcal{S} = \{s_1, s_2, \dots, s_M\}$.
Let the swarm consist of $N$ autonomous drones $\mathcal{D} = \{d_1, d_2, \dots, d_N\}$.

The state of each drone $d_i$ at time $t$ is defined as $\mathbf{x}_i(t) = [p_i(t), b_i(t), h_i(t)]^T$, where:
-   $p_i(t) \in \mathbb{R}^3$: Position capability vector.
-   $b_i(t) \in [0, 100]$: Battery level.
-   $h_i(t) \in \mathcal{H}$: Health status from the set of possible fault states (e.g., OK, SENSOR_FAIL, LOW_BATTERY).

## 2.2 Market Dynamics

The core of the coordination is a virtual economy where drones trade sections.

### 2.2.1 Pricing Function
The cost $C_{ij}(t)$ for drone $d_i$ to acquire a target section $s_j$ is dynamically determined to balance the intrinsic value of the task against the travel cost. We define the pricing function as:

$$
C_{ij}(t) = V_{base} \cdot \left( 1 + \frac{\| p_i(t) - p_{s_j} \|}{\delta} \right)
$$

Where:
-   $V_{base}$ is the base utility value of searching a section (default: $2.0$).
-   $p_{s_j}$ is the centroid position of section $s_j$.
-   $\| \cdot \|$ denotes the Euclidean distance.
-   $\delta$ is a distance scaling factor (set to $10.0$ in our experiments).

This function ensures that:
1.  **Locality**: Drones prefer closer sections to minimize energy expenditure.
2.  **Coverage**: Even distant sections have a finite price, ensuring they are eventually cleared if no closer options exist.

### 2.2.2 Auction and Allocation Mechanism
The allocation strategy employs a **Global Greedy Auction** to minimize the system-wide participation cost while adhering to agent budget constraints.

**Bidding Process**:
At each decision epoch $t$, every available drone $d_i$ generates a bid $B_{ij}$ for every available section $s_j$:
$$ B_{ij} = C_{ij}(t) $$

**Allocation Rule**:
The market clears transactions by iteratively selecting the triplet $(d_i, s_j, C_{ij})$ that minimizes the cost, subject to the drone's budget capability:
$$ (d^*, s^*) = \arg\min_{(d_i, s_j) \in \mathcal{D} \times \mathcal{S}} C_{ij}(t) $$
Subject to:
$$ W_i(t) \ge C_{ij}(t) $$

This greedy approach ensures that tasks are assigned to the closest capable agents, approximating an optimal solution to the assignment problem with lower computational overhead than a full combinatorial auction.

### 2.2.3 Wallet Dynamics (Budget Constraint)
The virtual currency system acts as a resource constraint and reliability metric. The wallet balance $W_i$ of drone $d_i$ evolves according to:

$$ W_i(t_{k+1}) = W_i(t_k) - \text{Cost}_{out} + \text{Reward}_{in} - \text{Penalty} $$

Where:
*   $\text{Cost}_{out} = C_{ij}(t_{buy})$: Paid upon task acquisition.
*   $\text{Reward}_{in} = V_{section}$: Earned upon task completion. In this system, $V_{section} = C_{ij}(t_{buy})$, implementing a "fair exchange" where successful agents recover their investment.
*   $\text{Penalty}$: Deducted for forced reallocations or timeouts (e.g., failing to recharge in time), reducing the agent's future purchasing power.

## 2.3 Consensus Mechanism

To verify potential target detections, we employ a spatial voting consensus algorithm.

### 2.3.1 Trigger Condition
A voting process is initiated when a drone $d_{det}$ detects a signal with confidence $c > c_{thresh}$ within section $s_k$.

### 2.3.2 Verification Set
The system identifies a set of verifiers $\mathcal{V} \subset \mathcal{D} \setminus \{d_{det}\}$ consisting of the $k$ nearest neighbors to the detection point $T_{est}$:

$$
\mathcal{V} = \{ d_v \mid \text{rank}(\| p_v - T_{est} \|) \le k \}
$$

In our simulation, $k=3$ (Total 3 verifiers + 1 detector).

### 2.3.3 Consensus Rule
The target is confirmed if and only if the number of positive confirmations (votes) $N_{pos}$ satisfies:

$$
N_{pos} \ge |\mathcal{V}|
$$

This strict consensus rule (unanimity among recruited verifiers) minimizes false positives in high-stakes SAR environments.

## 2.4 Fault Modeling

We model faults as stochastic transitions in the drone's health state $h_i(t)$. The probability of failure follows a Poisson distribution or can be deterministically injected for experimental validation.

Upon failure detection ($h_i(t) \neq \text{OK}$), the **Retasking Protocol** is triggered:
1.  **Task Release**: $\forall s \in \mathcal{S}$ where $\text{owner}(s) = d_i$:
    $$
    \text{owner}(s) \leftarrow \text{None}, \quad \text{status}(s) \leftarrow \text{Available}
    $$
2.  **Market Update**: The released sections effectively re-enter the market at the current dynamic price, allowing healthy agents to bid for them immediately.

## 2.5 Motion Planning and Avoidance

To navigate the environment safely, each drone employs a decentralized reactive collision avoidance controller based on **Artificial Potential Fields (APF)**.

### 2.5.1 Potential Function
The total repulsive force $\mathbf{F}_{rep}$ acting on drone $d_i$ is the superposition of forces from other drones and environmental boundaries:

$$ \mathbf{F}_{rep} = \sum_{d_j \in \mathcal{N}_i} \mathbf{F}_{drone}(d_i, d_j) + \mathbf{F}_{border}(d_i) $$

The inter-agent repulsion follows an inverse-square law with a safety radius $r_{safe}$:

$$
\mathbf{F}_{drone}(d_i, d_j) = 
\begin{cases} 
k_{rep} \left( \frac{1}{\|p_{ij}\|} - \frac{1}{r_{safe}} \right)^2 \frac{p_{ij}}{\|p_{ij}\|} & \text{if } \|p_{ij}\| < r_{safe} \\
0 & \text{otherwise}
\end{cases}
$$

Where $p_{ij} = p_i - p_j$ and $k_{rep}$ is a gain constant. This ensures smooth trajectory deviations without solving complex optimization problems, maintaining real-time performance.
