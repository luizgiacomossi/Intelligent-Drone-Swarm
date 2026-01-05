# 1. Introduction

## 1.1 Overview
The deployment of Multi-UAV (Unmanned Aerial Vehicle) systems for search and rescue (SAR) missions requires robust coordination strategies that can adapt to dynamic environments and agent failures. Traditional centralized control systems often suffer from single points of failure and communication bottlenecks. To address these challenges, this project implements a decentralized, market-based consensus approach for task allocation and target verification in a drone swarm.

The proposed solution utilizes a virtual economy where drones "bid" for search tasks (sections of the environment) based on their current state (position, battery level) and potential reward. This mechanism ensures efficient coverage while naturally handling dynamic constraints such as battery depletion or sensor failures.

*Note: While the conceptual model is decentralized (each agent calculates its own utility), the current simulation implementation utilizes a central `MarketSystem` module to manage bids and transactions for efficiency and analysis.*

## 1.2 Problem Formulation
The mission objective is to locate a dynamic target $T$ within a defined search area $\mathcal{A}$ using a swarm of $N$ drones $\mathcal{D} = \{d_1, d_2, \dots, d_N\}$. The search area is discretized into a set of grid sections $\mathcal{S} = \{s_1, s_2, \dots, s_M\}$.

The system must satisfy the following requirements:
1.  **Coverage Completeness**: The swarm must search all sections $s_j \in \mathcal{S}$ until the target is found.
2.  **Resource Efficiency**: Minimize the total distance traveled and time taken to locate the target.
3.  **Fault Tolerance**: The system must recover from partial failures (e.g., sensor malfunction, battery depletion) strategies such as *Retasking* and *Reallocation*.
4.  **Consensus Verification**: When a drone detects a potential target, the swarm must collaboratively verify the finding to avoid false positives.

## 1.3 Proposed Solution: Market-Based Consensus
We introduce a hybrid architecture combining **Market-Based Task Allocation (MBTA)** with a **Consensus-Based Verification Protocol**.

### 1.3.1 Market Mechanism
Each unsearched section $s_j$ is treated as a commodity in a virtual market. Drones act as rational agents maximizing their utility. The cost $C_{ij}$ for drone $d_i$ to acquire section $s_j$ is dynamically calculated based on:
-   **Distance Cost**: Proportional to the Euclidean distance between $d_i$ and $s_j$.
-   **Base Value**: A fixed intrinsic value of the section.
-   **Penalty Factors**: Additional costs for delayed or forced reallocation.

Upon completing a section, the drone receives a reward $R_j$, increasing its virtual budget. This incentivizes agents to choose tasks that maximize global efficiency (by minimizing their own travel costs).

### 1.3.2 Resilient Reallocation
A key contribution of this work is the **Dynamic Reallocation System**. If a drone $d_i$ experiences a failure (e.g., critical battery level, crash), its owned but uncompleted sections are immediately released back to the market. Other available drones can then bid for these released tasks, ensuring no section is left unsearched.

### 1.3.3 Decentralized Consensus
To mitigate sensor noise, a single detection event is not sufficient for mission termination. Upon detection, the finding drone initiates a *Voting Process*. A subset of nearest neighbors is recruited as verifiers. The mission concludes only when a consensus threshold (e.g., $\tau$ positive votes) is reached.
