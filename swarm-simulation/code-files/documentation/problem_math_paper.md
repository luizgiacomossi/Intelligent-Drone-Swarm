\section{Problem Formulation \& Mathematical Model}

We consider the problem of coordinating a swarm of $N$ autonomous agents, $\mathcal{D} = \{d_1, \dots, d_N\}$, to search a bounded area $\mathcal{A} \subset \mathbb{R}^2$ discretized into $M$ non-overlapping sectors $\mathcal{S} = \{s_1, \dots, s_M\}$. The objective is to maximize the covered area while minimizing time and energy, subject to dynamic constraints imposed by agent failures and limited communication ranges.

\subsection{Market-Based Task Allocation}
The core coordination mechanism is a decentralized virtual economy where agents bid for search sectors. Each drone $d_i$ evaluates the cost $C_{ij}(t)$ to acquire a target sector $s_j$ based on a trade-off between task value and travel distance:
\begin{equation}
    C_{ij}(t) = V_{base} \cdot \left( 1 + \frac{\| \mathbf{p}_i(t) - \mathbf{p}_{s_j} \|}{\delta} \right)
    \label{eq:cost}
\end{equation}
where $V_{base}$ is the intrinsic utility of a sector, $\mathbf{p}_i(t)$ is the agent's position, $\mathbf{p}_{s_j}$ is the sector centroid, and $\delta$ is a scaling factor.

The allocation follows a \textit{Global Greedy Auction} principle. At each decision epoch, the system assigns the pair $(d^*, s^*)$ that minimizes global cost, subject to the agent's budget constraint $W_i(t)$:
\begin{equation}
    (d^*, s^*) = \arg\min_{(d_i, s_j) \in \mathcal{D} \times \mathcal{S}} C_{ij}(t) \quad \text{s.t.} \quad W_i(t) \ge C_{ij}(t)
\end{equation}
This formulation ensures that tasks are allocated to the most suitable agents (closest and solvent), while the budget verification prevents resource monopolization by any single agent.

\subsection{Distributed Consensus Protocol}
To minimize false positives during target identifying, we employ a spatial voting mechanism. Upon detection of a potential target by agent $d_{det}$, a verification set $\mathcal{V}$ of the $k$ nearest neighbors is recruited. A target is confirmed only if the number of positive confirmations $N_{pos}$ satisfies the strict consensus rule:
\begin{equation}
    N_{pos} \ge |\mathcal{V}|
\end{equation}
This requirement guarantees unanimity among local observers before a global target declaration is made.

\subsection{Collision Avoidance}
Safety is enforced via a decentralized Artificial Potential Field (APF) controller. The repulsive force $\mathbf{F}_{rep}$ acting on agent $d_i$ from a neighbor $d_j$ is defined by an inverse-square law within a safety radius $r_{safe}$:

\begin{equation}
    \mathbf{F}_{drone}(d_i, d_j) = 
    \begin{cases} 
        k_{rep} \left( \frac{1}{d_{ij}} - \frac{1}{r_{safe}} \right)^2 \frac{\mathbf{p}_{ij}}{d_{ij}} & \text{if } d_{ij} < r_{safe} \\
        0 & \text{otherwise}
    \end{cases}
    \label{eq:repulsion_force_clean}
\end{equation}

\noindent where $d_{ij} = \| \mathbf{p}_{ij} \|$ is the Euclidean distance derived from the relative position $\mathbf{p}_{ij} = \mathbf{p}_i - \mathbf{p}_j$. This reactive force ensures collision-free trajectories without the need for computationally expensive trajectory optimization.
