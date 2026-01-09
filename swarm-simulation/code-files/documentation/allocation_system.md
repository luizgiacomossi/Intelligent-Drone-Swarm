# Market-Based Task Allocation System

This document describes the task allocation mechanism implemented in the drone swarm simulation. The system utilizes a **Centralized Green-Market Economy** to assign search tasks to drones.

## Core Concept: The "Points" Economy

The allocation system is built on a virtual economy where drones must "pay" to perform tasks.

*   **Currency:** Each drone operates with a wallet of points (configured via `STARTING_POINTS`).
*   **Purchasing Tasks:** Segments of the search area are treated as commodities. To be assigned a search segment, a drone must "buy" it from the central market.
*   **Incentivization:** Completing a search task rewards the drone with points equal to the section's value. This "profit" mechanism replenishes the drone's budget, allowing it to bid on and acquire future tasks, creating a sustainable cycle of work and reward.

## Allocation Phases

The allocation process is divided into two distinct phases to balance initial fairness with runtime efficiency.

### 1. Initial Phase: Static Round-Robin

**When:** At the start of the mission.

**Objective:** Fair distribution of initial workload.

**Process:**
*   **Pricing:** All sections have a flat, fixed rate (`SECTION_BASE_VALUE`), typically 2.0 points. Distance to the section is ignored.
*   **Distribution:** The `MarketSystem` acts as a central auctioneer, iterating through the drones in a round-robin sequence.
*   **Action:** Each drone purchases a random available section until its initial budget is exhausted. This ensures every drone starts with a roughly equal set of tasks spread across the map.

### 2. Dynamic Phase: Distance-Based Auction

**When:** Continuously during the mission (post-initialization).

**Objective:** Optimization of travel time and swarm efficiency.

**Process:**
*   **Trigger:** The market updates whenever a task is completed or a section becomes available (e.g., released by a fault).
*   **Dynamic Pricing:** The cost of a section is no longer fixed. It is calculated dynamically for each drone based on its distance from the section center.
    $$ \text{Price} = \text{Base Value} \times \left(1 + \frac{\text{Distance}}{\text{Dynamic Price Factor}}\right) $$
    *   *Effect:* Drones are charged a premium for distant sections.
*   **Auction Logic:**
    1.  The market generates "bids" for all valid drone-section pairs.
    2.  Bids are sorted by price (lowest to highest).
    3.  Transations are executed for the lowest bids first.
    *   *Result:* This inverse-cost auction effectively assigns available sections to the **closest capable drone**, minimizing global travel time.

## Fault Tolerance & Reallocation

The market system includes robust mechanisms to handle drone failures, deadlocks, and interruptions.

*   **Section Release:**
    *   If a drone crashes, lands for an emergency, or must return to base for recharging, it automatically **releases** ownership of all its unsearched sections.
    *   These released sections immediately re-enter the market pool.

*   **Re-Auctioning:**
    *   Released sections are picked up in the next "Dynamic Phase" update.
    *   Because pricing is distance-based, the released tasks are typically acquired by neighboring drones that are currently closest to the abandoned area.

*   **Battery Penalty:**
    *   A drone that delays its return for battery recharging (timing out) incurs a specific **Penalty Cost**.
    *   To rejoin the mission, it is forced to "buy" a random section at a high cost, penalizing its efficiency metric but ensuring it remains an active participant in the swarm.

## Summary

The system is a **Centralized Auctioneer** model. Complexity is managed by a single `MarketSystem` instance that calculates costs and assigns rights. There is no peer-to-peer negotiation or gossip protocol. The "highest bidder" for any task is simply the drone that can perform it with the lowest travel cost, aligning individual "selfish" economic goals with the global objective of efficient swarm coverage.
