# Voting and Consensus Mechanism

This document details the "Voting of YES" mechanism used in the drone swarm simulation for subject verification.

## Mechanism Overview

The system uses a multi-agent confirmation protocol to verify potential subject detections. When a drone detects a subject, it recruits nearby agents to verify the finding from different vantage points.

### 1. Trigger
The process is initiated when a searching drone detects a potential subject.
- **Condition**: Drone is within **0.4 units** of the subject while in its assigned search section.
- **Action**: The drone flags `SubjectFound=True` and broadcasts the potential location.

### 2. Recruitment
Upon detection, the central controller identifies the verification team:
- **Selection**: The **3 closest neighbors** to the potential target are selected as "verifiers".
- **Assignment**: These drones are temporarily pulled from their search tasks to perform verification.

### 3. Positioning
The verifiers are assigned specific hover positions to ensure a multi-perspective view (triangulation).
- **Configuration**: An equilateral triangle formation around the target.
- **Angles**: 0°, 120°, and 240°.
- **Radius**: 0.5 units from the target center.
- **Altitude**: Maintained at flight height (`FLY_HEIGHT`).

### 4. Voting Logic
The voting process in this simulation is position-based.
- **Action**: Verifiers fly to their assigned coordinates.
- **Vote Casting**: A "YES" vote is cast automatically when a verifier reaches its assigned vantage point.
- **Threshold**: The verifier must be within **0.25 units** of the target position to cast a vote.
- *Note: In this implementation, the arrival at the position is considered sufficient confirmation (simulating a successful secondary sensor reading).*

### 5. Consensus
The mission is considered successful only when full consensus is reached.
- **Requirement**: **All 3** assigned verifiers must cast a "YES" vote.
- **Outcome**: Once consensus is reached, the subject is confirmed, the mission is marked as complete, and all drones return to base.
