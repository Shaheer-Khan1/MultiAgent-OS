# Multi-Agent System Operating System

## Introduction

This project involves designing an operating system (OS) for a multi-agent system with 50 robots operating within a 100 by 100 room. Each robot is represented as a process, and threads can be created within these processes if necessary. The goal is to estimate the width of a single exit with fluctuating accuracy based on the robot's proximity to the exit. 

The system uses inter-process communication (IPC) to exchange information such as position, internal state, and estimated width among the robots. The project requires implementing synchronization mechanisms, memory management, IPC, thread management, and process scheduling.

## Scenario

- **Room Dimensions**: 100 by 100 units
- **Number of Robots**: 50
- **Exit Width**: Ranges from 16 to 26 units

**Estimation Accuracy**:
1. **Distance ≤ 5 units**: Prediction accuracy ≥ 95%
2. **Distance 5 to 10 units**: Accuracy ranges from 88% to 95%
3. **Distance > 10 units**: Accuracy decreases proportionally

**Robot Behavior**:
1. Each robot communicates its estimated width to all other robots.
2. A robot retains estimates only from neighboring robots (within 5 units).
3. Robots compute an average of received estimates and their own estimation.
4. After 10 seconds, robots aggregate the average estimates into a global variable `Total_width`.
5. The average width of all robots is calculated using `Total_width`.
6. Display the estimated and true exit width, along with the difference.
