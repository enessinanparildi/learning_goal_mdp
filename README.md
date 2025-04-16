# Markov Decision Process (MDP) Solver

This project implements various algorithms and techniques for solving and analyzing Markov Decision Processes (MDPs), with a focus on the Goal Reachability with Bounded Probability (GRBP) problem and learning algorithms.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Project Structure](#project-structure)
5. [Key Components](#key-components)
6. [Algorithms](#algorithms)
7. [Usage](#usage)

## Overview

This MDP solver implements several algorithms for solving maximum probability problems, detecting traps, and implementing learning strategies. It includes value iteration algorithms, Q-learning implementation, and various probability estimation techniques like Sample Mean, UCB1, and Thompson Sampling.

## Features

- Maximum probability (MAXPROB) calculation for general MDPs
- Trap detection and elimination using strongly connected components
- Various learning algorithms:
  - Q-learning
  - Sample Mean estimation
  - UCB1 estimation
  - Thomson Sampling
- Regret calculation and analysis
- Support for Goal Reachability with Bounded Probability (GRBP) models

## Dependencies

- C++ Standard Library

## Project Structure

The project is organized into object-oriented components in both C++ and MATLAB:

- Core MDP classes (State, Action)
- Algorithm implementations (MAXPROB, trap detection, learning algorithms)
- Analysis utilities (regret calculation, graph generation)

## Key Components

### State Class

The `State` class represents individual states in the MDP and includes:
- State number identification
- Goal probability values
- Action management
- Counters for learning algorithms

### Action Class

The `Action` class represents actions available in each state and stores:
- Next state transitions
- Real and estimated transition probabilities
- Counters for state-action-state transitions
- Better/worse state categorization

## Algorithms

### MAXPROB

Implements value iteration to calculate the maximum probability of reaching the goal state, updating values until convergence based on a threshold.

### Trap Elimination

Uses Tarjan's algorithm to detect strongly connected components (SCCs) and identifies:
- Permanent traps (dead ends with no exit)
- Transient traps (potential exits exist but not in current policy)

### Learning Algorithms

- **Sample Mean Estimation**: Estimates transition probabilities using sample averages
- **UCB1 Estimation**: Incorporates exploration bonuses for better probability estimation
- **Thomson Sampling**: Uses beta distributions to estimate transition probabilities
- **Q-Learning**: Reinforcement learning approach that works well with rewards proportional to transition probabilities

## Usage

Example setup for a simple GRBP MDP with six states:

```cpp
// Create states and actions
// Add next states and transition probabilities to actions
// Add actions to states
// Run desired algorithm (MAXPROB, Q-learning, etc.)
// Analyze results (optimal policy, goal probabilities)
```

The project supports various MDP types and can be extended to model different scenarios, including medical applications like cancer screening.
