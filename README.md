# Markov Decision Process (MDP) Solver

This project implements various algorithms and techniques for solving and analyzing Markov Decision Processes (MDPs), with a focus on the Goal Reachability with Bounded Probability (GRBP) problem and learning algorithms.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Project Structure](#project-structure)
5. [Key Components](#key-components)
6. [Algorithms](#algorithms)
7. [Applications](#applications)
8. [Usage](#usage)
9. [Performance](#performance)

## Overview

This MDP solver implements several algorithms for solving maximum probability problems, detecting traps, and implementing learning strategies in Markov Decision Processes. The project focuses on the Goal Reachability with Bounded Probability (GRBP) model, which includes states with different transition probabilities. It provides implementations of value iteration algorithms, Q-learning, and various probability estimation techniques like Sample Mean, UCB1, and Thompson Sampling.

## Features

- Maximum probability (MAXPROB) calculation for general MDPs
- Trap detection and elimination using strongly connected components
- Various learning algorithms:
  - Q-learning with customizable learning rates
  - Sample Mean estimation with control function
  - UCB1 estimation for exploration-exploitation balance
  - Thomson Sampling using beta distributions
- Regret calculation and analysis with logarithmic convergence assessment
- Support for Goal Reachability with Bounded Probability (GRBP) models
- Visualization tools for probability values and policy transitions

## Dependencies

- C++ Standard Library
- MATLAB (for specific graph generation and analysis)
- Python with NumPy and Matplotlib (for alternative implementation)

## Project Structure

The project is organized into object-oriented components in both C++ and Python:

- Core MDP classes (State, Action)
- Algorithm implementations (MAXPROB, trap detection, learning algorithms)
- Analysis utilities (regret calculation, graph generation)
- Visualization tools for probability surfaces and transition boundaries

## Key Components

### State Class

The `State` class represents individual states in the MDP and includes:
- State number identification for tracking within the MDP
- Goal probability values updated during value iteration
- Action management for state transitions
- Counters for learning algorithms and policy evaluation
- Delta values for convergence assessment

### Action Class

The `Action` class represents actions available in each state and stores:
- Next state transitions and their probabilities
- Real and estimated transition probabilities
- Counters for state-action-state transitions
- Better/worse state categorization for informed decision making
- Methods for probability retrieval and update

## Algorithms

### MAXPROB

Implements value iteration to calculate the maximum probability of reaching the goal state, updating values until convergence based on a threshold. The algorithm:
- Performs Bellman backups iteratively
- Updates state values based on maximum attainable probability
- Converges when updates fall below a specified threshold
- Records optimal actions for policy extraction

### Trap Elimination

Uses Tarjan's algorithm to detect strongly connected components (SCCs) and identifies:
- Permanent traps (dead ends with no exit)
- Transient traps (potential exits exist but not in current policy)

The process involves:
1. Constructing a greedy graph based on current policy
2. Finding strongly connected components using Tarjan's algorithm
3. Determining which SCCs are traps by checking exit states
4. Setting goal probability to zero for permanent traps

### Learning Algorithms

- **Sample Mean Estimation**: Estimates transition probabilities using sample averages from repeated simulations
- **UCB1 Estimation**: Incorporates exploration bonuses for better probability estimation with a control function to ensure adequate sampling of all actions
- **Thomson Sampling**: Uses beta distributions to estimate transition probabilities, showing logarithmic regret growth over time
- **Q-Learning**: Reinforcement learning approach that works well with rewards proportional to transition probabilities and decreasing learning rates

## Applications

The project has been tested on several MDP types:
- GRBP with and without self-transition probabilities
- Various configurations of transition probabilities (high/low pupper and pfinal)
- Medical cancer screening MDP (preliminary implementation)

## Usage

Example setup for a simple GRBP MDP with six states:

```cpp
// Create states and actions
vector<State*> states;
for (int i = 0; i <= G; i++) {
    states.push_back(new State(i));
}

// Add next states and transition probabilities to actions
Action* cont = new Action();
Action* final = new Action();

// Configure transitions
cont->addnextstatetprob({states[i+1], states[i-1]}, {P_upper, 1-P_upper});
final->addnextstatetprob({states[G], states[0]}, {P_final, 1-P_final});

// Add actions to states
for (int i = 1; i < G; i++) {
    states[i]->addaction({cont, final});
}

// Set goal state
states[G]->goalprob = 1;

// Run MAXPROB algorithm
MAXPROB(0.0001, states);

// Print optimal policy
for (int i = 1; i < G; i++) {
    cout << "State " << i << " optimal action: " << 
        (states[i]->optimalactionindex == 0 ? "Continue" : "Final") << endl;
}
```

## Performance

The algorithms show different performance characteristics:
- Value iteration (MAXPROB) converges quickly for small MDPs
- Q-learning performs well when rewards are proportional to transition probabilities
- Thomson Sampling shows logarithmic regret, converging after approximately 700 rounds in favorable scenarios
- Learning performance varies based on transition probability configurations, with some scenarios requiring more rounds (10,000+) for convergence
