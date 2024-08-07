# Markov Decision Process (MDP) Solver

This project implements various algorithms and techniques for solving and analyzing Markov Decision Processes (MDPs), with a focus on the maximum probability problem and trap detection.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Project Structure](#project-structure)
5. [Key Components](#key-components)
6. [Algorithms](#algorithms)
7. [Usage](#usage)
8. [Contributing](#contributing)

## Overview

This MDP solver implements several algorithms for policy optimization, trap detection, and transition probability estimation. It includes implementations of reinforcement learning techniques such as Q-learning, as well as graph-based algorithms like Tarjan's algorithm for detecting strongly connected components.

## Features

- Maximum probability (MAXPROB) calculation
- Trap detection and elimination
- Transition probability estimation
- Q-learning implementation
- Various sampling methods (UCB, Thompson Sampling)
- Regret calculation for Thompson Sampling
- Strongly Connected Component (SCC) detection using Tarjan's algorithm

## Dependencies

- C++ Standard Library
- PCH (Precompiled Headers)
- Custom modules: `action.h`, `global_definitions.h`, `state.h`

## Project Structure

The project is divided into several C++ files, each containing specific functionalities:

- `main.cpp`: Entry point of the program
- `action.cpp`: Implementation of action-related functions
- `state.cpp`: Implementation of state-related functions
- `algorithms.cpp`: Implementation of core algorithms (MAXPROB, Trap detection, etc.)
- `sampling.cpp`: Implementation of sampling methods and probability estimation

## Key Components

### Network Class

The `Network` class represents the MDP and provides methods for:

- Forward pass calculations
- Likelihood and prior calculations
- Parameter management

### State Class

The `State` class represents individual states in the MDP and includes:

- Action management
- Q-value storage
- Goal probability calculations

### Action Class

The `Action` class represents actions available in each state and stores:

- Transition probabilities
- Next state information

## Algorithms

### MAXPROB

Calculates the maximum probability of reaching the goal state for each state in the MDP.

### Trap Detection and Elimination

Uses Tarjan's algorithm to detect strongly connected components (SCCs) and identify trap states.

### Q-learning

Implements Q-learning for policy optimization with customizable learning rates and discount factors.

### Transition Probability Estimation

Includes methods for estimating transition probabilities using various sampling techniques:

- UCB (Upper Confidence Bound)
- Thompson Sampling

## Usage

To use the MDP solver:

1. Initialize the state space and action sets
2. Set up the Network with appropriate parameters
3. Run the desired algorithm (e.g., MAXPROB, Q-learning)
4. Analyze the results (e.g., optimal policy, goal probabilities)

Example:

```cpp
vector<state*> states = initializeStates();
Network mdp(layerInfo, inputTrain, inputTest, labels);
MAXPROB(threshold, states);
eliminateTrap(states, &size, scc_set);
qlearning(episodes, discount, states);
