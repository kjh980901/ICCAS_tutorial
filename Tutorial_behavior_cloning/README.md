# Behavior Cloning Tutorial for Level-k Game-Theoretic Intersection Control

## Overview
This tutorial demonstrates behavior cloning (imitation learning) for real-time implementation of level-k game-theoretic strategies in autonomous vehicle intersection navigation. 

* Provided by: Kyoungtae Ji, Kyoungseok Han, Hanyang University, Department of Automotive Engineering, 2025. 
* Contact us: wlrudxo644@hanyang.ac.kr, kyoungsh@hanyang.ac.kr

## Requirements

- MATLAB R2020b or later
- Deep Learning Toolbox
- Statistics and Machine Learning Toolbox

## Tutorial Structure

### Main Tutorial Script
- **`BehaviorCloning_Tutorial.mlx`** - Complete tutorial demonstrating:
  1. Single episode simulation with game-theoretic control
  2. Data collection from multiple episodes
  3. Neural network training for behavior cloning
  4. Performance comparison (Game Theory vs Neural Network)
  5. Results analysis and visualization

### Core Functions
- **`runLevelKSimulation.m`** - Main simulation function supporting both game theory and neural network modes
- **`computeLevelKStrategies.m`** - Computes L0-L3 strategies using game theory
- **`computeLevelKStrategies_NN.m`** - Fast neural network-based strategy computation
- **`plotSimulationLive.m`** - Real-time animation for MATLAB Live Scripts
- **`plotSimulationResults.m`** - Displays rewards and level estimation results

### Decision Making Functions
- **`DecisionTree_L0.m`** - Level-0 decision making (myopic greedy behavior)
- **`DecisionTree_L1.m`** - Level-1 decision making (assumes opponent is L0)

### Environment Functions
- **`Motion_Update.m`** - Updates vehicle states based on actions
- **`Reward.m`** - Calculates rewards and penalties
- **`CollisionDetection.m`** - Fast collision detection using Separating Axis Theorem
- **`RoadBoundaryCheck.m`** - Checks if vehicle is within road boundaries
- **`MidlineViolationCheck.m`** - Detects midline crossing violations
- **`getCarRectangle.m`** - Creates vehicle rectangles for visualization

### Data Files
- **`Action_history.mat`** - Training data containing state-action pairs from game-theoretic simulations
- **`trained_NN_models.mat`** - Pre-trained neural networks for each vehicle-level combination (8 models total)

## Quick Start

1. Open and run the complete tutorial:
```matlab
run('BehaviorCloning_Tutorial.mlx')
```

2. For quick testing with pre-trained models:
```matlab
% Load pre-trained models
load('trained_NN_models.mat');

% Run simulation with neural network
num_episodes = 1;
human_level = 2;  % L2 strategy
random_init = false;
use_nn = 1;  % Use neural network
[X_history, R_history, Level_ratio_history, Action_history, avg_compute_time] = ...
    runLevelKSimulation(num_episodes, human_level, random_init, use_nn, NN_models);
```

## Key Concepts

### Level-k Game Theory
- **L0**: Myopic greedy agent (no strategic reasoning)
- **L1**: Assumes opponent is L0
- **L2**: Assumes opponent is L1
- **L3**: Assumes opponent is L2

### State-Action Space
- **State**: 8D vector `[x1, y1, θ1, v1, x2, y2, θ2, v2]`
- **Actions**: 6 discrete actions
  1. Maintain current state
  2. Turn left
  3. Turn right
  4. Accelerate
  5. Decelerate
  6. Hard brake

### Neural Network Architecture
- 8 separate networks (one per vehicle-level combination)
- Input: 8D state vector
- Output: 6D action probabilities
- Architecture: Feed-forward network with ReLU activations

## Performance Metrics

### Computational Efficiency
- **Game Theory**: O(6^4) complexity for 4-step lookahead
- **Neural Network**: O(1) complexity
- **Speedup**: 15-20x faster with neural network

### Accuracy
- **Training Accuracy**: ~90%
- **Policy Fidelity**: Replication of game-theoretic decisions

## Customization

### Simulation Parameters
```matlab
human_level = 2;        % Human driver level (0-3)
random_init = false;    % Random initial conditions
use_nn = 1;            % 0: Game theory, 1: Neural network
```

### Training Parameters
- Hidden layers: [32, 64, 32]
- Training epochs: 500
- Learning rate: 0.01
- Validation split: 20%

## Visualization

The tutorial generates:
- Real-time simulation animations
- GIF outputs for each simulation
- Performance comparison plots
- Training convergence graphs

## Notes

- Control & simulation time step: 0.25s (computes new action every step)
- Game theory prediction time step: 0.5s (for 4-step lookahead = 2s)
- Episode completion: Vehicle 1 y > 24 AND Vehicle 2 x > 24
- Collision penalty: -10000
- Safety zone violation: -1000

## Troubleshooting

1. **Missing functions**: Ensure all function files are in MATLAB path
2. **Neural network training**: Requires `Action_history.mat` for training data
3. **Visualization issues**: Check that figure docking is enabled

## License

This tutorial is designed for educational purposes. Please maintain attribution when sharing or modifying.