<<<<<<< HEAD
# GP-MPC Tutorial for Autonomous Racing

## Overview
This tutorial demonstrates how to use Gaussian Processes (GP) to improve Model Predictive Control (MPC) performance by learning and compensating for model mismatch in autonomous racing applications.

**Note**: This example is adapted and modified from FORCESPRO tutorial materials.

**Author**: Kim Junghyo (hyo05122@hanyang.ac.kr)
- *Feel free to contact me for any questions, feedback, or bug reports regarding the code*

## Requirements

- MATLAB R2020b or later
- Optimization Toolbox
- Statistics and Machine Learning Toolbox (for K-means clustering)
- Parallel Computing Toolbox (Optionally)

## Tutorial Structure

### Function Files 
- **`vehicleDynamics.m`** - Vehicle dynamics and simulation functions
- **`pathFunctions.m`** - Path planning and track definition functions  
- **`gpFunctions.m`** - Gaussian Process related functions
- **`mpcFunctions.m`** - MPC objective and constraint functions
- **`visualizationFunctions.m`** - All plotting and visualization functions
- **`utilityFunctions.m`** - Utility functions for parameter definition and data preparation

### Main Tutorial Scripts
- **`GP_MPC_Tutorial.mlx`** - Complete tutorial in a single script with comprehensive documentation


## Quick Start

###  Run Complete Tutorial 
```matlab
run('GP_MPC_Tutorial.mlx')
```

## Key Concepts Covered

### 1. Model Mismatch
- Difference between MPC prediction model and real plant dynamics
- Impact on control performance
- Data collection strategy for learning errors

### 2. Gaussian Process Learning
- Learning model errors from collected data
- Probabilistic predictions with uncertainty quantification
- Efficient training using K-means subset selection

### 3. GP-Enhanced MPC
- Incorporating GP predictions into MPC formulation
- Chance constraints using GP uncertainty
- Uncertainty-aware cost functions

### 4. Performance Comparison
- Baseline MPC vs GP-MPC
- Tracking accuracy improvements
- Computational overhead analysis

## Generated Files

- **`gp_mpc_data.mat`** - Collected training data from Part 1
- **`gp_models.mat`** - Trained GP models from Part 2

## Tutorial Parameters

### Vehicle Model
- Kinematic bicycle model
- 5 states: [x, y, velocity, heading, steering_angle]
- 2 inputs: [Flon, steering_rate]

### MPC Configuration
- Sampling time: 0.1 s
- Prediction horizon: 5 steps
- Simulation length: 200 steps

### GP Configuration
- Subset size: 50 training points (K-means selected)
- Squared exponential kernel
- Individual GP for each state error

## Customization

### Tuning Parameters (Part 3)
```matlab
p.beta = 2.0;                    % Chance constraint strength (95.4% confidence)
p.variance_weight = 0.05;        % Uncertainty avoidance weight
p.gp_enable_mask = [1; 1; 0; 0; 0]; % Which states to correct with GP
```

## Performance Metrics

The tutorial typically shows:
- **Tracking improvement**: reduction in lateral error RMSE
- **Computational overhead**: ~2-5x increase in computation time
- **Robustness**: Better handling of model uncertainty

## Troubleshooting

### Common Issues
1. **Missing function files**: Ensure all 6 function files are in MATLAB path
2. **Optimization convergence**: Reduce constraint tolerance if solver fails
3. **Memory issues**: Reduce `p.num_gp_points` for large datasets


## License

This tutorial is designed for educational purposes. Please maintain attribution when sharing or modifying.

