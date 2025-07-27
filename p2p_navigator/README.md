# P2P Navigator

A ROS package for point-to-point navigation using Deep Reinforcement Learning (DRL) agents. This package provides deployment capabilities for pre-trained DRL models trained to navigate robots through environments with static and dynamic obstacles.

## Overview

The p2p_navigator package contains components for deploying and evaluating DRL-based navigation policies on both simulated and real robots. It provides:

- Point-to-point navigation using trained DRL models
- Handling of observations from robot sensors (lidar, odometry)
- Goal pursuit mechanisms
- Evaluation utilities for testing navigation performance
- Support for both standard and tunnel navigation scenarios
- Integration with dynamic obstacle environments

## Components

### Core Navigation Classes

- **Navigator (p2p_navigator.py)**: Main navigation class that handles state-action formulation, sensor processing, and goal pursuit for point-to-point navigation
- **Tunnel Navigator (tunnel_navigator.py)**: Specialized navigation for tunnel-like environments
- **P2P Evaluator (p2p_evaluator.py)**: Evaluation utilities for testing navigation policies

### Utility Modules

- **Goal Randomizer (`goal_randomizer.py`)**: Utilities for generating random navigation goals
- **Dynamic Environment Manager (`dynamicEnvironemntManger.py`)**: Handles dynamic obstacles in the environment
- **Various Configuration Files**: YAML files for environment setup, obstacle definitions, and robot spawning

## Installation

### Prerequisites

- ROS (tested with Noetic)
- Python 3.x
- `stable_baselines3` for DRL model loading
- Additional Python packages: numpy, scipy, etc.

### Setup

1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/your_ros_workspace/src
   git clone https://github.com/yourusername/Trajectory_Aware_DRL-NAV.git
   ```

2. Build the workspace:
   ```bash
   cd ~/your_ros_workspace
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source ~/your_ros_workspace/devel/setup.bash
   ```

## Usage

### Running the Point-to-Point Navigator

The main navigator can be used with pre-trained models:

```bash
rosrun p2p_navigator p2p_navigator.py
```

Or with custom model paths:

```bash
rosrun p2p_navigator p2p_navigator.py --model_path path/to/your/model.zip --model_type MODEL_TYPE
```

### Running the Evaluator

To evaluate a trained model's performance:

```bash
rosrun p2p_navigator p2p_evaluator.py
```

### Configuration

Various YAML files in the src directory configure the navigation behavior:

- `dynamicElements.Yaml`: Configuration for dynamic obstacles
- `obstacles.yaml`: Static obstacle definitions
- `nav_goals.yaml`: Navigation goal definitions
- `robotSpawn.Yaml`: Robot spawn positions

## Model Details

The package supports various DRL model types, with "M-COST" being the trajectory-aware approach. Pre-trained models are stored in:

- `best_p2p/`: Models for point-to-point navigation
- `best_tunnel/`: Models for tunnel navigation

For more information about these models please view then [model zoo](/models/README.md).

## License

[Your license information here]

## Contact

For questions or issues, please contact the maintainer:
- Email: tareq.a.diab@gmail.com

## Citation

```bibtex
@article{Fahmy2024,
  author    = {Tareq A. Fahmy , Omar M. Shehta and Shady A. Maged},
  title     = {Trajectory-Aware Deep Reinforcement Learning Navigation Using Multichannel Cost Maps},
  journal   = {MDPI Robotics},
  year      = {2024},
  volume    = {13}, 
  pages     = {29}, 
  doi       = {https://doi.org/10.3390/robotics13110166}
}
```