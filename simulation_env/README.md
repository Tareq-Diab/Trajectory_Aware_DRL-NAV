# simulation_env

A ROS package for simulating robot-environment interactions in reinforcement learning applications. This package provides a comprehensive framework for training RL agents in dynamic environments.

## Overview

The `simulation_env` package extends OpenAI's toolkit to develop customized simulation environments using Gazebo and ROS. It's designed to accurately represent robotic navigation problems for reinforcement learning agents, allowing the agent to:
- Interact with the environment
- Collect experiences through trial and error
- Receive balanced reward signals to guide learning

There are two main Training setups :
 1. A tunnel environemnt which main objective is to train the agent to get to the end of a tunnel filled with dynamic obstacles without collisions. This environemnt will doesnt train the agent for a parcticle real world general scenario it forces the agent to face dynamic obstacles and learn to understand trajectories, Which works very will in highlighting the strenth points of the MCOST approach compared to other approaches. 

![tunnel world env](/Images/tunnel_world.png)

 2. Two general navigation environemnt : 
 
  A. Environemnt of a 20mx20m space filled with random objects that is also randomized every epsiode with few dynamic "Moving during epsiode" obstacles. 
  
  B. Envrionment with Dynamic actors that has randomized pathes and each time the agent is spawned in a diffrerent start posiotion and has a diffrerent goal positoin. 



![](/Images/p2p_worlds.png)

## Features

- Multiple environment configurations for different learning scenarios
- Point-to-point (p2p) navigation challenges
- Dynamic obstacle environments
- Integration with OpenAI Gym for reinforcement learning compatibility
- Trajectory prediction for dynamic obstacles

## Dependencies

- ROS (tested on ROS Noetic)
- Gazebo
- Python 3.8+
- OpenAI Gym
- Navigation stack dependencies (move_base, costmap_2d, etc.)

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Tareq-Diab/Trajectory_Aware_DRL-NAV.git
   ```

2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source your workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

## Usage

### Starting the Simulation

Launch the basic simulation environment with a specific world:
```bash
roslaunch simulation_env basic_simulation.launch sim_world:=p2p
```

options for sim world are :
-p2p 
-singleAgentDynamicActorsTunnel
-singleAgentDynamicBoxesTunnel



#### To launch costmap for agents that relay on costmaps like M-COST : 

Enable simulation time:
```bash
rosparam set /use_sim_time true
```

Launch the costmap:
```bash
roslaunch simulation_env costmap.launch
```


#### To launch trajectory predection costmap node. 

For trajectory-based costmaps (for dynamic obstacle environments):
```bash
roslaunch customCostMap trajectoryCostMap.launch
```

### Available Simulation Worlds

The package includes several world configurations:
- `singleAgentdynamic_p2p`: Dynamic environment with point-to-point navigation
- `singleAgentDynamicActorsTunnel`: Environment with dynamic actors in a tunnel configuration
- `singleAgentDynamicBoxesTunnel`: Environment with dynamic box obstacles in a tunnel

### Environment Variants

Each .world environemnt has multiple ros-openai implementations for example tyhe p2p.world gazebo world can be used with :

- **diff_Robot_gym_v2_p2p** : for training a lidar agen 
- **diff_Robot_gym_v6_monoChannel_p2p** : for training a cnn agent with an obtsacle costmap observation.  
- **diff_Robot_gym_v6_Nchannel_p2p** : for training a cnn agent with a stacked previous n number of obtsacle costmap observation. 
- **diff_Robot_gym_v6_p2p** : to train an MCOST observation agent.


# Simulation Environment Variants

| Environment Name | Observation Space | Action Space | Initialization |
|------------------|-------------------|--------------|----------------|
| `diff_Robot_gym_v2` | `Box(-1, 1, (laser_res+5,), float32)` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v2`<br>`env = diff_Robot_gym_v2.DiffRobotEnv()` |
| `diff_Robot_gym_v2_p2p` | `Box(-1, 1, (laser_res+5,), float32)` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v2_p2p`<br>`env = diff_Robot_gym_v2_p2p.DiffRobotEnv(mode="actors", nactors=12)` |
| `diff_Robot_gym_v6` | `Dict({"image": Box(0, 255, (2, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6`<br>`env = diff_Robot_gym_v6.DiffRobotEnv()` |
| `diff_Robot_gym_v6_single` | `Dict({"image": Box(0, 255, (1, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6_single`<br>`env = diff_Robot_gym_v6_single.DiffRobotEnv()` |
| `diff_Robot_gym_v6_monoChannel` | `Dict({"image": Box(0, 255, (1, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6_monoChannel`<br>`env = diff_Robot_gym_v6_monoChannel.DiffRobotEnv()` |
| `diff_Robot_gym_v6_p2p` | `Dict({"image": Box(0, 255, (2, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6_p2p`<br>`env = diff_Robot_gym_v6_p2p.DiffRobotEnv(mode="actors", nactors=5)` |
| `diff_Robot_gym_v6_NChannel` | `Dict({"image": Box(0, 255, (N, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6_NChannel`<br>`env = diff_Robot_gym_v6_NChannel.DiffRobotEnv(Nchannels=4)` |
| `diff_Robot_gym_v6_NChannel_p2p` | `Dict({"image": Box(0, 255, (N, 50, 50), uint8), "scalers": Box(-1, 1, (5,), float32)})` | `Box([-1.0, -1.0], [1.0, 1.0], (2,), float32)` | `from simulation_env.src.Environment import diff_Robot_gym_v6_NChannel_p2p`<br>`env = diff_Robot_gym_v6_NChannel_p2p.DiffRobotEnv(Nchannels=4, mode="actors", nactors=5)` |

## Package Structure

```
simulation_env/
├── launch/            # Launch files for different simulation scenarios
├── config/            # Configuration files
├── map/               # Map files for navigation
├── param/             # Parameter files for navigation stack
├── rviz/              # RViz configuration files
├── src/               # Source code
│   ├── Environment/   # RL environment implementations
│   └── trajectory/    # Trajectory prediction for dynamic obstacles
├── urdf/              # Robot model definitions
└── worlds/            # Gazebo world files
```



Different environment variations offer different observation spaces, action spaces, and reward structures to experiment with different RL approaches.

## Dynamic Obstacle Handling

The package includes utilities for dynamic environment management:
- `dynamicEnvironmentManger.py`: Manages dynamic objects in the environment
- `dynamicEnvironmentUtils.py`: Utility functions for dynamic environments
- `dynamicObjectPredictor.py`: Predicts trajectories of dynamic obstacles
- `trajectoryCostmap.py`: Creates costmaps based on predicted trajectories

## Customization

Environment parameters can be customized through YAML files:
- `dynamicElements.Yaml`: Configure dynamic elements in the environment
- `robotSpawn.Yaml`: Configure robot spawn positions
- `obstacles.yaml`: Configure static obstacles
- `nav_goals.yaml`: Configure navigation goals



## Citation
If you find this work useful in your research, please consider citing our paper:

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

## Acknowledgments

This work extends OpenAI's toolkit =D.
