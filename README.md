# Trajectory-Aware Deep Reinforcement Learning Navigation Using Multichannel Cost Maps

![alt](/Images/1.gif)

Welcome to the official repository for **Trajectory-Aware Deep Reinforcement Learning Navigation Using Multichannel Cost Maps**, published in the *MDPI Robotics Journal*.
This Project implements The algorithm descriped in the paper for autonomous navigation while avoiding dynamic obstacles by leveraging a spatial temporal multichannel costmap. 


## Key Features

- **Multichannel Cost Map Integration**: Spatial-Temporal cost map representation using multiple channels for enhanced environmental awareness.
- **Trajectory Prediction**: Real-time prediction of dynamic obstacle movements for proactive navigation. 
- **Deep Reinforcement Learning**: DRL algorithms (SAC) optimized for dynamic navigation scenarios.
- **ROS Integration**: Seamless integration with ROS ecosystem for real-world deployment.
- **Multiple Environment Variants**: Various simulation environments for comprehensive testing.

## Paper Abstract

This work addresses the challenge of autonomous navigation in dynamic environments by introducing a novel trajectory-aware deep reinforcement learning approach. Our method utilizes multichannel cost maps that incorporate predicted trajectories of dynamic obstacles, enabling more informed decision-making and safer navigation paths.

![](/Images/p2p_overview.gif)


## Repository Structure 

```
.
├── actor_collisions          # responisble for caclulating collisions with dynamic actors.
├── customCostMap             # Trajectory-based cost map implementation.
├── diffRobot                 # Robot URDF and configuration files.
├── gazebo-ros-actor-plugin
├── Images                 
├── models                     # Pre-trained models and checkpoints.
├── p2p_navigator              # Navigation inference and deployment package.
├── scripts                    # Utility scripts and tools.
├── simulation_env             # ROS package for simulation environments.
└── src

```

## Package Overview

### simulation_env
A ROS package for simulating robot-environment interactions in reinforcement learning applications. Features multiple environment varies in scenarios and objectives. All inviournementrs are compatible with openAI and developed to run in gazebo-ros. 

**Key Components:**
- Multiple world configurations (p2p, tunnel environments)
- Dynamic obstacle environments , and a control environemnts without the dynamic obstacles for clear comparison. 
- Integration with OpenAI Gym for RL compatibility
- Various observation and action space configurations

### navigator
Package for deploying trained models in navigation inference mode. Provides real-time navigation capabilities using the trained DRL agents.

**Features:**
- Model inference and deployment
- Script to generate onnx and tensorRT engine for deployment on edge devices. 
- Real-time navigation decision making.
- Integration with ROS navigation stack
- Integration with RVIZ for vizualization and send goals. 

### diffRobot
Contains robot model definitions, URDF files (exported from solidworks), and configuration parameters for the differential drive robot used in experiments. 

**Includes:**
- Robot URDF and mesh files
- Sensor configurations
- Physical parameters and constraints
- Gazebo integration files

### customCostMap
Implementation of Multichannel costmap that incapsulates spatial (obstacle costmap) temporal (obstacle predicted trajectories) information. 

**Features:**
- Dynamic obstacle tracking
- Trajectory prediction algorithms
- Multichannel cost map calculation

## Installation

### Prerequisites
- Ubuntu 20.04 LTS 
- ROS Noetic
- Python 3.8+
- Gazebo 11
- CUDA-capable GPU (recommended)

### Ubuntu 20.0 native 

1. **Clone the repository:**
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Tareq-Diab/Trajectory_Aware_DRL-NAV.git
   cd Trajectory_Aware_DRL-NAV
   git pull --recurse-submodules
   ```

2. **Create the Conda environment:**
   ```bash
   conda env create -f environment.yml
   ```
3. **Activate the environemnt:** 
   ```bash
   conda activate MCOST
   ```
3. **Build the workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Verify installation:**
   ```bash
   roslaunch simulation_env basic_simulation.launch sim_world:=p2p use_gui:=true
   ```

### docker (Recommended)

1. Pull the docker image :
```
docker pull tareqdiab/trajectory_aware_drl-nav
```
2. Clone this repo 
```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Tareq-Diab/Trajectory_Aware_DRL-NAV.git
   cd Trajectory_Aware_DRL-NAV
   git pull --recurse-submodules
```
2. Run the container 
```
xhost +local:docker

docker run -it --gpus all --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -v {path to the repo}:/root/deep_rl_ws trajectory_aware_drl-nav:latest
```
3. Inside that container navigate to the repo and build it 
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
4. verify simulation is working 
```
roslaunch simulation_env basic_simulation.launch sim_world:=p2p #you can use use_gui:=true to open gazebo gui
```


## Usage
**Note**: follwoing command works for the docker container and for native. You can run the multiple commands each into seperate terminal attached to the same docker container :

```
docker exec -it "docker container hash" /bin/bash
```


### Training a New Model

1. **Start the simulation environment:**
   ```bash
   source devel/setup.sh
   roslaunch simulation_env basic_simulation.launch sim_world:=singleAgentDynamicActorsTunnel
   ```
2.  **Start obstacle costmap for all environemnts v2 envioenmnts:**
       ```bash
       source devel/setup.sh
       rosparam set /use_sim_time true
       roslaunch simulation_env costmap.launch
       ```
3. **Start trajectory predection script**
   ```bash
   source devel/setup.sh
   roslaunch customCostMap trajectoryCostMap.launch
   ```

4. **Launch the training script:**
   ```bash
   conda activate MCOST
   cd scripts/
   python3 sac_*.py # * the type of agent 
   ```

### Running Inference

1. **Launch the navigation system:**
   ```bash
   roslaunch navigator inference_navigation.launch
   ```

2. **Set navigation goals:**
   ```bash
   rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "..."
   ```

### Visualization

Monitor training progress and navigation performance:
```bash
# Launch RViz for visualization
roslaunch simulation_env visualization.launch

# Monitor training metrics
tensorboard --logdir logs/
```

## Environment Variants

The repository includes multiple environment configurations for different research scenarios:

| Experiment Category | Environment Name | Environment Description | Dynamic Obstacles | Agent Types Tested | Observation Structure | Purpose |
|---------------------|------------------|------------------------|-------------------|--------------------|-----------------------|---------|
| **Environment 1: Tunnel** | Tunnel Environment | Tunnel with agent at one end, goal at other | Boxes and humanoids moving side-to-side at varying speeds (0.3-1.0x robot speed) | 4 agent types | Various observation approaches | Investigate validity of incorporating predicted trajectories for dynamic obstacle avoidance |
| **Environment 2A: Point-to-Point Simple** | P2P Simple Navigation | Random start/goal points with simple static structure | 3 dynamic obstacles with simple paths + static obstacles | 4 agent types | Various observation approaches | General navigation task evaluation with moderate complexity |
| **Environment 2B: Point-to-Point Complex** | P2P Complex Navigation | Random start/goal points with complex static structures | 12 dynamic actors with varied paths and speeds (0.3-1.3x robot speed) | 4 agent types | Various observation approaches | Realistic navigation scenarios with high complexity |

The Agent Types and Observation Structures

| Agent Name | Observation Type | Observation Components | Description | Network Architecture | Gym Environment |
|------------|------------------|------------------------|-------------|---------------------|-----------------|
| **Baseline LIDAR** | Raw LIDAR + scalars | `[dr:g, θrg, vxr, vzr, ddeviation, LLIDAR_vector]` | Uses raw 2D LIDAR data directly | Fully Connected NN | `diff_Robot_gym_v2` |
| **Single Channel Cost Map** | Obstacle cost map + scalars | `[dr:g, θrg, vxr, vzr, ddeviation, C(x,y,t)]` | Spatial information only via cost map | CNN + FC layers | `diff_Robot_gym_v6` |
| **M-COST (Proposed)** | Dual channel cost map + scalars | `[dr:g, θrg, vxr, vzr, ddeviation, C(x,y,c,t)]` | Spatial + temporal information via multichannel cost maps | CNN + FC layers | `diff_Robot_gym_v6_p2p` |
| **Stacked Cost Maps** | Multi-frame cost maps + scalars | Previous cost maps stacked with current | Temporal via frame stacking | CNN + FC layers | `diff_Robot_gym_v6_NChannel` |

more detailed overview in [The simulation env package](/simulation_env/README.md)

## Results


## M-COST Training Results Summary

### **Tunnel Environment Results**

The tunnel environment focused extensively on dynamic obstacle avoidance, where the M-COST agent outperformed all other observation methods.

**Training Performance:**
- **Target Performance**: Reward ≥400 considered acceptable
- **Training Duration**: 350,000 steps
- **M-COST Agent**: Achieved highest training rewards (~400) with fastest convergence
- **Baseline Comparison**: Raw LIDAR performed worst, while obstacle cost map and stacked observations showed similar intermediate performance

![Training Rewards - Tunnel Environment](/Images/tunnel_training_results.jpg)

![Evaluation Scores - Tunnel Environment](/Images/tunne_eval_results.jpg)

**Navigation Performance Metrics:**
- **Success Rate**: M-COST achieved 95.00% (highest among all methods)
- **Mean Distance**: ~10m (slightly longer due to maneuvering around dynamic obstacles)
- **Control Steps**: ~350 steps (more due to proactive avoidance behaviors)

![Navigation Performance - Tunnel](/Images/tunnel_performance.jpg)

### **Point-to-Point Simple Environment Results**

In environments with predominantly static obstacles and three dynamic obstacles, all observation approaches performed nearly the same, with agents using obstacle cost maps performing slightly better than LIDAR due to spatial information.

**Training Performance:**
- **Convergence**: All methods reached similar final performance (~400 reward)
- **M-COST Advantage**: Minimal in static-dominant environments
- **Training Stability**: Cost map-based methods showed more stable training curves

![Training Rewards - Simple Point-to-Point](/Images/Point-to-Point_Training_in_Changing_Environments_with_Static_Obstacles_training.png)



![Evaluation Scores - Simple Point-to-Point](/Images/Point-to-Point_Training_in_Changing_Environments_with_Static_Obstacles_eval)

### **Point-to-Point Complex Environment Results**

Training in the dense dynamic environment with 12 actors tested the agent's ability to navigate complex environments. The M-COST agent scored better results and demonstrated slight performance advantage over other approaches.

**Training Performance:**
- **Final Reward**: M-COST achieved ~300-350 reward (highest among all methods)
- **Training Stability**: More consistent performance compared to other methods
- **Convergence Rate**: Faster convergence to optimal policy

![Training Rewards - Complex Point-to-Point](/Images/p2p_training.jpg)

![Evaluation Scores - Complex Point-to-Point](/Images/p2p_eval.jpg)

**Navigation Performance Metrics:**
- **Success Rate**: M-COST achieved 66.00% vs. 52.00% (stacked), 54.00% (single cost map), 46.00% (LIDAR)
- **Mean Distance**: ~12.5m (balanced between efficiency and safety)
- **Adaptability**: Higher variance indicating more sophisticated maneuvering strategies

![Navigation Performance - Complex Environment](/Images/p2p_perf.jpg)

### **Key Training Insights**

**Environment-Specific Performance:**
1. **Static Environments**: Minimal M-COST advantage
2. **Dynamic-Heavy Environments**: Significant M-COST superiority
3. **Mixed Environments**: Moderate M-COST benefits

**Convergence Characteristics:**
- **M-COST**: Fastest convergence in dynamic scenarios
- **Baseline Methods**: Slower, less stable convergence
- **Training Efficiency**: M-COST required fewer episodes to reach optimal performance

**Performance Scaling:**
- **Simple → Complex**: M-COST advantage increases with environment complexity
- **Static → Dynamic**: M-COST benefits scale with dynamic obstacle density
- **Temporal Information Value**: Most pronounced in concentrated dynamic scenarios

### **Overall Training Success**

The M-COST approach more than doubled the convergence rate in concentrated tunnel situations and improved navigation efficiency by 35% in tunnel scenarios and 12% in dense-environment navigation compared to standard methods.

The training results demonstrate that M-COST's multichannel approach effectively captures temporal dynamics, leading to superior performance in environments where anticipating dynamic obstacle behavior is crucial for successful navigation.

## Configuration

### Environment Parameters
Modify environment configurations in:
- `simulation_env/config/dynamicElements.yaml`
- `simulation_env/config/robotSpawn.yaml`
- `simulation_env/config/nav_goals.yaml`

### Training Parameters
Adjust training hyperparameters in:
- `scripts/training/config/ppo_config.yaml`
- `scripts/training/config/environment_config.yaml`

## Contributing

We welcome contributions! 

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Documentation

- [model-zoo](/models/README.md)
- [simulation-environemnt](/simulation_env/README.md)
- [navigator](/p2p_navigator/README.md)
- [spatial temporal custom-costmap](/customCostMap/README.md)

## Citation

If you find this work useful in your research, please consider citing our paper:

```bibtex
@article{Fahmy2024,
  author    = {Tareq A. Fahmy and Omar M. Shehta and Shady A. Maged},
  title     = {Trajectory-Aware Deep Reinforcement Learning Navigation Using Multichannel Cost Maps},
  journal   = {MDPI Robotics},
  year      = {2024},
  volume    = {13}, 
  number    = {11},
  pages     = {166},
  doi       = {10.3390/robotics13110166}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- OpenAI for the Gym toolkit
- ROS community for the navigation stack
- MDPI Robotics Journal for publication


## Contact

For questions, issues, or collaborations:

- **Tareq A. Fahmy**: tareq.a.fahmy@gmail.com  

---
