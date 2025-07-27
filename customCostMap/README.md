# Custom Cost Map Package

The custom costmap packages s responisble for calulcating transformations for the detected/simulated dynamic obstacles. It also predectes their Trajectories and project these trajectories into temporal costmap channel of the MCOST. 

## Features

- Publishes transformations for dynamic obstacles (e.g., animated boxes, humanoid actors) in the environment.
- Uses Gazebo's `/gazebo/model_states` topic to retrieve obstacle positions and orientations.
- Supports broadcasting transformations for both static and dynamic frames.
- Provides a node for managing dynamic obstacle transformations.

## Nodes

### 1. `dynamic_transform_publisher`
This node publishes transformations for a dynamic trajectory map relative to the `base_link` frame.

#### Key Features:
- Listens to the `odom` topic for robot odometry data.
- Uses the `tf` library to compute transformations between `odom` and `base_link`.
- Broadcasts the computed transformations using `tf2_ros`.
