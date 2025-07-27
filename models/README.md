

### Model Zoo



#### Tunnel Models

![](/Images/tunnel_performance.jpg)

| Model Name                  | Number of Parameters | Avg Distance | Avg Steps | Success Rate |
|-----------------------------|-----------------------|--------------|-----------|--------------|
| sac_tunnel_MCOST.zip        | 1087464              | 10.21     | 314.38    | 75.0        |
| sac_tunnel_stacked_costmaps.zip | 1099752          | 9.52      | 285.86    | 38.0        |
| sac_tunnel_mono_costmap.zip | 1081320              | 8.53      | 263.42    | 14.00       |
| sac_tunnel_raw_lidar.zip    | 801544               | 8.56      | 267.5     | 4.00        |

---

#### P2P Models

![](/Images/p2p_perf.jpg)

| Model Name                  | Number of Parameters | Avg Distance(m) | Avg Steps | Success Rate % |
|-----------------------------|-----------------------|--------------|-----------|--------------|
| sac_p2p_MCOST.zip           | 1087464              | 11.64    | 344.21    | 66        |
| sac_p2p_stacked_costmaps.zip | 1099752             | 10.53    | 302.82    | 52        |
| sac_p2p_mono_costmap.zip    | 1081320              | 7.76     | 237.97    | 46        |
| sac_p2p_raw_lidar.zip       | 801544               | 10.94    | 310.31    | 54        |


---




### Explanation of the Columns
- **Model Name**: The name of the model file.
- **Number of Parameters**: The total number of trainable parameters in the model.
- **Avg Distance**: The average distance achieved by the model during evaluation to reach its goal .
- **Avg Steps**: The average number of steps taken by the model during evaluation to reach its goal.
- **Success Rate**: The percentage of successful episodes during evaluation.

---
