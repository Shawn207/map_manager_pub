# 3D Dynamic Mapping For Autonomous Robots: 
## A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera
This repo contains 3D [occupancy map](https://en.wikipedia.org/wiki/Occupancy_grid_mapping), dynamic obstacle detection and osbatcle trajectory prediction for autonomous navigation. The dynamic obstacles will be detected, tracked and merged into the static occupancy map with a corresponding trajectory prediction (ICRA2023)



paper link: [A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera](https://arxiv.org/abs/2209.08258)

Youtube Demo Video Link: https://www.youtube.com/watch?v=u5zblVx8KRc

https://user-images.githubusercontent.com/60746163/199142937-e9493b8b-fb23-4d75-b5fc-f5c48da14e91.mp4



You may want to link Eigen to defalt location if Eigen is not found
```
sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
```

### I. Install
```
prerequisite: Eigen, OpenCV
git clone https://github.com/Shawn207/map_manager_pub.git
cd ~/catkin_ws
catkin_make
```
### II. Run demo
Please specify the camera parameters and the depth image/pose topic in ```map_manager_pub/map_manager/cfg/dynamic_map_param```, then run the dynamic map launch: 
```
roslaunch map_manager dynamic_map.launch
roslaunch map_manager rviz.launch
```

### III. Parameters
Please find parameters in ```map_manager_pub/map_manager/cfg/dynamic_map_param.yaml``` files. The parameters are explained by comments.

### IV. ROS Topics
Subsribe the following topics for occupancy and ESDF map:
  - Localization topic: ```robot/odometry``` or ```robot/pose``` (please enter the name of your topic in the parameter files)
  - Depth camera topic: ```camera/depth``` (defined in the config file)
  
Publish the following topics:
  - occupancy map visualization: ```dynamic_map/inflated_voxel_map```
  - dynamic obstacles bounding boxes: ```dynamic_map/box_visualization_marker```
  - obstacle trajectory prediciton: ```dynamic_map/traj_marker```

### V. C++ Code API
Example code:
```
#include <map_manager/dynamicMap.h>

int main(){
  ...
  map_manager::dynamicMap m;
  m.initMap(nh);
  
  // collision checking with static obstacles given a point
  Eigen::Vector3d pos (1.0, 1.0, 1.0)
  bool hasCollision = m.isOccupied(pos);
  
  // get dynamic obstacles information(poseition, velocity, size)
  std::vector<Eigen::Vector3d> obstaclesPos;
  std::vector<Eigen::Vector3d> obstaclesVel;
  std::vector<Eigen::Vector3d> obstaclesSize;
  m.getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
  
  // get predicted trajectory
  std::vector<std::vector<std::vector<geometry_msgs::Point>>> predTraj;
  m.getTPredTraj(predTraj);
  
  ...
}
```

### VI. Reference
If you find this work useful, please cite the paper:

```
@article{xu2022real,
  title={A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera},
  author={Xu, Zhefan and Zhan, Xiaoyang and Chen, Baihan and Xiu, Yumeng and Yang, Chenhao and Shimada, Kenji},
  journal={arXiv preprint arXiv:2209.08258},
  year={2022}
}
```




