# time_opt_trajs
This is the trajectory generator for DodgeDrone Competition


## usage

1. clone and complie the kr mav control and this repo within the same workspace (src) of the agile flight.

```
git clone -b poly git@github.com:yuwei-wu/kr_mav_control.git
git clone -b dodge git@github.com:yuwei-wu/DoubleDescription.git
```

2. run the code 
```
roslaunch plan_manage sim.launch

```

3. navigate the drone

using the 2D nav goal to set a goal in the environment.


![](docs/eg1.gif)



## adjustment


1. you can take a look at the sim.launch and local_planner.launch, there are some parameters you can change.


2. For now I only to the velocity control, send to the "kr_mav_control" and directly publish the command, you can go to kr_mav_control, and take a look at "poly_tracker.cpp" to see how to adjust the command. It can be converted to so3 command and other low-level one.

3. the package "local_mapping" has the details about how to deal with the depth image and the obstacles ground truth. For now I only use the ground truth. (I'm not sure with some parameters related to the camera, when I figure it out, I will update that).


## visualization

- the point cloud map: /kingfisher/opt_planner_node/grid_map/occupancy_inflate

- trajectory: /kingfisher/opt_planner_node/optimal_list

- local goal: /kingfisher/opt_planner_node/local_goal_point

and you can explore other topics also.
