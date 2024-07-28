# time_opt_trajs
This is the trajectory generator for DodgeDrone Competition. 

Check this video: https://youtu.be/pTRV9kNtkNw


## usage

1. clone and complie the kr mav control and this repo within the same workspace (src) of the agile flight.

```
git clone -b dodge git@github.com:yuwei-wu/kr_mav_control.git
git clone -b dodge git@github.com:yuwei-wu/DoubleDescription.git
```

2. run the code, you can directly copy the launch_bench.bash to the vitfly folder and run it
in one terminal, run:
```
 bash launch_bench.bash 1 vision
```

open another terminal, run:
```
roslaunch plan_manage sim.launch
```


3. navigate the drone

it will fly forward as the Vitfly. You can adjust the forward distance and goal. Let me know your desired velocity and there might be some paramters to be tuned.


## adjustment


1. you can take a look at the sim.launch and local_planner.launch, there are some parameters you can change.

use this to adjust the next goal point distance. Now it's directly set as map boundary
```
    <param name="forward_dist"  value="80.0" type="double"/>
```
you can set the map range to set the longest position you want to reach



2. The trajectory is sent to the "kr_mav_control" and directly publish the command, you can go to kr_mav_control, and take a look at "poly_tracker.cpp" to see how to adjust the command. It can be converted to so3 command and other low-level one.

3. the package "local_mapping" has the details about how to deal with the depth image and the obstacles ground truth. 


## topics

- trgger: /kingfisher/start_bench

- the point cloud map: /kingfisher/opt_planner_node/grid_map/occupancy_inflate

- trajectory: /kingfisher/opt_planner_node/optimal_list

- local goal: /kingfisher/opt_planner_node/local_goal_point

and you can explore other topics also.
