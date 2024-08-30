# kr_demo_planner: branch autonomous flight 

This is a generalized minimize-jerk based local planner for benchmark comparsion.

(revised from a fast [trajectory optimizer](https://github.com/ZJU-FAST-Lab/large_scale_traj_optimizer) )


## 0. demo visualization

This is the example usage, see video: https://www.youtube.com/watch?v=SbeC8FY-QrU

## 1. set up

You can follow the wiki [here](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Building-from-Source), or directly build all from source as:


### 1.1 Create a folder for your work environment, and clone the repo inside
```
mkdir -p ~/${YOUR_WORKSPACE_NAME}/src
cd ~/${YOUR_WORKSPACE_NAME}/src
git clone git@github.com:yuwei-wu/kr_autonomous_flight.git
```

### 1.2 Get the dependencies
```
cd ~/${YOUR_WORKSPACE_NAME}/src/kr_autonomous_flight
vcs import < external_all.yaml
vcs pull
cd ../..
```
### 1.3 Building
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## 2. run in simulator

I only test the gazebo environment because I didn't set up the license for unity simulator :)


refer [here](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Gazebo-Simulation-Setup) for more informations. 


Step 1: Launch file

```
source ~/${YOUR_WORKSPACE_NAME}/devel/setup.bash
roslaunch gazebo_utils full_sim.launch
```


Step 2: Take off:

Then, click ```motors on```. Wait for several seconds, click ```take off``` in rqt GUI, and you should see the UAV take off.


Step 3: Send waypoints:

click ```Waypoint Nav``` button on the top of RVIZ, drag it to where you want in the map, and click again to place the waypoint. Next, click ```Publish Waypoints``` on the lower-left panel of RVIZ. Finally, click ```Execute Waypoint Mission``` in the rqt GUI.


## 3. parameters for planner


I add the parameters here: ```/autonomy_core/control/control_launch/config/tracker_params_mp.yaml```, from line 45 to line 60. Usually you don't have to change anything.

