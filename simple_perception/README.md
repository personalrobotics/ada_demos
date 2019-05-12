# simple_perception demo
Starts up an Aikido PoseEstimatorModule to listen for incoming detected objects to display in RViz. For a simulated robot, markers can be sent from the simulation model in `deep_pose_estimators` (see [here](https://github.com/personalrobotics/deep_pose_estimators#building-a-custom-pose-estimator)).

### Running in simulation

* Setup catkin environment and start rosmaster.
```
cd CATKIN_WS/src/
source $(catkin locate)/devel/setup.bash
roscore
```

* Start up `rviz` 

* Launch `deep_pose_estimators` and run `simple_perception`. For convenience, this can be done from a launch file:
```
roslaunch ada_launch simple_perception.launch adaReal:=false
```

* In RViz, subscribe to `/dart_markers/simple_perception`. You should see the robot. In the terminal, you should see something like:
```
[ INFO] [1550887845.170752756]: You can view ADA in RViz now.
[ INFO] [1550887845.170789120]: If running from the launch file, here are the simulated object positions (map frame) from deep_pose_estimators:
[[1.  0.  0.  0.3]
 [0.  1.  0.  0.4]
 [0.  0.  1.  0.5]
 [0.  0.  0.  1. ]]
[[1.  0.  0.  0.1]
 [0.  1.  0.  0.2]
 [0.  0.  1.  0.3]
 [0.  0.  0.  1. ]]
[ INFO] [1550887845.670942534]: Press [ENTER] to proceed:
```

* After pressing `[ENTER]`, you should see the markers appear in RViz exactly at the locations specified in the terminal (in `map` frame).

### Running on the real robot

* Make sure to set the detector topic name:
```
rosparam set /perception/detectorTopicName <my_topic_name>
```

* Start up `simple_perception`, either directly, or through the launch file:
```
roslaunch ada_launch simple_perception.launch adaReal:=true
```

* Start your detector. After pressing `[ENTER]`, you should see the markers it publishes in RViz.
