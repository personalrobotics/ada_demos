This tutorial focuses on the feeding demo.
If you want to run something else on Ada, you will need to adapt the steps appropriately.

## Packages to install

### Ubuntu Repos
Assuming you already have `ros-melodic-desktop-full` installed, here are the additional dependencies required from the Ubuntu repos:

```
sudo apt install libdart6-all-dev libopencv-dev libblas-dev liblapack-dev libeigen3-dev ros-melodic-control-toolbox ros-melodic-ompl ros-melodic-force-torque-sensor-controller ros-melodic-srdfdom python-wstool ros-melodic-octomap-ros ros-melodic-joint-trajectory-controller ros-melodic-transmission-interface ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-image-geometry ros-melodic-diagnostic-updater ros-melodic-controller-manager ros-melodic-rviz
```
Replace `melodic` with `kinetic` if working on Ubuntu 16.04 (Xenial).


### Kinova JACO SDK (Optional: Real Robot Only)

Install the Debian Package in [this ZIP file](https://drive.google.com/file/d/0B790iVm0vRTlUjhOb0FURm9KcjA/view)

It should be in `KINOVA SDK JACO/Ubuntu/64 bits/`.

You can install it using `dpkg`:
```
sudo dpkg -i JACO1APi-5.2.0-amd64.deb
```

### PRL Git Packages

We can install these all at once with `wstool`:
```
$ git clone https://github.com/personalrobotics/pr-rosinstalls.git ~/pr-rosinstalls
$ cd my_catkin_workspace/src
$ wstool init # exclude if already have .rosinstall
$ wstool merge ~/pr-rosinstalls/ada-feeding.rosinstall
$ wstool up
```

#### Compilation Troubleshooting

* **DLIB_NO_GUI_SUPPORT**: If you get this error when building `face_detection`: un-comment the `#define` statement in `/usr/include/dlib/config.h`.

##### Additional Notes
- There are some repositories that have `ada` in their name but are out of date! Only the repositories in the rosinstall above should be required.
- `openrave` is out of date and not required for this project.
- Whenever you install something to fix dependencies, make sure to _clean_ the affected repositories before you build them again!
- Whenever you run something, make sure to _source the setup.bash_ in the workspace in _every terminal_ you use! We recommend putting it in your `~/.bashrc` file.
- If you have dartsim in the workspace, it might not link to `libnlopt` correctly and you might see an error message when compiling `libada`. When this happens, remove dartsim and install `sudo apt-get libdart6-all-dev`.

### Workspace Setup

1) Build your workspace with `catkin build`
2) Download the checkpoint by going into `src/pytorch_retinanet` and running `load_checkpoint.sh`
2) Do the same in `src/bite_selection_package`: run `load_checkpoint.sh`
3) Make sure your source `devel/setup.bash` in *every* terminal you use.

## Running the demo

1) Start `roscore`, `rviz`
2) Turn on Ada
3) After some time, home Ada with the Joystick
4) `ssh nano` (you may need to add `nano` to your `.ssh/config`). Once there, set your ROS Master using `uselovelace` or `useweebo` (or set your ROS_MASTER_URI manually), execute `./run_all.sh` or `./run_camera.sh` to start streaming RGBD data.
   * You may have to adjust the camera exposure, depending on the lighting condition. Either run `run_adjust_camera_daylight.sh` or `run_adjust_camera_all.sh` after running `run_all.sh`. Check the image stream via rviz, by adding the image topic `/camera/color/image_raw/color`. If some area is too bright and look burnt or saturated, reduce the exposure.
6) `roslaunch forque_sensor_hardware forque.launch`
6) `rosrun face_detection face_detection`
7) `roslaunch ada_launch default.launch feeding:=true`
8) `roslaunch ada_demos feeding.launch` (will quit after writing ROS parameters) Optionally run `roslaunch ada_demos data_collection.launch` after `feeding.launch` if you're doing data collection.
9) `cd ~/Workspace/ada_ws/devel/bin/` and `./feeding -af`
    * `-a`: specified that this is the real robot, and not a simulation
    * `-f`: enables the force/torque sensor on the real robot (**REQUIRED when picking up food for safety**)
    * `-c`: causes demo to proceed without the user pressing \[ENTER\] between steps
    * `-d`: Select demo to run.
10) Go to the next step by pressing Return or terminate nicely by pressing `n` and then return


## Running the Demo in Simulation

1) Start 'roscore', 'rviz', subscribe to the topic 'feeding/update/InteractiveMarkers'
2) `roslaunch ada_launch simulation.launch` (will put 2 simulated *cantaloupe* on the plate)
3) `roslaunch ada_demos feeding.launch` (will quit after writing ROS parameters) 
3) `cd ~/Workspace/ada_ws/devel/bin/` and `./feeding`

## Running with acquisition detection
To run with acquisition detection (and not require manual success/failure input from the supervisor) run
`roslaunch ada_launch default.launch feeding:=true acquisition_detection:=true`

The model files will need to be downloaded by running `ada_demos/feeding/bash_scripts/download_detector_checkpoint.sh`

## Other things to note
- In step 7, `feeding:=true` is responsible for loading the robot with camera and forque
- Step 8 loads ros parameters for the feeding executable
- When the demo exits, it shuts down some controllers that were started. If it crashes, you'll need to restart the controller node (step 7)
- If launching the controllers (step 7) doesn't work properly, chances are a `JacoHardware` node didn't exit cleanly. I recommend checking for that by turning every ros node off and running `ps -aux | grep jaco` to see if a process is still running and preventing your controllers from starting properly.
- After running the demo one time, the Joystick switches from cartesian control to joint control until you restart Ada.

##### Weebo

If you run into internet connection problems, try running `sudo route delete default gw 192.168.1.1`. In general, when running `route -n`, you should see the gateway `192.168.2.1` *above* `192.168.1.1`.

## Safety notes
- The feeding demo has collision boxes for you and your computer, so the robot shouldn't try to hit you usually. But still:
- You can stop Ada's movement by `Ctrl-C`-ing the controller node started in step 7.
- **Never use the joystick while the controllers (step 7) are running.** Both will fight over control of Ada and they will not care about collision boxes.
- Be familiar with the location of Ada's on/off-switch :)
