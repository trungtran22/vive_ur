# vive_urbot
## Controlling Universal Robots with HTC VIVE - tf2_ros - ROS Noetic
- This package is inspired and developed based on [robosavvy](https://github.com/robosavvy) for **Connection with VR server** [vive_ros](https://github.com/robosavvy/vive_ros) and Mr. Jonathan Österberg's Master Thesis for the **Publisher** ([moveit_vive](https://github.com/Machine-Jonte/moveit_vive)). 
- This Package will perform **Transformation** from VIVE Controller's pose frame to UR's base_link using **tf2_ros** (Python) instead of hard coding for robot's pose, but calibration is needed.
- The version only publish information from the **right controller (pose and trigger button)**.
### Calibration VIVE position to UR10e postition 
You will need to adjust these values in the code for the calibration. It's based on the position of the controller to the robot. Just try and change it until it works perfectly:
```
    t.transform.translation.x = -1.0 
    t.transform.translation.y = -1.0
    t.transform.translation.z = -0.8
```
### Adjustments for every UR types:
Another version is provided for control all other UR Robot with different Reachability. Adjust the SCALE_FACTOR in the code vive_scale_ur.py and run:
```
rosrun vive_ur10e vive_scale_ur.py
```
### Installation VIVE SDK [vive_ros](https://github.com/robosavvy/vive_ros):
```
  cd ~
  mkdir libraries
  cd libraries
  git clone https://github.com/ValveSoftware/openvr.git -b v1.3.22
  cd openvr
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release ../
  make
```
Copy the file 60-HTC-Vive-perms.rules to the folder /etc/udev/rules.d. Then run:
```
  sudo udevadm control --reload-rules && sudo udevadm trigger
```
Install **Steam** and **SteamVR** in Ubuntu Software.

### Run the package:
Connect to Steam VR Server
```
roslaunch vive_ur10e server_vr.launch
```
Run Publisher:
```
rosrun vive_ur10e vive_pose
```
Init MoveIT:
```
roslaunch ur10e_moveit_config demo.launch
```
Give permission to vive_ur10e_tf.py. Then run:
```
rosrun vive_ur10e vive_ur10e_tf.py
```
Close the server:
```
rosrun vive_ur10e close_servervr.sh
```
### Result:
- Visualization in RViz:
![](https://github.com/trungtran22/vive_ur10e/blob/main/docs/vive_controlling_2.gif)
- Implementation on UR3e: I slowed down the robot velocity to avoid damage 



