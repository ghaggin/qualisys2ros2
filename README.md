# ROS Driver for Motion Capture Systems
This project ports ros1 [motion_capture_system](https://github.com/KumarRobotics/motion_capture_system) for Qualisys to ROS2 eloquent.

![QUALISYS Logo](http://isbs2015.sciencesconf.org/conference/isbs2015/pages/Qualisys_Logga_PMS.png)

This package contains a ROS2 driver for the **QUALISYS** motion capture systems.

## License
For the QUALISYS driver, we use the interface from [Qualisys2Ros](https://github.com/omwdunkley/Qualisys2Ros).

For the rest of the software, the license is Apache 2.0 wherever not specified.

## Compiling
Ensure that ROS2 eloquent is installed on WSL:Ubuntu 18.04 and that the environment is configured.  Clone the repo into your ros workspace and build with colcon. You can install the colcon build system [here](https://colcon.readthedocs.io/en/released/user/installation.html)

```
cd ~/ros2_ws
colcon build --packages-select mocap_base mocap_qualisys
```

## Example Usage

I got this to run on WLS 1 and 2, however, on WSL 1 I could not communicate with nodes operating in another terminal.  This happened when I opened another Ubuntu terminal (which I think is another instance of Ubuntu) and when I opened a new tmux pane in the active Ubuntu terminal.  To fixed this I switch to WSL 2 which allowed me to communicate between ROS nodes.  The only issue is that the local host works differently.  For WSL1, connect to QTM at 127.0.0.1:22222.  For WSL2 go the windows command prompt and run ```ipconfig```.  Then find the connection for ```Ethernet adapter vEthernet (WSL)```.  Use this ip address at port 22222 to connect to QTM.

**Common Parameters**

`server` (`string`)

Address of the server of the motion capture system to be connected.

`frame_rate` (`int`, `default: 100`)

The frame rate of the motion capture system

`max_accel` (`double`, `default: 10.0`)

The max possible acceleration which serves to construct the noise parameters.

`publish_tf` (`bool`, `default: false`)

If set to true, tf msgs for the subjects are published.

`fixed_frame_id` (`string`, `mocap`)

The fixed frame ID of the tf msgs for each subject. Note that the child frame id is automatically set to the name of the subject.

`model_list` (`vector<string>`, `default: []`)

A vector of subjects of interest. Leave the vector empty if all subjects are to be tracked.

**Published Topics**

`/{mocap_sys}/{subject_name}/odom` (`nav_msgs/Odometry`)

Odometry message for each specified subject in `model_list`.

**Node**

`ros2 launch mocap_qualisys qualisys.launch.py`

## FAQ

1. Will the msgs be delayed if the driver is handling several subjects?
   The driver is multi-threaded. It uses different threads to process the msg from different subjects. So, emmmmm, don't worry =).

2. How to calibrate the transformation between the subject frame (centered at the centroid of the markers) and the body frame of a robot?
   This functionality is not provided, since few people use that now. If you really want that, please consider [vicon repo of KumarRobotics](https://github.com/KumarRobotics/vicon) or [vicon_bridge repo of ethz-asl](https://github.com/ethz-asl/vicon_bridge).

## Bug Report

Prefer to open an issue. You can also send an e-mail to sunke.polyu@gmail.com.

## OSX Support

You will need to compile ROS from source for OSX first. The vicon node is supported, but the qualisys node is not. There are no current plans to extend compatibility to OSX for the qualisys driver.
