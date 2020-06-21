# Position-based visual servoing + other stuff using ROS (Kinetic version)

ROS workspace has been used to take advantage of the features provided by ROS (packages, nodes, topics, services, bags etc).

Although the project at this stage is still plain C++ without any ROS libraries ,only PCL libraries atm, but they can surely be used from now on.

**My Setup**:

1. Ubuntu 16.04
2. ROS kinetic
3. PCL library
4. Microsoft Kinect XBOX-360 (used for capturing 3D data)
5. Freenectdrive + rviz (allows to capture data from kinect)

Files of interest for Visual servoing are InteractiveICP.cpp and InteractiveICP.hpp .
(others files are about normal ROS stuff)

How to execute (use Terminator with atleast 2 terminals open):
```
T1:
cd ~/project/
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
catkin_make
roscore

T2:
cd ~/project/
source devel/setup.bash
rosrun ros_package InteractiveICP
```