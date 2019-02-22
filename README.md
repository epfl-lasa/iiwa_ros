iiwa_ros
========
 
ROS Stack for KUKA's IIWA robots
---------------------------------------
 
  - Use the Fast Research Interface (FRI) to connect to the robot
  - Integration with ROS control
  - Gazebo integration with gravity compensation (similar to real robot)

Requirements
-----------

iiwa_ros requires several packages to be installed in order to work properly:

* [ROS] - ROS: tested in **Melodic**, but *Kinetic* should work also
* [KUKA FRI]
* [ROS Control]
* [Gazebo] and gazebo-ros-pkgs
* [SpaceVecAlg]
* [RBDyn]
* [mc_rbdyn_urdf]

Compilation
------------

### SpaceVecAlg

```sh
cd /source/directory
git clone https://github.com/costashatz/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### RBDyn

```sh
cd /source/directory
git clone https://github.com/costashatz/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### mc_rbdyn_urdf

```sh
cd /source/directory
git clone https://github.com/costashatz/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### ROS related packages

```sh
cd /path/to/ros_workspace
# source ros workspace
catkin_make
```

Basic Usage
--------------

### Bringup nao_dcm driver
```sh
roslaunch iiwa_driver iiwa_bringup.launch
```

This will connect to IIWA robot and provide torque-based controllers.

### Gazebo Simulation

**To launch Gazebo with IIWA in torque-control**

```sh
roslaunch iiwa_gazebo iiwa_gazebo.launch
```

**To launch Gazebo with IIWA in position-control**

```sh
roslaunch iiwa_gazebo iiwa_gazebo_position.launch
```


Copyright (c) 2019, **Konstantinos Chatzilygeroudis**

[ros]: http://www.ros.org
[gazebo]: http://gazebosim.org/
[ros control]: http://wiki.ros.org/ros_control
[kuka fri]: https://github.com/costashatz/kuka_fri
[spacevecalg]: https://github.com/costashatz/SpaceVecAlg
[rbdyn]: https://github.com/costashatz/RBDyn
[mc_rbdyn_urdf]: https://github.com/costashatz/mc_rbdyn_urdf