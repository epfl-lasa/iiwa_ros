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

* [ROS] - ROS: tested in **Melodic** and **Kinetic**; *Indigo* should work also
* [KUKA FRI] - This is a modified version of the C++ FRI library provided by KUKA: unfortunately, we cannot release this code, but if you own a KUKA manipulator please contact us in order to acquire this modified library.
* [ROS Control]
* [Gazebo] and gazebo-ros-pkgs
* [SpaceVecAlg]
* [RBDyn]
* [mc_rbdyn_urdf]
* [corrade]
* [robot_controllers]

Dependencies
-------------

### KUKA FRI (Private)

```sh
cd /source/directory
git clone https://github.com/epfl-lasa/kuka_fri.git
cd kuka_fri
./waf configure
./waf
sudo ./waf install
```

### SpaceVecAlg

```sh
cd /source/directory
git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### RBDyn

```sh
cd /source/directory
git clone --recursive https://github.com/costashatz/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### mc_rbdyn_urdf

```sh
cd /source/directory
git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### corrade

```sh
cd /source/directory
git clone https://github.com/mosra/corrade.git
cd corrade
mkdir build && cd build
cmake ..
make -j
sudo make install
```

### robot_controllers

```sh
cd /source/directory
git clone https://github.com/epfl-lasa/robot_controllers.git
cd robot_controllers
mkdir build && cd build
cmake ..
make -j
sudo make install
```

Compilation
------------

```sh
cd /path/to/ros_workspace
# source ros workspace
catkin_make
```

Basic Usage
--------------

### Bringup iiwa_driver

**Control IIWA with FRI**

```sh
roslaunch iiwa_driver iiwa_bringup.launch
```

This will connect to IIWA robot using FRI.

### Gazebo Simulation

**To launch Gazebo with IIWA**

```sh
roslaunch iiwa_gazebo iiwa_gazebo.launch
```

**Both of the above commands will launch IIWA in torque-control mode (with gravity compensation enabled)! To change the control mode (e.g., position), please edit the launch files to select the appropriate controller.**

Documentation
---------------------

**UNDER CONSTRUCTION**

Contributing
---------------------

**iiwa_ros** is being actively developed. Please see [CONTRIBUTING](CONTRIBUTING.md) for more on how to help.

Acknowledgements
---------------------
The URDF description files are copied and refactored from [iiwa_stack] (by Salvatore Virga and Marco Esposito).

Authors/Maintainers
---------------------

- Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@epfl.ch)
- Bernardo Fichera (bernardo.fichera@epfl.ch)
- Walid Amanhoud (walid.amanhoud@epfl.ch)

[ros]: http://www.ros.org
[gazebo]: http://gazebosim.org/
[ros control]: http://wiki.ros.org/ros_control
[kuka fri]: https://github.com/costashatz/kuka_fri
[spacevecalg]: https://github.com/costashatz/SpaceVecAlg
[rbdyn]: https://github.com/costashatz/RBDyn
[mc_rbdyn_urdf]: https://github.com/costashatz/mc_rbdyn_urdf
[robot_controllers]: https://github.com/epfl-lasa/robot_controllers
[corrade]: https://github.com/mosra/corrade
[iiwa_stack]: https://github.com/IFL-CAMP/iiwa_stack