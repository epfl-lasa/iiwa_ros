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
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native -faligned-new" -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

Instead of passing `-DCMAKE_CXX_FLAGS="-march=native -faligned-new"` for `SpaceVecAlg`, `RBDyn` and `mc_rbdyn_urdf` builds you can also set the `CXXFLAGS` environment variables and omit the option:

```sh
export CXXFLAGS="-march=native -faligned-new"
```

### RBDyn

```sh
cd /source/directory
git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native -faligned-new" -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### mc_rbdyn_urdf

```sh
cd /source/directory
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native -faligned-new" -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

### corrade

We are using a specific version of Corrade.

```sh
cd /source/directory
git clone https://github.com/mosra/corrade.git
cd corrade
git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5
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

Sunrise Robot Application upload
------------
You need a specific application to run on the robot side.
0. Make sure a Windows laptop is connected on the X66 Ethernet port and has IP `172.31.1.42` mask `255.255.0.0`
1. Create a new Sunrise project with Sunrise Workbench and setup an empty RobotApplication template 
2. Setup the safety configuration in `SafetyConfiguration.sconf` ([example](https://github.com/IFL-CAMP/iiwa_stack/wiki/safetyconf))
3. Replace the empty template with the [online app](https://github.com/epfl-lasa/iiwa_ros/blob/master/iiwa_driver/java/FRIOverlay.java)
4. In `StationSetup.cat`, tab `Software`, active the FRI extension, push it to the robot with `Installation` and accept the reboot question
5. Synchronise your new Sunrise project to the robot with icon `Synchronize project`
6. On the Smartpad tablet, your app must be listed in [Applications] and you must also see a new [FRI] tab

Basic Usage
--------------

### Bringup iiwa_driver

**Control IIWA with FRI**

1. Make sure your Linux/ROS laptop is connected on the KONI Ethernet port and has IP `192.170.10.1` mask `255.255.255.0`.
2. On the Smartpad tablet:

* Activate `AUT` mode (turn key right > AUT > key left)
* In `[Application]`, check yours in order to select it
* Press the mechanical `Play` button ▶

3. Within 10 seconds before the timeout, launch: `roslaunch iiwa_driver iiwa_bringup.launch`. This will connect to IIWA robot using FRI.
4 The Smartpad lets you select control mode and stiffness
5. Check that everything works if `/iiwa/joint_states` is being published and reflects the actual robot state.
6. The Smarpad'd [Application] tab must remain green. Otherwise you can press `Play ▶` again to reconnect.

In case of a hard failure, unload the app by unchecking it in [Application] before retrying.

### Gazebo Simulation

**To launch Gazebo with IIWA**

```sh
roslaunch iiwa_gazebo iiwa_gazebo.launch
```

Both of the above commands will launch IIWA in **torque-control mode**. To change the control mode (e.g., position-control), please edit the launch files to select the appropriate controller.

### MoveIt planning

If everything looks in simulation or with the FRI driver, a next step might be to try out your robot with [MoveIt](/iiwa_moveit).

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

- Konstantinos Chatzilygeroudis (costashatz@gmail.com)
- Matthias Mayr (matthias.mayr@cs.lth.se)
- Bernardo Fichera (bernardo.fichera@epfl.ch)

### Other Contributors

- Yoan Mollard (yoan@aubrune.eu)
- Walid Amanhoud (walid.amanhoud@epfl.ch)

Citing iiwa_ros
------------------

```bibtex
@software{iiwa2019github,
  author = {Chatzilygeroudis, Konstantinos and Mayr, Matthias and Fichera, Bernardo and Billard, Aude},
  title = {iiwa_ros: A ROS Stack for KUKA's IIWA robots using the Fast Research Interface},
  url = {http://github.com/epfl-lasa/iiwa_ros},
  year = {2019},
}
```

[ros]: http://www.ros.org
[gazebo]: http://gazebosim.org/
[ros control]: http://wiki.ros.org/ros_control
[kuka fri]: https://github.com/costashatz/kuka_fri
[spacevecalg]: https://github.com/jrl-umi3218/SpaceVecAlg
[rbdyn]: https://github.com/jrl-umi3218/RBDyn
[mc_rbdyn_urdf]: https://github.com/jrl-umi3218/mc_rbdyn_urdf
[robot_controllers]: https://github.com/epfl-lasa/robot_controllers
[corrade]: https://github.com/mosra/corrade
[iiwa_stack]: https://github.com/IFL-CAMP/iiwa_stack
