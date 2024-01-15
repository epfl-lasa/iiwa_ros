# IIWA MoveIt package

Starting the MoveIt package requires that you have first setup your robot and environment as specified in the main [README](/README.md).

In simulation this is launched in two shells as follows:
```sh
roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionTrajectoryController

roslaunch iiwa_moveit demo.launch
```

Alternatively it can then be started on a real robot by activating the `driver:=true` option:
```sh
roslaunch iiwa_moveit demo.launch driver:=true
```
Your robot will release brakes at startup of MoveIt.
