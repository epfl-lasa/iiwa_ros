# IIWA MoveIt package

Starting the MoveIt package requires that you have first setup your robot and environment as specified in the main [README](/README.md#ros-stack-for-kukas-iiwa-robots).

It can then be started on a real robot by activating the `driver:=true` option:
```sh
roslaunch iiwa_moveit demo.launch driver:=true
```
Your robot will release brakes at startup of MoveIt.
