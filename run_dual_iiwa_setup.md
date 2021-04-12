# Tutorial for launching multiple iiwas
This tutorial shows how to launch multiple iiwas that share the same ros master.

Prerequisites:
- installed all the packages needed for `iiwa_ros`
- ROS
- linux OS

## Connect multiple computers to the same ros master

In order to have a controller that works over multiples robots we need to connect the computers that are connected to each robot to the same ros master.
To do so the instruction written at the ros page [ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) are followed.

Prerequisites:
- ROS over all the PC
- All the PC connected to the same local area network

In the bashrc file of each computer the following command have to be written:

```
export ROS_MASTER_URI=http://<ip_master_pc>:11311
export ROS_IP=<ip_current_pc>
```
Where `<ip_current_pc>` reffers to the IP of the current PC and `<ip_master_pc>` to the IP of the computer that hosts the ros master.

## Access remotly to a PC (Optional)

This step avoids to loose time by swithcing from a PC to another.
To connect remotely to another PC on the same LAN the following code has to be written in the terminal:

```
ssh -X <pc_name>
```
e.g. ` ssh -X user_foo@ip_user_foo`

And than instert the password of the user of the PC where we want to connect.

## Launch bringup files

Now that all the PCs shares the same ros master, in the PC where we want to host the ros master we have to write the following command:
```
roscore
```
If everything works correctly in the terminal you must have the following line:
```
...
ROS_MASTER_URI=http:/<ip_master_pc>:11311/
...
```
Now that the ros master is launched we can launch the `iiwa_bringup.launch` file conresponding to each robot on the directly connected PC.

WARNING on each `iiwa_bringup.launch` file is important to select a different `robot_name` because otherwhise they will have conflicts on the respective topics.

In the package iiwa_driver there are present two example of bringup files (`iiwa7_bringup.launch` and `iiwa14_bringup.launch`)

An exeple of running a bringup would be:
```
roslaunch iiwa_driver iiwa7_bringup.launch
```