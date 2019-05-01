#!/usr/bin/env python
# encoding: utf-8
#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Bernardo Fichera
#|              Walid Amanhoud
#|    email:   konstantinos.chatzilygeroudis@epfl.ch
#|             bernardo.fichera@epfl.ch
#|             walid.amanhoud@epfl.ch
#|    website: lasa.epfl.ch
#|
#|    This file is part of iiwa_ros.
#|
#|    iiwa_ros is free software: you can redistribute it and/or modify
#|    it under the terms of the GNU General Public License as published by
#|    the Free Software Foundation, either version 3 of the License, or
#|    (at your option) any later version.
#|
#|    iiwa_ros is distributed in the hope that it will be useful,
#|    but WITHOUT ANY WARRANTY; without even the implied warranty of
#|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#|    GNU General Public License for more details.
#|
import rosbag
import rospy
from std_msgs.msg import Float64MultiArray

# bag = rosbag.Bag('2019-02-27-11-05-45.bag')
# bag = rosbag.Bag('2019-02-27-14-33-31.bag')
# bag = rosbag.Bag('2019-02-27-14-36-35.bag')
# bag = rosbag.Bag('2019-03-04-16-58-13.bag')
bag = rosbag.Bag('2019-03-07-10-05-35.bag')

t_prev = rospy.Time(0)
i = 0
cmd_msg = Float64MultiArray()

pub = rospy.Publisher('/iiwa/PositionController/command', Float64MultiArray, queue_size=10)

rospy.init_node('replay')

for topic, msg, t in bag.read_messages(topics=['/iiwa/joint_states']):
    dt = msg.header.stamp - t_prev
    t_prev = msg.header.stamp

    # print msg.position
    cmd_msg.data = msg.position
    pub.publish(cmd_msg)

    if i>0:
        rospy.sleep(dt.to_sec())
    i = i + 1
bag.close()