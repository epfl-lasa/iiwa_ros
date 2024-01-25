//|
//|    Copyright (C) 2019-2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Matthias Mayr
//|              Bernardo Fichera
//|    email:    costashatz@gmail.com
//|              matthias.mayr@cs.lth.se
//|              bernardo.fichera@epfl.ch
//|    Other contributors:
//|              Yoan Mollard (yoan@aubrune.eu)
//|              Walid Amanhoud (walid.amanhoud@epfl.ch)
//|    website:  lasa.epfl.ch
//|
//|    This file is part of iiwa_ros.
//|
//|    iiwa_ros is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_ros is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#include <gazebo_ros_control/default_robot_hw_sim.h>

// Iiwa service headers
#include <iiwa_tools/GetGravity.h>

// std headers
#include <vector>

namespace iiwa_gazebo {
    class GravityCompensationHWSim : public gazebo_ros_control::DefaultRobotHWSim {
    public:
        virtual bool initSim(
            const std::string& robot_namespace,
            ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model,
            const urdf::Model* const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions) override;

        virtual void readSim(ros::Time time, ros::Duration period) override;
        virtual void writeSim(ros::Time time, ros::Duration period) override;

    protected:
        // ROS related
        ros::ServiceClient _iiwa_gravity_client;
        iiwa_tools::GetGravity _gravity_srv;
    };

} // namespace iiwa_gazebo