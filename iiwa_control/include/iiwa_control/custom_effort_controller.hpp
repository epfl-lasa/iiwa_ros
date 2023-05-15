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
#ifndef IIWA_CONTROL_CUSTOM_EFFORT_CONTROLLER_H
#define IIWA_CONTROL_CUSTOM_EFFORT_CONTROLLER_H

// ROS headers
#include <ros/node_handle.h>
// ros control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// msgs
#include <std_msgs/Float64MultiArray.h>

// URDF
#include <urdf/model.h>

// Iiwa tools
#include <iiwa_tools/iiwa_tools.h>

// RobotControllers
#include <robot_controllers/AbstractController.hpp>

namespace iiwa_control {
    class CustomEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        using ControllerPtr = Corrade::Containers::Pointer<robot_controllers::AbstractController>;

        CustomEffortController();
        ~CustomEffortController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);

        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

        std::vector<hardware_interface::JointHandle> joints_;

        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;

        unsigned int n_joints_;

        std::vector<std::string> joint_names_;

    protected:
        ros::Subscriber sub_command_;

        // Controller
        ControllerPtr controller_;
        // Plugin controller manager
        Corrade::PluginManager::Manager<robot_controllers::AbstractController> manager_;

        // Controller's settings
        unsigned int space_dim_;
        unsigned int cmd_dim_;
        bool has_orientation_, null_space_control_;
        std::string operation_space_, gravity_comp_;

        // Iiwa tools
        iiwa_tools::IiwaTools tools_;

        // URDF
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

        // Null-space control
        Eigen::VectorXd null_space_joint_config_;
        double null_space_Kp_, null_space_Kd_, null_space_max_torque_;

        // Command callback
        void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

        // Enforce effort limits
        void enforceJointLimits(double& command, unsigned int index);
    };
} // namespace iiwa_control

#endif