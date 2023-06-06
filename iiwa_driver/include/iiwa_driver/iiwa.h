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
#ifndef IIWA_DRIVER_IIWA_H
#define IIWA_DRIVER_IIWA_H

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <realtime_tools/realtime_publisher.h>

#include <iiwa_driver/AdditionalOutputs.h>
#include <iiwa_driver/FRIState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// FRI Headers
#include <kuka/fri/LBRCommand.h>
#include <kuka/fri/LBRState.h>
#include <kuka/fri/UdpConnection.h>

namespace controller_manager {
    class ControllerManager;
}

namespace kuka {
    namespace fri {
        class ClientData;

        class DummyState : public LBRState {
        public:
            FRIMonitoringMessage* message() { return _message; }
            void set_message(FRIMonitoringMessage* msg) { _message = msg; }
            int monitoring_message_id() { return LBRMONITORMESSAGEID; }
        };

        class DummyCommand : public LBRCommand {
        public:
            FRICommandMessage* message() { return _message; }
            void set_message(FRICommandMessage* msg) { _message = msg; }
            int command_message_id() { return LBRCOMMANDMESSAGEID; }
        };
    } // namespace fri
} // namespace kuka

namespace iiwa_ros {
    class Iiwa : public hardware_interface::RobotHW {
    public:
        Iiwa(ros::NodeHandle& nh);
        ~Iiwa();

        void init(ros::NodeHandle& nh);
        void run();
        bool initialized();

    protected:
        void _init();
        void _ctrl_loop();
        void _load_params();
        void _read(const ros::Duration& ctrl_duration);
        void _write(const ros::Duration& ctrl_duration);
        bool _init_fri();
        bool _connect_fri();
        void _disconnect_fri();
        bool _read_fri(kuka::fri::ESessionState& current_state);
        bool _write_fri();
        void _publish();
        void _on_fri_state_change(kuka::fri::ESessionState old_state, kuka::fri::ESessionState current_state) {}

        // External torque and commanding status publishers
        realtime_tools::RealtimePublisher<iiwa_driver::AdditionalOutputs> _additional_pub;
        realtime_tools::RealtimePublisher<iiwa_driver::FRIState> _fri_state_pub;
        realtime_tools::RealtimePublisher<std_msgs::Bool> _commanding_status_pub;

        // Interfaces
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::PositionJointInterface _position_joint_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        hardware_interface::EffortJointInterface _effort_joint_interface;

        joint_limits_interface::EffortJointSaturationInterface _effort_joint_saturation_interface;
        joint_limits_interface::EffortJointSoftLimitsInterface _effort_joint_limits_interface;
        joint_limits_interface::PositionJointSaturationInterface _position_joint_saturation_interface;
        joint_limits_interface::PositionJointSoftLimitsInterface _position_joint_limits_interface;
        joint_limits_interface::VelocityJointSaturationInterface _velocity_joint_saturation_interface;
        joint_limits_interface::VelocityJointSoftLimitsInterface _velocity_joint_limits_interface;

        // Shared memory
        size_t _num_joints;
        int _joint_mode; // position, velocity, or effort
        std::vector<std::string> _joint_names;
        std::vector<int> _joint_types;
        std::vector<double> _joint_position, _joint_position_prev;
        std::vector<double> _joint_velocity;
        std::vector<double> _joint_effort;
        std::vector<double> _joint_position_command;
        std::vector<double> _joint_velocity_command;
        std::vector<double> _joint_effort_command;

        // Controller manager
        std::shared_ptr<controller_manager::ControllerManager> _controller_manager;

        // FRI Connection
        kuka::fri::UdpConnection _fri_connection;
        kuka::fri::ClientData* _fri_message_data;
        kuka::fri::DummyState _robot_state; //!< wrapper class for the FRI monitoring message
        kuka::fri::DummyCommand _robot_command; //!< wrapper class for the FRI command message
        int _message_size;
        bool _idle, _commanding;

        int _port;
        std::string _remote_host;

        // ROS communication/timing related
        ros::NodeHandle _nh;
        std::string _robot_description;
        ros::Duration _control_period;
        double _control_freq;
        bool _initialized;
        bool _publish_additional_info{false};
        bool _publish_commanding_status{false};
    };
} // namespace iiwa_ros

#endif
