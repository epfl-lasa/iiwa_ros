#ifndef IIWA_DRIVER_IIWA_H
#define IIWA_DRIVER_IIWA_H

// ROS Headers
#include <ros/ros.h>

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

        void init();
        void update(const ros::TimerEvent& e);
        void read(ros::Duration elapsed_time);
        void write(ros::Duration elapsed_time);

    protected:
        bool _init_fri();
        bool _connect_fri();
        void _disconnect_fri();
        bool _read_fri(kuka::fri::ESessionState& current_state);
        bool _write_fri();
        void _on_fri_state_change(kuka::fri::ESessionState old_state, kuka::fri::ESessionState current_state) {}

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
        int _num_joints;
        int _joint_mode; // position, velocity, or effort
        std::vector<std::string> _joint_names;
        std::vector<int> _joint_types;
        std::vector<double> _joint_position, _joint_position_prev;
        std::vector<double> _joint_velocity;
        std::vector<double> _joint_effort;
        std::vector<double> _joint_position_command;
        std::vector<double> _joint_velocity_command;
        std::vector<double> _joint_effort_command;
        std::vector<double> _joint_lower_limits;
        std::vector<double> _joint_upper_limits;
        std::vector<double> _joint_effort_limits;

        // Controller manager
        std::shared_ptr<controller_manager::ControllerManager> _controller_manager;

        // FRI Connection
        kuka::fri::UdpConnection _fri_connection;
        kuka::fri::ClientData* _fri_message_data;
        kuka::fri::DummyState _robotState; //!< wrapper class for the FRI monitoring message
        kuka::fri::DummyCommand _robotCommand; //!< wrapper class for the FRI command message
        int _message_size;
        bool _idle;

        int _port;
        std::string _remote_host;

        // ROS communication/timing related
        ros::NodeHandle _nh;
        ros::Timer _update_timer;
        ros::Duration _control_period;
        double _control_freq;
    };
} // namespace iiwa_ros

#endif