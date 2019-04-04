#ifndef IIWA_CONTROL_CUSTOM_JOINT_STATE_CONTROLLER_H
#define IIWA_CONTROL_CUSTOM_JOINT_STATE_CONTROLLER_H

// ROS headers
#include <ros/node_handle.h>
// ros control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// msgs
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

// URDF
#include <urdf/model.h>

// iiwa_tools
// #include <iiwa_tools/GetGravity.h>
#include <iiwa_tools/GetJacobian.h>
#include <iiwa_tools/GetFK.h>

#include <iiwa_tools/iiwa_tools.h>

// Control stack headers

namespace iiwa_control {

    class CustomJointStateController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
    public:
        CustomJointStateController();
        ~CustomJointStateController();

        bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh_root, ros::NodeHandle& nh_controller);

        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

        std::vector<hardware_interface::JointStateHandle> joints_;

        unsigned int n_joints_;

        std::vector<std::string> joint_names_;

    protected:

        // URDF
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

        iiwa_tools::GetFK fk_srv_;
        Eigen::Vector3d omega_, v_, x_;
        Eigen::Vector4d q_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > pub_pose_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > pub_twist_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_joints_;

        iiwa_tools::GetFK::Request _requestFK;
        iiwa_tools::GetFK::Response _responseFK;
        iiwa_tools::GetJacobian::Request _requestJacobian;
        iiwa_tools::GetJacobian::Response _responseJacobian;

        ros::Time last_publish_time_;
        double publish_rate_;

        iiwa_tools::IiwaTools server;
    };
} // namespace iiwa_control

#endif