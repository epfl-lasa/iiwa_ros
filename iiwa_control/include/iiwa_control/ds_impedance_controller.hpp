#ifndef IIWA_CONTROL_DS_IMPEDANCE_CONTROLLER_H
#define IIWA_CONTROL_DS_IMPEDANCE_CONTROLLER_H

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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// URDF
#include <urdf/model.h>

// iiwa_tools
#include <iiwa_tools/GetJacobian.h>
#include <iiwa_tools/GetFK.h>

// Control stack headers
#include <robot_controllers/low/PassiveDS.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

//Eigen
#include <Eigen/Geometry>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <iiwa_control/DSImpedance_paramConfig.h>
#include <iiwa_tools/iiwa_tools.h>



namespace iiwa_control {
    class DSImpedanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        DSImpedanceController();
        ~DSImpedanceController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);

        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

        std::vector<hardware_interface::JointHandle> joints_;

        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;

        unsigned int n_joints_;

        std::vector<std::string> joint_names_;

    protected:
        ros::Subscriber sub_command_;

        // Controller
        robot_controllers::low::PassiveDS passive_ds_;

        // Controller's settings
        unsigned int space_dim_;
        std::string operation_space_, gravity_comp_;

        // URDF
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

        Eigen::Vector3d vd_, omegad_, omega_, v_, x_;
        Eigen::Vector4d q_, qd_;
        bool firstCommand_ = false;

        double rotationalStiffness_;
        double rotationalDamping_;
        double jointLimitsGain_;
        double desiredJointsGain_;
        double jointVelocitiesGain_;
        bool useNullSpace_;

        iiwa_tools::IiwaTools tools_;

        Eigen::VectorXd j0_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > pub_pose_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > pub_twist_;


        boost::scoped_ptr< dynamic_reconfigure::Server<iiwa_control::DSImpedance_paramConfig> > _dynamicServerParam;

        // Command callback
        void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

        // Enforce effort limits
        void enforceJointLimits(double& command, unsigned int index);

        void dynamicReconfigureCallback(iiwa_control::DSImpedance_paramConfig& config,uint32_t level);
    };
} // namespace iiwa_control

#endif