/**
 * @file iiwa_eef_node.cpp
 * @author William Galand (william.galand@gmail.com)
 * @brief This node is a utility node for control tool control. It calculates
 *  and publishes the end-effector (EEF) state using the forward kinematics, the
 *  jacobian and pseudo-inverse jacobian.
 * @version 0.1
 * @date 2024-02-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iiwa_tools/iiwa_service.h>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

// Messages
#include <iiwa_driver/AdditionalOutputs.h>
#include <iiwa_tools/MsgEefState.h>
#include <sensor_msgs/JointState.h>

#define QUEUE_SIZE 1
#define NB_DIM 6

typedef message_filters::sync_policies::
    ApproximateTime<sensor_msgs::JointState, iiwa_driver::AdditionalOutputs>
        AprxmtSyncJointState;

template <class MatT>
Eigen::Matrix<typename MatT::Scalar,
              MatT::ColsAtCompileTime,
              MatT::RowsAtCompileTime>
pseudo_inverse(const MatT& mat)
{
    // More efficient (~2x) and numerically stable than with jacobiSvd
    return mat.completeOrthogonalDecomposition().pseudoInverse();
}

/**
 * @brief Transforms the robot joint states to the end effector state.
 * 
 */
class EefPublisher
{
 public:
    EefPublisher() : _syncJointState(AprxmtSyncJointState(QUEUE_SIZE)) {}
    ~EefPublisher() = default;
    /**
     * @brief Initialize the instance components, subscribers and publishers.
     * 
     * @param nh Handle to the ROS node on which the instance is running.
     * @return true If successful.
     * @return false Otherwise.
     */
    bool init(ros::NodeHandle& nh)
    {
        // RBDyn

        // Get the URDF XML from the parameter server
        std::string ns = nh.getNamespace() + "/";
        std::string robot_description = ns + "robot_description";
        std::string full_param, urdf_string;

        // gets the location of the robot description on the parameter server
        if (!nh.searchParam(robot_description, full_param))
        {
            ROS_ERROR_ONCE_NAMED(
                "EefPublisher",
                "Could not find parameter %s on parameter server",
                robot_description.c_str());
            return false;
        }

        // search and wait for robot_description on param server
        while (urdf_string.empty())
        {
            ROS_INFO_ONCE_NAMED("EefPublisher",
                                "EefPublisher is waiting for model URDF in "
                                "parameter [%s] on the ROS param server.",
                                robot_description.c_str());

            nh.getParam(full_param, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("EefPublisher",
                              "Received urdf from param server, parsing...");

        // Get the end-effector
        std::string end_effector;
        nh.param<std::string>("params/end_effector",
                              end_effector,
                              ns.substr(1, ns.length() - 2) + "_link_ee");

        // Initialize iiwa tools
        _iiwaTools.init_rbdyn(urdf_string, end_effector);

        // Init subscribers
        _subJointState.subscribe(nh,
                                 "joint_states",
                                 QUEUE_SIZE,
                                 ros::TransportHints().reliable().tcpNoDelay());
        _subAdditionalOutputs.subscribe(
            nh,
            "additional_outputs",
            QUEUE_SIZE,
            ros::TransportHints().reliable().tcpNoDelay());

        // Synchronizer
        _syncJointState.connectInput(_subJointState, _subAdditionalOutputs);

        // Init publishers
        _pubEefState.init(nh, "eef_state", QUEUE_SIZE);

        return true;
    }
    /**
     * @brief Callback for synchronized joint state subscribers.
     * 
     * @param msgJointState State of the robot's joints.
     * @param msgAddOutputs External torque on the robot's joints.
     */
    void onJointState(
        const sensor_msgs::JointStateConstPtr& msgJointState,
        const iiwa_driver::AdditionalOutputsConstPtr& msgAddOutputs)
    {
        // Extract joint state from msgs
        Eigen::Map<const Eigen::VectorXd> position(
            msgJointState->position.data(), msgJointState->position.size()),
            velocity(msgJointState->velocity.data(),
                     msgJointState->velocity.size()),
            torque(msgJointState->effort.data(), msgJointState->effort.size()),
            externalTorque(msgAddOutputs->external_torques.data.data(),
                           msgAddOutputs->external_torques.data.size());
        iiwa_tools::RobotState robot_state_tool({position, velocity, torque});

        // Forward kinematics to compute EEF pose
        iiwa_tools::EefState eefPose = _iiwaTools.perform_fk(robot_state_tool);

        // Jacobians for velocity/effort
        Eigen::MatrixXd jac(NB_DIM, position.size());
        Eigen::MatrixXd jacDeriv(NB_DIM, position.size());
        Eigen::MatrixXd jacTPInv(position.size(), NB_DIM);
        std::tie(jac, jacDeriv) = _iiwaTools.jacobians(robot_state_tool);
        jacTPInv = pseudo_inverse(Eigen::MatrixXd(jac.transpose()));

        // Compute the end effector state
        Eigen::Vector3d eefPosition = eefPose.translation;
        Eigen::Quaterniond eefOrientation = eefPose.orientation;
        Eigen::Vector6d eefTwist = jac * velocity,
                        eefWrench = jacTPInv * torque,
                        eefExtWrench = jacTPInv * externalTorque;

        // Publish the end effector state
        if (_pubEefState.trylock())
        {
            _pubEefState.msg_.header.stamp = ros::Time::now();
            tf::pointEigenToMsg(eefPosition, _pubEefState.msg_.position);
            tf::quaternionEigenToMsg(eefOrientation,
                                     _pubEefState.msg_.orientation);
            iiwa_tools::twistEigenToMsg(eefTwist, _pubEefState.msg_.twist);
            iiwa_tools::wrenchEigenToMsg(eefWrench, _pubEefState.msg_.wrench);
            iiwa_tools::wrenchEigenToMsg(eefExtWrench,
                                         _pubEefState.msg_.external_wrench);
            _pubEefState.unlockAndPublish();
        }
    }

 protected:
    // Topics
    message_filters::Subscriber<sensor_msgs::JointState>
        _subJointState;  ///< Joint state subscriber.
    message_filters::Subscriber<iiwa_driver::AdditionalOutputs>
        _subAdditionalOutputs;  ///< External torque subscriber.
    message_filters::Synchronizer<AprxmtSyncJointState>
        _syncJointState;  ///< Publisher synchronizer.
    // Debug/Test
    realtime_tools::RealtimePublisher<iiwa_tools::MsgEefState>
        _pubEefState;  ///< End effector state.

    // Iiwa tools
    iiwa_tools::IiwaTools _iiwaTools;  ///< IIWA tools to compute FK, jacobians.
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_eef");
    ros::NodeHandle nh;

    // Subscribe to joint states and other necessary messages
    // Compute and publish EEF state on callback
    EefPublisher eefPublisher;
    if (!eefPublisher.init(nh))
    {
        ROS_ERROR_NAMED("EefPublisher", "Failed to initialize node.");
        return -1;
    }

    ros::spin();

    return 0;
}
