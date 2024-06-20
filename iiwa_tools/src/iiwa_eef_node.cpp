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
#define DOF_ANGLE_SELF 6

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
    EefPublisher() {}
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
        nh.param<std::string>("params/end_effector", end_effector, "link_ee");

        // Initialize iiwa tools
        _iiwaTools.init_rbdyn(urdf_string, end_effector);

        // Init subscribers
        _firstExternalTorque = false;
        _subJointState = nh.subscribe<sensor_msgs::JointState>(
            "joint_states",
            QUEUE_SIZE,
            boost::bind(&EefPublisher::updateJointState, this, _1),
            ros::VoidPtr(),
            ros::TransportHints().reliable().tcpNoDelay());
        _subAdditionalOutputs = nh.subscribe<iiwa_driver::AdditionalOutputs>(
            "additional_outputs",
            QUEUE_SIZE,
            boost::bind(&EefPublisher::updateAdditionalOutputs, this, _1),
            ros::VoidPtr(),
            ros::TransportHints().reliable().tcpNoDelay());

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
    void updateJointState(const sensor_msgs::JointStateConstPtr& msgJointState)
    {
        // Extract joint state from msgs
        Eigen::Map<const Eigen::VectorXd> position(
            msgJointState->position.data(), msgJointState->position.size()),
            velocity(msgJointState->velocity.data(),
                     msgJointState->velocity.size()),
            torque(msgJointState->effort.data(), msgJointState->effort.size());
        iiwa_tools::RobotState robot_state_tool({position, velocity, torque});

        // Forward kinematics to compute EEF pose
        iiwa_tools::EefState eefPose = _iiwaTools.perform_fk(robot_state_tool);

        // Jacobians for velocity/effort
        Eigen::MatrixXd jac(NB_DIM, position.size());
        Eigen::MatrixXd jacDeriv(NB_DIM, position.size());
        Eigen::MatrixXd jacTPInv(position.size(), NB_DIM);
        std::tie(jac, jacDeriv) = _iiwaTools.jacobians(robot_state_tool);
        jacTPInv = pseudo_inverse(jac.transpose());

        // Compute the end effector state
        Eigen::Vector3d eefPosition = eefPose.translation;
        Eigen::Quaterniond eefOrientation = eefPose.orientation;
        Eigen::Vector6d eefTwist = jac * velocity,
                        eefWrench = jacTPInv * torque, eefExtWrench;
        if (_firstExternalTorque)
        {
            std::lock_guard<std::mutex> lockGuard(_mutExternalTorque);
            eefExtWrench = jacTPInv * _externalTorque;
        }

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
            _pubEefState.msg_.angle_self =
                msgJointState->position[DOF_ANGLE_SELF];
            _pubEefState.unlockAndPublish();
        }
    }
    void updateAdditionalOutputs(
        const iiwa_driver::AdditionalOutputsConstPtr& msgAddOutputs)
    {
        std::lock_guard<std::mutex> lockGuard(_mutExternalTorque);
        _externalTorque = Eigen::Map<const Eigen::VectorXd>(
            msgAddOutputs->external_torques.data.data(),
            msgAddOutputs->external_torques.data.size());
        _firstExternalTorque = true;
    }

 protected:
    // Topics
    ros::Subscriber _subJointState,  ///< Joint state subscriber.
        _subAdditionalOutputs;       ///< External torque subscriber.

    // State
    Eigen::VectorXd _externalTorque;  ///< Robot external torque for each joint.
    bool
        _firstExternalTorque;  ///< Flag to signal that first data was received from topic.
    std::mutex _mutExternalTorque;  ///< Mutex for the external torque.

    // Publisher
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
