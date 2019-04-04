#ifndef IIWA_TOOLS_IIWA_SERVICE_H
#define IIWA_TOOLS_IIWA_SERVICE_H

// std headers
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// IIWA Tools
#include <iiwa_tools/iiwa_tools.h>

// Iiwa IK server headers
#include <iiwa_tools/GetFK.h>
#include <iiwa_tools/GetGravity.h>
#include <iiwa_tools/GetIK.h>
#include <iiwa_tools/GetJacobian.h>

namespace iiwa_tools {
    class IiwaService {
    public:
        IiwaService(ros::NodeHandle nh);

        void init();

        bool perform_fk(iiwa_tools::GetFK::Request& request,
            iiwa_tools::GetFK::Response& response);

        bool perform_ik(iiwa_tools::GetIK::Request& request,
            iiwa_tools::GetIK::Response& response);

        bool get_jacobian(iiwa_tools::GetJacobian::Request& request,
            iiwa_tools::GetJacobian::Response& response);

        bool get_gravity(iiwa_tools::GetGravity::Request& request,
            iiwa_tools::GetGravity::Response& response);

    protected:
        void _load_params();

        // ROS related
        ros::NodeHandle _nh;
        std::string _robot_description, _fk_service_name, _ik_service_name, _jacobian_service_name, _gravity_service_name;
        ros::ServiceServer _fk_server, _ik_server, _jacobian_server, _gravity_server;

        // Robot
        unsigned int _n_joints;
        std::string _end_effector;

        // IIWA Tools
        IiwaTools _tools;
    }; // class IiwaService
} // namespace iiwa_tools

#endif // IIWA_SERVICE_IIWA_SERVICE_H