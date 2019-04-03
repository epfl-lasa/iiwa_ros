#ifndef IIWA_SERVICE_IIWA_SERVICE_H
#define IIWA_SERVICE_IIWA_SERVICE_H

// std headers
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// RBDyn headers
#include <RBDyn/IK.h>
#include <mc_rbdyn_urdf/urdf.h>

// IIWA Tools
#include <iiwa_service/iiwa_tools.h>

// Iiwa IK server headers
#include <iiwa_service/GetFK.h>
#include <iiwa_service/GetGravity.h>
#include <iiwa_service/GetIK.h>
#include <iiwa_service/GetJacobian.h>

namespace iiwa_service {
    class IiwaService {
    public:
        IiwaService(ros::NodeHandle nh);

        bool perform_fk(iiwa_service::GetFK::Request& request,
            iiwa_service::GetFK::Response& response);

        bool perform_ik(iiwa_service::GetIK::Request& request,
            iiwa_service::GetIK::Response& response);

        bool get_jacobian(iiwa_service::GetJacobian::Request& request,
            iiwa_service::GetJacobian::Response& response);

        bool get_gravity(iiwa_service::GetGravity::Request& request,
            iiwa_service::GetGravity::Response& response);

    protected:
        void _load_params();
        void _init_rbdyn();
        size_t _rbd_index(const std::string& body_name) const;

        // ROS related
        ros::NodeHandle _nh;
        std::string _robot_description, _fk_service_name, _ik_service_name, _jacobian_service_name, _gravity_service_name;
        ros::ServiceServer _fk_server, _ik_server, _jacobian_server, _gravity_server;

        // IIWA Tools
        iiwa_service::IiwaTools tools;
        std::vector<size_t> _rbd_indices;
        RobotState robot_state_;
    }; // class IiwaService
} // namespace iiwa_service

#endif // IIWA_SERVICE_IIWA_SERVICE_H