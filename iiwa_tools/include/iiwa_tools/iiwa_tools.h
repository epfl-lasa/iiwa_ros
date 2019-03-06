#ifndef IIWA_TOOLS_IIWA_TOOLS_H
#define IIWA_TOOLS_IIWA_TOOLS_H

// std headers
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// RBDyn headers
#include <RBDyn/IK.h>
#include <mc_rbdyn_urdf/urdf.h>

// Iiwa IK server headers
#include <iiwa_tools/GetFK.h>
#include <iiwa_tools/GetGravity.h>
#include <iiwa_tools/GetIK.h>
#include <iiwa_tools/GetJacobian.h>

namespace iiwa_tools {
    class IiwaTools {
    public:
        IiwaTools(ros::NodeHandle nh);

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
        void _init_rbdyn();
        size_t _rbd_index(const std::string& body_name) const;

        // ROS related
        ros::NodeHandle _nh;
        std::string _robot_description, _fk_service_name, _ik_service_name, _jacobian_service_name, _gravity_service_name;
        ros::ServiceServer _fk_server, _ik_server, _jacobian_server, _gravity_server;

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        std::unique_ptr<rbd::InverseKinematics> _ik;
        std::vector<double> _gravity;

        std::vector<size_t> _rbd_indices;
        std::string _end_effector;
        size_t _ef_index;
    };
} // namespace iiwa_tools

#endif