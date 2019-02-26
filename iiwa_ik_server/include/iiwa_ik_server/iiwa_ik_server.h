#ifndef IIWA_IK_SERVER_IIWA_IK_SERVER_H
#define IIWA_IK_SERVER_IIWA_IK_SERVER_H

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// RBDyn headers
#include <RBDyn/IK.h>
#include <mc_rbdyn_urdf/urdf.h>

// TRAC IK headers
#include <trac_ik/trac_ik.hpp>

// Iiwa IK server headers
#include <iiwa_ik_server/GetIK.h>

namespace iiwa_ik_server {
    class IiwaIKServer {
    public:
        IiwaIKServer(ros::NodeHandle nh);

        bool perform_ik(iiwa_ik_server::GetIK::Request& request,
            iiwa_ik_server::GetIK::Response& response);

    protected:
        void _load_params();
        void _init_rbdyn();
        void _init_trac_ik();
        size_t _rbd_index(const std::string& body_name) const;

        // ROS related
        ros::NodeHandle _nh;
        std::string _robot_description, _service_name;
        ros::ServiceServer _ik_server;

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        std::unique_ptr<rbd::InverseKinematics> _ik;

        std::vector<size_t> _rbd_indices;
        std::string _end_effector, _base_link;
        size_t _ef_index;

        // TRAC IK
        std::unique_ptr<TRAC_IK::TRAC_IK> _trac_ik_solver;
        KDL::Chain _chain;
    };
} // namespace iiwa_ik_server

#endif