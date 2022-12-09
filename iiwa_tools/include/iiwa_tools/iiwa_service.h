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
#include <iiwa_tools/GetJacobians.h>
#include <iiwa_tools/GetMassMatrix.h>

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

        bool get_jacobian_deriv(iiwa_tools::GetJacobian::Request& request,
            iiwa_tools::GetJacobian::Response& response);

        bool get_jacobians(iiwa_tools::GetJacobians::Request& request,
            iiwa_tools::GetJacobians::Response& response);

        bool get_gravity(iiwa_tools::GetGravity::Request& request,
            iiwa_tools::GetGravity::Response& response);

	bool get_mass_matrix(iiwa_tools::GetMassMatrix::Request& request,
	    iiwa_tools::GetMassMatrix::Response& response);

    protected:
        void _load_params();

        // ROS related
        ros::NodeHandle _nh;
        std::string _robot_description, _fk_service_name, _ik_service_name, _jacobian_service_name, _jacobian_deriv_service_name, _jacobians_service_name, _gravity_service_name, _mass_service_name;
        ros::ServiceServer _fk_server, _ik_server, _jacobian_server, _jacobian_deriv_server, _jacobians_server, _gravity_server, _mass_server;

        // Robot
        unsigned int _n_joints;
        std::string _end_effector;

        // IIWA Tools
        IiwaTools _tools;
    }; // class IiwaService
} // namespace iiwa_tools

#endif // IIWA_SERVICE_IIWA_SERVICE_H
