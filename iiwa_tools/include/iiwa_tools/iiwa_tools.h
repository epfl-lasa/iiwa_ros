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
#ifndef IIWA_TOOLS_IIWA_TOOLS_H
#define IIWA_TOOLS_IIWA_TOOLS_H

// std headers
#include <utility>
#include <vector>

// RBDyn headers
#include <RBDyn/IK.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace iiwa_tools {
    struct RobotState {
        Eigen::VectorXd position,
            velocity,
            torque;
    };

    struct EefState {
        Eigen::Vector3d translation;
        Eigen::Quaterniond orientation;
    };

    class IiwaTools {
    public:
        IiwaTools() {}
        ~IiwaTools() {}

        void init_rbdyn(const std::string& urdf_string, const std::string& end_effector);

        std::vector<size_t> get_indices() { return _rbd_indices; }

        EefState perform_fk(const RobotState& robot_state);
        Eigen::VectorXd perform_ik(const EefState& ee_state, const RobotState& seed_state = RobotState());
        Eigen::MatrixXd jacobian(const RobotState& robot_state);
        Eigen::MatrixXd jacobian_deriv(const RobotState& robot_state);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> jacobians(const RobotState& robot_state);
        Eigen::VectorXd gravity(const std::vector<double>& gravity, const RobotState& robot_state);
	Eigen::MatrixXd mass_matrix(const RobotState& robot_state);

    protected:
        size_t _rbd_index(const std::string& body_name) const;
        void _update_urdf_state(mc_rbdyn_urdf::URDFParserResult& rbdyn_urdf, const RobotState& robot_state);

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        std::unique_ptr<rbd::InverseKinematics> _ik;

        // Helper variables
        std::vector<size_t> _rbd_indices;
        size_t _ef_index;

    }; // class IiwaTools
} // namespace iiwa_tools

#endif
