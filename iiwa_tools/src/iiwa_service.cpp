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
#include <iiwa_tools/iiwa_service.h>

namespace iiwa_tools {
    double get_multi_array(const std_msgs::Float64MultiArray& array, size_t i, size_t j)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        return array.data[offset + i * array.layout.dim[0].stride + j];
    }

    void set_multi_array(std_msgs::Float64MultiArray& array, size_t i, size_t j, double val)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        array.data[offset + i * array.layout.dim[0].stride + j] = val;
    }

    IiwaService::IiwaService(ros::NodeHandle nh) : _nh(nh)
    {
        ROS_INFO_STREAM("Starting Iiwa IK server..");
        _load_params();
        init();

        _fk_server = _nh.advertiseService(_fk_service_name, &IiwaService::perform_fk, this);
        ROS_INFO_STREAM("Started Iiwa FK server..");

        _ik_server = _nh.advertiseService(_ik_service_name, &IiwaService::perform_ik, this);
        ROS_INFO_STREAM("Started Iiwa IK server..");

        _jacobian_server = _nh.advertiseService(_jacobian_service_name, &IiwaService::get_jacobian, this);
        ROS_INFO_STREAM("Started Iiwa Jacobian server..");

        _jacobian_deriv_server = _nh.advertiseService(_jacobian_deriv_service_name, &IiwaService::get_jacobian_deriv, this);
        ROS_INFO_STREAM("Started Iiwa Jacobian Derivative server..");

        _jacobians_server = _nh.advertiseService(_jacobians_service_name, &IiwaService::get_jacobians, this);
        ROS_INFO_STREAM("Started Iiwa Jacobians server..");

        _gravity_server = _nh.advertiseService(_gravity_service_name, &IiwaService::get_gravity, this);
        ROS_INFO_STREAM("Started Iiwa Gravity Compensation server..");

        _mass_server = _nh.advertiseService(_mass_service_name, &IiwaService::get_mass_matrix, this);
        ROS_INFO_STREAM("Started Iiwa Mass Matrix server..");
    }

    bool IiwaService::perform_fk(iiwa_tools::GetFK::Request& request,
        iiwa_tools::GetFK::Response& response)
    {
        if (request.joints.layout.dim.size() != 2 || request.joints.layout.dim[1].size != _n_joints) {
            ROS_ERROR("Request joint angles not properly defined.");
            return false;
        }

        response.poses.resize(request.joints.layout.dim[0].size);

        for (size_t point = 0; point < request.joints.layout.dim[0].size; ++point) {
            iiwa_tools::RobotState robot_state;
            robot_state.position.resize(_n_joints);
            for (size_t i = 0; i < _n_joints; i++)
                robot_state.position[i] = get_multi_array(request.joints, point, i);

            iiwa_tools::EefState ee_state = _tools.perform_fk(robot_state);

            response.poses[point].position.x = ee_state.translation(0);
            response.poses[point].position.y = ee_state.translation(1);
            response.poses[point].position.z = ee_state.translation(2);

            response.poses[point].orientation.w = ee_state.orientation.w();
            response.poses[point].orientation.x = ee_state.orientation.x();
            response.poses[point].orientation.y = ee_state.orientation.y();
            response.poses[point].orientation.z = ee_state.orientation.z();
        }

        return true;
    }

    bool IiwaService::perform_ik(iiwa_tools::GetIK::Request& request,
        iiwa_tools::GetIK::Response& response)
    {
        bool seeds_provided = request.seed_angles.layout.dim.size() == 2 && (request.seed_angles.layout.dim[0].size == request.poses.size());
        // // copy RBDyn for thread-safety
        // // TO-DO: Check if it takes time
        // mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        // double damp = 1e-3;
        // Eigen::VectorXd damping = Eigen::VectorXd::Ones(_n_joints).array() * damp;
        // if (request.damping.size() != _n_joints) {
        //     ROS_WARN_STREAM("No damping parameters given. Using " << damping.transpose() << ".");
        // }
        // else {
        //     for (size_t i = 0; i < _n_joints; i++) {
        //         damping(i) = request.damping[i];
        //     }
        // }

        // double slack = 10000.;
        // Eigen::VectorXd slack_vec = Eigen::VectorXd::Ones(6).array() * slack;
        // if (request.slack.size() != 6) {
        //     ROS_WARN_STREAM("No slack parameters given. Using " << slack_vec.transpose() << ".");
        // }
        // else {
        //     for (size_t i = 0; i < 6; i++) {
        //         slack_vec(i) = request.slack[i];
        //     }
        // }

        // int max_iterations = request.max_iterations;
        // if (max_iterations <= 0) {
        //     max_iterations = 50;
        //     ROS_WARN_STREAM("No max iterations given. Using " << max_iterations << " iterations.");
        // }
        // double tolerance = request.tolerance;
        // if (tolerance <= 0.) {
        //     tolerance = 1e-5;
        //     ROS_WARN_STREAM("No tolerance given. Using " << tolerance << ".");
        // }

        response.joints.layout.dim.resize(2);
        response.joints.layout.data_offset = 0;
        response.joints.layout.dim[0].size = request.poses.size();
        response.joints.layout.dim[0].stride = _n_joints;
        response.joints.layout.dim[1].size = _n_joints;
        response.joints.layout.dim[1].stride = 0;
        response.joints.data.resize(request.poses.size() * _n_joints);

        for (size_t point = 0; point < request.poses.size(); ++point) {
            iiwa_tools::RobotState seed_state;
            if (seeds_provided) {
                seed_state.position.resize(_n_joints);
                for (size_t i = 0; i < _n_joints; i++) {
                    seed_state.position(i) = get_multi_array(request.seed_angles, point, i);
                }
            }

            iiwa_tools::EefState ee_state;
            ee_state.translation = {request.poses[point].position.x, request.poses[point].position.y, request.poses[point].position.z};
            ee_state.orientation = Eigen::Quaterniond(request.poses[point].orientation.w,
                request.poses[point].orientation.x,
                request.poses[point].orientation.y,
                request.poses[point].orientation.z);

            Eigen::VectorXd q_best = _tools.perform_ik(ee_state, seed_state);

            for (size_t joint = 0; joint < _n_joints; ++joint) {
                set_multi_array(response.joints, point, joint, q_best[joint]);
            }

            // TO-DO: Fix those
            response.is_valid.push_back(true);
            response.accepted_tolerance.push_back(1e-5);
            // response.is_valid.push_back(iter < max_iterations);
            // response.accepted_tolerance.push_back(best);
        }

        return true;
    }

    bool IiwaService::get_jacobian(iiwa_tools::GetJacobian::Request& request,
        iiwa_tools::GetJacobian::Response& response)
    {
        if (request.joint_angles.size() != _n_joints || request.joint_angles.size() != request.joint_velocities.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        RobotState robot_state;
        robot_state.position.resize(_n_joints);
        robot_state.velocity.resize(_n_joints);
        for (size_t i = 0; i < _n_joints; i++) {
            robot_state.position[i] = request.joint_angles[i];
            robot_state.velocity[i] = request.joint_velocities[i];
        }

        Eigen::MatrixXd jac_mat = _tools.jacobian(robot_state);

        // Fill response
        response.jacobian.layout.dim.resize(2);
        response.jacobian.layout.data_offset = 0;
        response.jacobian.layout.dim[0].size = jac_mat.rows();
        response.jacobian.layout.dim[0].stride = jac_mat.cols();
        response.jacobian.layout.dim[1].size = jac_mat.cols();
        response.jacobian.layout.dim[1].stride = 0;
        response.jacobian.data.resize(jac_mat.rows() * jac_mat.cols());

        for (int i = 0; i < jac_mat.rows(); i++) {
            for (int j = 0; j < jac_mat.cols(); j++) {
                set_multi_array(response.jacobian, i, j, jac_mat(i, j));
            }
        }

        return true;
    }

    bool IiwaService::get_jacobian_deriv(iiwa_tools::GetJacobian::Request& request,
        iiwa_tools::GetJacobian::Response& response)
    {
        if (request.joint_angles.size() != _n_joints || request.joint_angles.size() != request.joint_velocities.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        RobotState robot_state;
        robot_state.position.resize(_n_joints);
        robot_state.velocity.resize(_n_joints);
        for (size_t i = 0; i < _n_joints; i++) {
            robot_state.position[i] = request.joint_angles[i];
            robot_state.velocity[i] = request.joint_velocities[i];
        }

        Eigen::MatrixXd jac_deriv_mat = _tools.jacobian_deriv(robot_state);

        // Fill response
        response.jacobian.layout.dim.resize(2);
        response.jacobian.layout.data_offset = 0;
        response.jacobian.layout.dim[0].size = jac_deriv_mat.rows();
        response.jacobian.layout.dim[0].stride = jac_deriv_mat.cols();
        response.jacobian.layout.dim[1].size = jac_deriv_mat.cols();
        response.jacobian.layout.dim[1].stride = 0;
        response.jacobian.data.resize(jac_deriv_mat.rows() * jac_deriv_mat.cols());

        for (int i = 0; i < jac_deriv_mat.rows(); i++) {
            for (int j = 0; j < jac_deriv_mat.cols(); j++) {
                set_multi_array(response.jacobian, i, j, jac_deriv_mat(i, j));
            }
        }

        return true;
    }

    bool IiwaService::get_jacobians(iiwa_tools::GetJacobians::Request& request,
        iiwa_tools::GetJacobians::Response& response)
    {
        if (request.joint_angles.size() != _n_joints || request.joint_angles.size() != request.joint_velocities.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        RobotState robot_state;
        robot_state.position.resize(_n_joints);
        robot_state.velocity.resize(_n_joints);
        for (size_t i = 0; i < _n_joints; i++) {
            robot_state.position[i] = request.joint_angles[i];
            robot_state.velocity[i] = request.joint_velocities[i];
        }

        Eigen::MatrixXd jac_mat, jac_deriv_mat;
        std::tie(jac_mat, jac_deriv_mat) = _tools.jacobians(robot_state);

        // Fill response (jacobian)
        response.jacobian.layout.dim.resize(2);
        response.jacobian.layout.data_offset = 0;
        response.jacobian.layout.dim[0].size = jac_mat.rows();
        response.jacobian.layout.dim[0].stride = jac_mat.cols();
        response.jacobian.layout.dim[1].size = jac_mat.cols();
        response.jacobian.layout.dim[1].stride = 0;
        response.jacobian.data.resize(jac_mat.rows() * jac_mat.cols());

        // Fill response (jacobian derivative)
        response.jacobian_deriv.layout.dim.resize(2);
        response.jacobian_deriv.layout.data_offset = 0;
        response.jacobian_deriv.layout.dim[0].size = jac_deriv_mat.rows();
        response.jacobian_deriv.layout.dim[0].stride = jac_deriv_mat.cols();
        response.jacobian_deriv.layout.dim[1].size = jac_deriv_mat.cols();
        response.jacobian_deriv.layout.dim[1].stride = 0;
        response.jacobian_deriv.data.resize(jac_deriv_mat.rows() * jac_deriv_mat.cols());

        for (int i = 0; i < jac_deriv_mat.rows(); i++) {
            for (int j = 0; j < jac_deriv_mat.cols(); j++) {
                set_multi_array(response.jacobian, i, j, jac_mat(i, j));
                set_multi_array(response.jacobian_deriv, i, j, jac_deriv_mat(i, j));
            }
        }

        return true;
    }

    bool IiwaService::get_gravity(iiwa_tools::GetGravity::Request& request,
        iiwa_tools::GetGravity::Response& response)
    {
        if (request.joint_angles.size() != _n_joints || request.joint_angles.size() != request.joint_velocities.size() || request.joint_angles.size() != request.joint_torques.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        std::vector<double> gravity = {0., 0., -9.8};
        if (request.gravity.size() != 3) {
            ROS_WARN_STREAM("Gravity not given. Assuming default [0, 0, -9.8]!");
        }
        else {
            for (size_t i = 0; i < 3; i++) {
                gravity[i] = request.gravity[i];
            }
        }

        RobotState robot_state;
        robot_state.position.resize(_n_joints);
        robot_state.velocity.resize(_n_joints);
        robot_state.torque.resize(_n_joints);
        for (size_t i = 0; i < _n_joints; i++) {
            robot_state.position[i] = request.joint_angles[i];
            robot_state.velocity[i] = request.joint_velocities[i];
            robot_state.torque[i] = request.joint_torques[i];
        }

        Eigen::VectorXd C = _tools.gravity(gravity, robot_state);

        // Fill response
        response.compensation_torques.resize(_n_joints);

        for (size_t i = 0; i < _n_joints; i++) {
            response.compensation_torques[i] = C(i);
        }

        return true;
    }

    bool IiwaService::get_mass_matrix(iiwa_tools::GetMassMatrix::Request& request,
        iiwa_tools::GetMassMatrix::Response& response)
    {
        if (request.joint_angles.size() != _n_joints) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        RobotState robot_state;
        robot_state.position.resize(_n_joints);
        for (size_t i = 0; i < _n_joints; i++) {
            robot_state.position[i] = request.joint_angles[i];
        }

        Eigen::MatrixXd H = _tools.mass_matrix(robot_state);

        // Fill response
        response.mass_matrix.layout.dim.resize(2);
        response.mass_matrix.layout.data_offset = 0;
        response.mass_matrix.layout.dim[0].size = H.rows();
        response.mass_matrix.layout.dim[0].stride = H.cols();
        response.mass_matrix.layout.dim[1].size = H.cols();
        response.mass_matrix.layout.dim[1].stride = 0;
        response.mass_matrix.data.resize(H.rows() * H.cols());

        for (int i = 0; i < H.rows(); i++) {
            for (int j = 0; j < H.cols(); j++) {
                set_multi_array(response.mass_matrix, i, j, H(i, j));
            }
        }
	
        return true;
    }

  void IiwaService::_load_params()
    {
        ros::NodeHandle n_p("~");

        n_p.param<std::string>("service/robot_description", _robot_description, "/robot_description");
        n_p.param<std::string>("service/end_effector", _end_effector, "iiwa_link_ee");
        n_p.param<std::string>("service/fk_service_name", _fk_service_name, "iiwa_fk_server");
        n_p.param<std::string>("service/ik_service_name", _ik_service_name, "iiwa_ik_server");
        n_p.param<std::string>("service/jacobian_service_name", _jacobian_service_name, "iiwa_jacobian_server");
        n_p.param<std::string>("service/jacobian_deriv_service_name", _jacobian_deriv_service_name, "iiwa_jacobian_deriv_server");
        n_p.param<std::string>("service/jacobians_service_name", _jacobians_service_name, "iiwa_jacobians_server");
        n_p.param<std::string>("service/gravity_service_name", _gravity_service_name, "iiwa_gravity_server");
	n_p.param<std::string>("service/mass_service_name", _mass_service_name, "iiwa_mass_server");
    }

    void IiwaService::init()
    {
        // Get the URDF XML from the parameter server
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("IiwaService", "IiwaService is waiting for model"
                                               " URDF in parameter [%s] on the ROS param server.",
                _robot_description.c_str());

            _nh.getParam(_robot_description, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("IiwaService", "Received urdf from param server, parsing...");

        // Initialize iiwa tools
        _tools.init_rbdyn(urdf_string, _end_effector);

        // Number of joints
        _n_joints = _tools.get_indices().size();

        ROS_INFO_STREAM_NAMED("IiwaService", "Number of joints found: " << _n_joints);
    }
} // namespace iiwa_tools
