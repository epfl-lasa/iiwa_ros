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
#include <iiwa_tools/iiwa_tools.h>

// RBDyn headers
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <SpaceVecAlg/Conversions.h>

// cvxgen headers
#include <iiwa_tools/cvxgen/iiwa_ik_cvxgen.hpp>

// ROS headers
#include <ros/ros.h>

namespace iiwa_tools {
    double wrap_angle(const double& angle)
    {
        double wrapped;
        if ((angle <= M_PI) && (angle >= -M_PI)) {
            wrapped = angle;
        }
        else if (angle < 0.0) {
            wrapped = std::fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
        }
        else {
            wrapped = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
        }
        return wrapped;
    }

    iiwa_tools::EefState IiwaTools::perform_fk(const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
        Eigen::VectorXd q_high = q_low;

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t index = _rbd_indices[i];
            q_low(i) = rbdyn_urdf.limits.lower[rbdyn_urdf.mb.joint(index).name()][0];
            q_high(i) = rbdyn_urdf.limits.upper[rbdyn_urdf.mb.joint(index).name()][0];
        }

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i];
            double jt = robot_state.position[i];
            // wrap in [-pi,pi]
            jt = wrap_angle(jt);
            // enforce limits
            if (jt < q_low(i))
                jt = q_low(i);
            if (jt > q_high(i))
                jt = q_high(i);

            rbdyn_urdf.mbc.q[rbd_index][0] = jt;
        }

        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        sva::PTransformd tf = rbdyn_urdf.mbc.bodyPosW[_ef_index];

        Eigen::Matrix4d eig_tf = sva::conversions::toHomogeneous(tf);
        Eigen::Vector3d trans = eig_tf.col(3).head(3);
        Eigen::Matrix3d rot_mat = eig_tf.block(0, 0, 3, 3);
        Eigen::Quaterniond quat = Eigen::Quaterniond(rot_mat).normalized();

        return {trans, quat};
    }

    Eigen::VectorXd IiwaTools::perform_ik(const EefState& ee_state, const RobotState& seed_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        // TO-DO: Get this from parameters?
        double damp = 1e-3;
        Eigen::VectorXd damping = Eigen::VectorXd::Ones(_rbd_indices.size()).array() * damp;

        double slack = 10000.;
        Eigen::VectorXd slack_vec = Eigen::VectorXd::Ones(6).array() * slack;

        int max_iterations = 50;
        double tolerance = 1e-5;
        // End TO-DO

        Eigen::VectorXd zero = Eigen::VectorXd::Zero(_rbd_indices.size());
        Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
        Eigen::VectorXd q_high = q_low;

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t index = _rbd_indices[i];
            q_low(i) = rbdyn_urdf.limits.lower[rbdyn_urdf.mb.joint(index).name()][0];
            q_high(i) = rbdyn_urdf.limits.upper[rbdyn_urdf.mb.joint(index).name()][0];
        }

        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf.col(3).head(3) << ee_state.translation;

        tf.block(0, 0, 3, 3) = ee_state.orientation
                                   .normalized()
                                   .matrix();
        sva::PTransformd target_tf = sva::conversions::fromHomogeneous(tf);

        Eigen::VectorXd qref = Eigen::VectorXd::Zero(_rbd_indices.size());

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        bool seeds_provided = (seed_state.position.size() == _rbd_indices.size());
        if (seeds_provided) {
            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                size_t rbd_index = _rbd_indices[i];
                // wrap in [-pi,pi]
                double seed = wrap_angle(seed_state.position[i]);
                // enforce limits
                if (seed < q_low(i))
                    seed = q_low(i);
                if (seed > q_high(i))
                    seed = q_high(i);

                rbdyn_urdf.mbc.q[rbd_index][0] = seed;
                qref(i) = seed;
            }
        }

        // Solve IK with traditional approach and pass it as a seed if successful
        bool valid = _ik->inverseKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc, target_tf);
        if (valid) {
            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                size_t rbd_index = _rbd_indices[i];

                // wrap in [-pi,pi]
                qref(i) = wrap_angle(rbdyn_urdf.mbc.q[rbd_index][0]);

                // enforce limits
                if (qref(i) < q_low(i))
                    qref(i) = q_low(i);
                if (qref(i) > q_high(i))
                    qref(i) = q_high(i);
            }
            ROS_DEBUG_STREAM("Using seed from RBDyn: " << qref.transpose());
        }
        else {
            if (seeds_provided) {
                for (size_t i = 0; i < _rbd_indices.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];
                    // wrap in [-pi,pi]
                    double seed = wrap_angle(seed_state.position[i]);
                    // enforce limits
                    if (seed < q_low(i))
                        seed = q_low(i);
                    if (seed > q_high(i))
                        seed = q_high(i);

                    rbdyn_urdf.mbc.q[rbd_index][0] = seed;
                    qref(i) = seed;
                }
            }
            else
                rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        }

        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        double best = std::numeric_limits<double>::max();
        Eigen::VectorXd q_best = qref;

        int iter = 0;
        double error = 0.;
        for (iter = 0; iter < max_iterations; iter++) {
            rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
            rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

            Eigen::Vector3d rotErr = sva::rotationError(rbdyn_urdf.mbc.bodyPosW[_ef_index].rotation(), target_tf.rotation());
            Eigen::Vector6d v;
            v << rotErr, target_tf.translation() - rbdyn_urdf.mbc.bodyPosW[_ef_index].translation();

            error = v.norm();

            if (error < best) {
                best = error;
                q_best = qref;
            }

            if (error < tolerance)
                break;

            Eigen::MatrixXd jac_mat = jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);

            iiwa_ik_cvxgen::Solver ik_solver;

            ik_solver.set_defaults();
            ik_solver.setup_indexing();
            ik_solver.settings.verbose = 0;
            ik_solver.settings.resid_tol = 1e-10;
            ik_solver.settings.eps = 1e-10;
            ik_solver.settings.max_iters = 100;

            // set params
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < _rbd_indices.size(); c++) {
                    ik_solver.params.J[r + 1][c] = jac_mat(r, c);
                }
            }

            // adapt the limits
            Eigen::VectorXd qlow = q_low - qref;
            Eigen::VectorXd qhigh = q_high - qref;

            memcpy(ik_solver.params.damping, damping.data(), _rbd_indices.size() * sizeof(double));
            memcpy(ik_solver.params.slack, slack_vec.data(), 6 * sizeof(double));
            memcpy(ik_solver.params.qref, zero.data(), _rbd_indices.size() * sizeof(double)); // we set qref to zero so that we minimize the dq
            memcpy(ik_solver.params.qlow, qlow.data(), _rbd_indices.size() * sizeof(double));
            memcpy(ik_solver.params.qup, qhigh.data(), _rbd_indices.size() * sizeof(double));
            memcpy(ik_solver.params.dx, v.data(), 6 * sizeof(double));

            ik_solver.solve();

            Eigen::VectorXd q_prev = qref;

            for (size_t j = 0; j < _rbd_indices.size(); j++) {
                qref(j) += ik_solver.vars.dq[j];
            }

            for (size_t j = 0; j < _rbd_indices.size(); j++) {
                size_t rbd_index = _rbd_indices[j];
                rbdyn_urdf.mbc.q[rbd_index][0] = qref(j);
            }

            if ((q_prev - qref).norm() < 1e-8)
                break;
        }

        return q_best;
    }

    Eigen::MatrixXd IiwaTools::mass_matrix(const RobotState& robot_state)
    {
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;
	rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
	_update_urdf_state(rbdyn_urdf, robot_state);
	
	// Forward Dynamics
	rbd::ForwardDynamics fd(rbdyn_urdf.mb);
	
	// Compute Mass Matrix
	rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc); // TODO: needed?
	rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc); // TODO: needed?
	fd.computeH(rbdyn_urdf.mb, rbdyn_urdf.mbc);
      
	// Get Mass Matrix
	return fd.H();
    }

    Eigen::VectorXd IiwaTools::gravity(const std::vector<double>& gravity, const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        rbdyn_urdf.mbc.gravity = {gravity[0], gravity[1], gravity[2]};

        _update_urdf_state(rbdyn_urdf, robot_state);

        // Forward Dynamics
        rbd::ForwardDynamics fd(rbdyn_urdf.mb);

        // Compute gravity compensation
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        fd.computeC(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        // Get gravity and Coriolis forces
        return -fd.C();
    }

    Eigen::MatrixXd IiwaTools::jacobian(const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

        _update_urdf_state(rbdyn_urdf, robot_state);

        // Compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        // TO-DO: Check if we need this
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        return jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);
    }

    Eigen::MatrixXd IiwaTools::jacobian_deriv(const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

        _update_urdf_state(rbdyn_urdf, robot_state);

        // Compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        // TO-DO: Check if we need this
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        return jac.jacobianDot(rbdyn_urdf.mb, rbdyn_urdf.mbc);
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> IiwaTools::jacobians(const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

        _update_urdf_state(rbdyn_urdf, robot_state);

        // Compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        // TO-DO: Check if we need this
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        return std::make_pair(jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc), jac.jacobianDot(rbdyn_urdf.mb, rbdyn_urdf.mbc));
    }

    void IiwaTools::init_rbdyn(const std::string& urdf_string, const std::string& end_effector)
    {
        // Convert URDF to RBDyn
        _rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);

        _rbd_indices.clear();

        for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++) {
            if (_rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
                _rbd_indices.push_back(i);
        }

        _ef_index = _rbd_index(end_effector);

        _ik.reset(new rbd::InverseKinematics(_rbdyn_urdf.mb, _ef_index));
    }

    size_t IiwaTools::_rbd_index(const std::string& body_name) const
    {
        for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++) {
            if (_rbdyn_urdf.mb.body(i).name() == body_name) {
                return i;
            }
        }

        // TO-DO: Should never reach here
        return 0;
    }

    void IiwaTools::_update_urdf_state(mc_rbdyn_urdf::URDFParserResult& rbdyn_urdf, const RobotState& robot_state)
    {
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i];

            if (robot_state.position.size() > i)
                rbdyn_urdf.mbc.q[rbd_index][0] = robot_state.position[i];
            if (robot_state.velocity.size() > i)
                rbdyn_urdf.mbc.alpha[rbd_index][0] = robot_state.velocity[i];
            if (robot_state.torque.size() > i)
                rbdyn_urdf.mbc.jointTorque[rbd_index][0] = robot_state.torque[i];
        }
    }

} // namespace iiwa_tools
