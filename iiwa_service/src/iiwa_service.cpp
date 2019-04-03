#include <iiwa_service/cvxgen/iiwa_ik_cvxgen.hpp>
#include <iiwa_service/iiwa_service.h>

// RBDyn headers
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <SpaceVecAlg/Conversions.h>

namespace iiwa_service {
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

        _gravity_server = _nh.advertiseService(_gravity_service_name, &IiwaService::get_gravity, this);
        ROS_INFO_STREAM("Started Iiwa Gravity Compensation server..");
    }

    bool IiwaService::perform_fk(iiwa_service::GetFK::Request& request,
        iiwa_service::GetFK::Response& response)
    {
        if (request.joints.layout.dim.size() != 2 || request.joints.layout.dim[1].size != _rbd_indices.size()) {
            ROS_ERROR("Request joint angles not properly defined.");
            return false;
        }

        response.poses.resize(request.joints.layout.dim[0].size);
        
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

        for (size_t point = 0; point < request.joints.layout.dim[0].size; ++point) {
            rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                size_t rbd_index = _rbd_indices[i];
                double jt = get_multi_array(request.joints, point, i);
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

            response.poses[point].position.x = trans(0);
            response.poses[point].position.y = trans(1);
            response.poses[point].position.z = trans(2);

            response.poses[point].orientation.w = quat.w();
            response.poses[point].orientation.x = quat.x();
            response.poses[point].orientation.y = quat.y();
            response.poses[point].orientation.z = quat.z();
        }

        return true;
    }

    bool IiwaService::perform_ik(iiwa_service::GetIK::Request& request,
        iiwa_service::GetIK::Response& response)
    {
        bool seeds_provided = request.seed_angles.layout.dim.size() == 2 && (request.seed_angles.layout.dim[0].size == request.poses.size());
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        double damp = 1e-3;
        Eigen::VectorXd damping = Eigen::VectorXd::Ones(_rbd_indices.size()).array() * damp;
        if (request.damping.size() != _rbd_indices.size()) {
            ROS_WARN_STREAM("No damping parameters given. Using " << damping.transpose() << ".");
        }
        else {
            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                damping(i) = request.damping[i];
            }
        }

        double slack = 10000.;
        Eigen::VectorXd slack_vec = Eigen::VectorXd::Ones(6).array() * slack;
        if (request.slack.size() != 6) {
            ROS_WARN_STREAM("No slack parameters given. Using " << slack_vec.transpose() << ".");
        }
        else {
            for (size_t i = 0; i < 6; i++) {
                slack_vec(i) = request.slack[i];
            }
        }

        int max_iterations = request.max_iterations;
        if (max_iterations <= 0) {
            max_iterations = 50;
            ROS_WARN_STREAM("No max iterations given. Using " << max_iterations << " iterations.");
        }
        double tolerance = request.tolerance;
        if (tolerance <= 0.) {
            tolerance = 1e-5;
            ROS_WARN_STREAM("No tolerance given. Using " << tolerance << ".");
        }

        Eigen::VectorXd zero = Eigen::VectorXd::Zero(_rbd_indices.size());
        Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
        Eigen::VectorXd q_high = q_low;

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t index = _rbd_indices[i];
            q_low(i) = rbdyn_urdf.limits.lower[rbdyn_urdf.mb.joint(index).name()][0];
            q_high(i) = rbdyn_urdf.limits.upper[rbdyn_urdf.mb.joint(index).name()][0];
        }

        response.joints.layout.dim.resize(2);
        response.joints.layout.data_offset = 0;
        response.joints.layout.dim[0].size = request.poses.size();
        response.joints.layout.dim[0].stride = _rbd_indices.size();
        response.joints.layout.dim[1].size = _rbd_indices.size();
        response.joints.layout.dim[1].stride = 0;
        response.joints.data.resize(request.poses.size() * _rbd_indices.size());

        for (size_t point = 0; point < request.poses.size(); ++point) {
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            tf.col(3).head(3) << request.poses[point].position.x, request.poses[point].position.y, request.poses[point].position.z;

            tf.block(0, 0, 3, 3) = Eigen::Quaterniond(request.poses[point].orientation.w,
                request.poses[point].orientation.x,
                request.poses[point].orientation.y,
                request.poses[point].orientation.z)
                                       .normalized()
                                       .matrix();
            sva::PTransformd target_tf = sva::conversions::fromHomogeneous(tf);

            Eigen::VectorXd qref = Eigen::VectorXd::Zero(_rbd_indices.size());

            rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
            if (seeds_provided) {
                for (size_t i = 0; i < _rbd_indices.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];
                    double seed = get_multi_array(request.seed_angles, point, i);
                    // wrap in [-pi,pi]
                    seed = wrap_angle(seed);
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

            for (size_t j = 0; j < _rbd_indices.size(); j++) {
                size_t rbd_index = _rbd_indices[j];
                rbdyn_urdf.mbc.q[rbd_index][0] = q_best(j);
            }

            for (size_t joint = 0; joint < _rbd_indices.size(); ++joint) {
                size_t rbd_index = _rbd_indices[joint];
                set_multi_array(response.joints, point, joint, rbdyn_urdf.mbc.q[rbd_index][0]);
            }

            response.is_valid.push_back(iter < max_iterations);
            response.accepted_tolerance.push_back(best);
        }

        return true;
    }

    bool IiwaService::get_jacobian(iiwa_service::GetJacobian::Request& request,
        iiwa_service::GetJacobian::Response& response)
    {
        if (request.joint_angles.size() != _rbd_indices.size() || request.joint_angles.size() != request.joint_velocities.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }


        update_state();

        tools.jacobian(robot_state_);

        Eigen::MatrixXd jac_mat = tools.get_J_matrix();

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

    bool IiwaService::get_gravity(iiwa_service::GetGravity::Request& request,
        iiwa_service::GetGravity::Response& response)
    {
        if (request.joint_angles.size() != _rbd_indices.size() || request.joint_angles.size() != request.joint_velocities.size() || request.joint_angles.size() != request.joint_torques.size()) {
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

        update_state();

        tools.gravity(gravity, robot_state_);

        Eigen::VectorXd C = tool.get_C_matrix();
        
        // Fill response
        response.compensation_torques.resize(_rbd_indices.size());

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            response.compensation_torques[i] = C(i);
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
        n_p.param<std::string>("service/gravity_service_name", _gravity_service_name, "iiwa_gravity_server");
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

        tools.InitRBDyn();

        _rbd_indices = tools.get_indices();

        ROS_INFO_STREAM_NAMED("IiwaService", "Number of joints found: " << _rbd_indices.size());

    }

    void IiwaService::update_state()
    {
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            robot_state_.position[i] = wrap_angle(request.joint_angles[i]); // wrap in [-pi,pi]
            robot_state_.velocity = request.joint_velocities[i];
            robot_state_.torque = request.joint_torques[i];
        }
    }
} // namespace iiwa_service