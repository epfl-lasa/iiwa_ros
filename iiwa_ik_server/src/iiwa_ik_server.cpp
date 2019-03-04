#include <iiwa_ik_server/cvxgen/solver.h>
#include <iiwa_ik_server/iiwa_ik_server.h>

// RBDyn headers
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <SpaceVecAlg/Conversions.h>

namespace iiwa_ik_cvxgen {
    Vars vars;
    Params params;
    Workspace work;
    Settings settings;
} // namespace iiwa_ik_cvxgen

namespace iiwa_ik_server {
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

    IiwaIKServer::IiwaIKServer(ros::NodeHandle nh) : _nh(nh)
    {
        ROS_INFO_STREAM("Starting Iiwa IK server..");
        _load_params();
        _init_rbdyn();

        _ik_server = _nh.advertiseService(_service_name, &IiwaIKServer::perform_ik, this);
        ROS_INFO_STREAM("Started Iiwa IK server..");
    }

    bool IiwaIKServer::perform_ik(iiwa_ik_server::GetIK::Request& request,
        iiwa_ik_server::GetIK::Response& response)
    {
        bool seeds_provided = request.seed_angles.layout.dim.size() == 2 && (request.seed_angles.layout.dim[0].size == request.poses.size());

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
            q_low(i) = _rbdyn_urdf.limits.lower[_rbdyn_urdf.mb.joint(index).name()][0];
            q_high(i) = _rbdyn_urdf.limits.upper[_rbdyn_urdf.mb.joint(index).name()][0];
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

            _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);
            if (seeds_provided) {
                for (size_t i = 0; i < _rbd_indices.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];
                    double seed = get_multi_array(request.seed_angles, point, i);

                    _rbdyn_urdf.mbc.q[rbd_index][0] = seed;
                    qref(i) = seed;
                }
            }

            // Solve IK with traditional approach and pass it as a seed if successful
            bool valid = _ik->inverseKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc, target_tf);
            if (valid) {
                for (size_t i = 0; i < _rbd_indices.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];
                    qref(i) = wrap_angle(_rbdyn_urdf.mbc.q[rbd_index][0]);
                }
                ROS_DEBUG_STREAM("Using seed from RBDyn: " << qref.transpose());
            }

            rbd::Jacobian jac(_rbdyn_urdf.mb, _rbdyn_urdf.mb.body(_ef_index).name());

            double best = std::numeric_limits<double>::max();
            Eigen::VectorXd q_best = qref;

            int iter = 0;
            double error = 0.;
            for (iter = 0; iter < max_iterations; iter++) {
                rbd::forwardKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
                rbd::forwardVelocity(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

                Eigen::Vector3d rotErr = sva::rotationError(_rbdyn_urdf.mbc.bodyPosW[_ef_index].rotation(), target_tf.rotation());
                Eigen::Vector6d v;
                v << rotErr, target_tf.translation() - _rbdyn_urdf.mbc.bodyPosW[_ef_index].translation();

                error = v.norm();

                if (error < best) {
                    best = error;
                    q_best = qref;
                }

                if (error < tolerance)
                    break;

                Eigen::MatrixXd jacMat = jac.jacobian(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

                iiwa_ik_cvxgen::set_defaults();
                iiwa_ik_cvxgen::setup_indexing();
                iiwa_ik_cvxgen::settings.verbose = 0;
                iiwa_ik_cvxgen::settings.resid_tol = 1e-10;
                iiwa_ik_cvxgen::settings.eps = 1e-10;
                iiwa_ik_cvxgen::settings.max_iters = 100;

                // set params
                for (int r = 0; r < 6; r++) {
                    for (int c = 0; c < _rbd_indices.size(); c++) {
                        iiwa_ik_cvxgen::params.J[r + 1][c] = jacMat(r, c);
                    }
                }

                // adapt the limits
                Eigen::VectorXd qlow = q_low - qref;
                Eigen::VectorXd qhigh = q_high - qref;

                memcpy(iiwa_ik_cvxgen::params.damping, damping.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.slack, slack_vec.data(), 6 * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.qref, zero.data(), _rbd_indices.size() * sizeof(double)); // we set qref to zero so that we minimize the dq
                memcpy(iiwa_ik_cvxgen::params.qlow, qlow.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.qup, qhigh.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.dx, v.data(), 6 * sizeof(double));

                iiwa_ik_cvxgen::solve();

                Eigen::VectorXd q_prev = qref;

                for (size_t j = 0; j < _rbd_indices.size(); j++) {
                    qref(j) += iiwa_ik_cvxgen::vars.dq[j];
                }

                for (size_t j = 0; j < _rbd_indices.size(); j++) {
                    size_t rbd_index = _rbd_indices[j];
                    _rbdyn_urdf.mbc.q[rbd_index][0] = qref(j);
                }

                if ((q_prev - qref).norm() < 1e-8)
                    break;
            }

            for (size_t j = 0; j < _rbd_indices.size(); j++) {
                size_t rbd_index = _rbd_indices[j];
                _rbdyn_urdf.mbc.q[rbd_index][0] = q_best(j);
            }

            for (size_t joint = 0; joint < _rbd_indices.size(); ++joint) {
                size_t rbd_index = _rbd_indices[joint];
                set_multi_array(response.joints, point, joint, _rbdyn_urdf.mbc.q[rbd_index][0]);
            }

            response.is_valid.push_back(iter < max_iterations);
            response.accepted_tolerance.push_back(best);
        }

        return true;
    }

    void IiwaIKServer::_load_params()
    {
        ros::NodeHandle n_p("~");

        n_p.param<std::string>("ik/robot_description", _robot_description, "/robot_description");
        n_p.param<std::string>("ik/end_effector", _end_effector, "iiwa_link_ee");
        n_p.param<std::string>("ik/name", _service_name, "iiwa_ik_server");
    }

    void IiwaIKServer::_init_rbdyn()
    {
        // Get the URDF XML from the parameter server
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("IiwaIKServer", "IiwaIKServer is waiting for model"
                                                " URDF in parameter [%s] on the ROS param server.",
                _robot_description.c_str());

            _nh.getParam(_robot_description, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("IiwaIKServer", "Received urdf from param server, parsing...");

        // Convert URDF to RBDyn
        _rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);
        // _fd = rbd::InverseKinematics(_rbdyn_urdf.mb);

        _rbd_indices.clear();

        for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++) {
            if (_rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
                _rbd_indices.push_back(i);
        }

        ROS_INFO_STREAM_NAMED("IiwaIKServer", "Number of joints found: " << _rbd_indices.size());

        _ef_index = _rbd_index(_end_effector);

        _ik.reset(new rbd::InverseKinematics(_rbdyn_urdf.mb, _ef_index));
    }

    size_t IiwaIKServer::_rbd_index(const std::string& body_name) const
    {
        for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++) {
            if (_rbdyn_urdf.mb.body(i).name() == body_name) {
                return i;
            }
        }

        // TO-DO: Should never reach here
        return 0;
    }
} // namespace iiwa_ik_server