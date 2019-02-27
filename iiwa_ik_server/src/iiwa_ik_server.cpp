#include <iiwa_ik_server/cvxgen/solver.h>
#include <iiwa_ik_server/iiwa_ik_server.h>

// RBDyn headers
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace iiwa_ik_cvxgen {
    Vars vars;
    Params params;
    Workspace work;
    Settings settings;
} // namespace iiwa_ik_cvxgen

namespace iiwa_ik_server {
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
        // sensor_msgs::JointState joint_state;

        // bool seeds_provided = request.seed_angles.size() == request.pose_stamp.size();

        // for (size_t point = 0; point < request.pose_stamp.size(); ++point) {
        //     joint_state.position.clear();

        //     sva::PTransformd target_tf(Eigen::Quaterniond(request.pose_stamp[point].pose.orientation.w,
        //                                    request.pose_stamp[point].pose.orientation.x,
        //                                    request.pose_stamp[point].pose.orientation.y,
        //                                    request.pose_stamp[point].pose.orientation.z)
        //                                    .normalized(),
        //         Eigen::Vector3d(request.pose_stamp[point].pose.position.x,
        //             request.pose_stamp[point].pose.position.y,
        //             request.pose_stamp[point].pose.position.z));

        //     _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);
        //     if (seeds_provided) {
        //         sensor_msgs::JointState& js = request.seed_angles[point];
        //         for (size_t i = 0; i < js.position.size(); i++) {
        //             size_t rbd_index = _rbd_indices[i];

        //             _rbdyn_urdf.mbc.q[rbd_index][0] = js.position[i];
        //         }
        //     }

        //     int max_iterations = request.max_iterations;
        //     double threshold = request.tolerance;

        //     if (max_iterations > 0)
        //         _ik->max_iterations_ = max_iterations;
        //     if (threshold > 0.)
        //         _ik->threshold_ = threshold;

        //     bool valid = _ik->inverseKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc, target_tf);

        //     for (size_t joint = 0; joint < _rbd_indices.size(); ++joint) {
        //         size_t rbd_index = _rbd_indices[joint];

        //         joint_state.position.push_back(_rbdyn_urdf.mbc.q[rbd_index][0]);
        //     }

        //     response.joints.push_back(joint_state);
        //     response.isValid.push_back(valid);
        // }

        sensor_msgs::JointState joint_state;

        bool seeds_provided = request.seed_angles.size() == request.pose_stamp.size();

        double damp = 1e-3;
        Eigen::VectorXd damping = Eigen::VectorXd::Ones(_rbd_indices.size()).array() * damp;

        double slack = 100.;
        Eigen::VectorXd slack_vec = Eigen::VectorXd::Ones(6).array() * slack;

        int num_iters = 15;

        Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
        Eigen::VectorXd q_high = q_low;

        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t index = _rbd_indices[i];
            q_low(i) = _rbdyn_urdf.limits.lower[_rbdyn_urdf.mb.joint(index).name()][0];
            q_high(i) = _rbdyn_urdf.limits.upper[_rbdyn_urdf.mb.joint(index).name()][0];
        }

        for (size_t point = 0; point < request.pose_stamp.size(); ++point) {
            joint_state.position.clear();
            sva::PTransformd target_tf(Eigen::Quaterniond(request.pose_stamp[point].pose.orientation.w,
                                           request.pose_stamp[point].pose.orientation.x,
                                           request.pose_stamp[point].pose.orientation.y,
                                           request.pose_stamp[point].pose.orientation.z)
                                           .normalized(),
                Eigen::Vector3d(request.pose_stamp[point].pose.position.x,
                    request.pose_stamp[point].pose.position.y,
                    request.pose_stamp[point].pose.position.z));

            Eigen::VectorXd qref = Eigen::VectorXd::Zero(_rbd_indices.size());

            _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);
            if (seeds_provided) {
                sensor_msgs::JointState& js = request.seed_angles[point];
                for (size_t i = 0; i < js.position.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];

                    _rbdyn_urdf.mbc.q[rbd_index][0] = js.position[i];
                    qref(i) = js.position[i];
                }
            }

            rbd::Jacobian jac(_rbdyn_urdf.mb, _rbdyn_urdf.mb.body(_ef_index).name());

            for (int i = 0; i < num_iters; i++) {
                rbd::forwardKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
                rbd::forwardVelocity(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

                Eigen::Vector3d rotErr = sva::rotationError(_rbdyn_urdf.mbc.bodyPosW[_ef_index].rotation(), target_tf.rotation());
                Eigen::Vector6d v;
                v << rotErr, target_tf.translation() - _rbdyn_urdf.mbc.bodyPosW[_ef_index].translation();

                Eigen::MatrixXd jacMat = jac.jacobian(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

                iiwa_ik_cvxgen::set_defaults();
                iiwa_ik_cvxgen::setup_indexing();
                iiwa_ik_cvxgen::settings.verbose = 0;

                // set params
                for (int j = 0; j < 6; j++) {
                    memcpy(iiwa_ik_cvxgen::params.J[j+1], jacMat.row(j).data(), _rbd_indices.size() * sizeof(double));
                }
                memcpy(iiwa_ik_cvxgen::params.damping, damping.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.slack, slack_vec.data(), 6 * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.qref, qref.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.qlow, q_low.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.qup, q_high.data(), _rbd_indices.size() * sizeof(double));
                memcpy(iiwa_ik_cvxgen::params.dx, v.data(), 6 * sizeof(double));

                iiwa_ik_cvxgen::solve();

                Eigen::VectorXd dq(_rbd_indices.size());
                for (size_t j = 0; j < _rbd_indices.size(); j++) {
                    dq(j) = iiwa_ik_cvxgen::vars.dq[j];
                }

                qref += dq;

                for (size_t j = 0; j < _rbd_indices.size(); j++) {
                    size_t rbd_index = _rbd_indices[j];

                    _rbdyn_urdf.mbc.q[rbd_index][0] = qref(j);
                }
            }

            for (size_t joint = 0; joint < _rbd_indices.size(); ++joint) {
                size_t rbd_index = _rbd_indices[joint];

                joint_state.position.push_back(_rbdyn_urdf.mbc.q[rbd_index][0]);
            }

            response.joints.push_back(joint_state);
            response.isValid.push_back(true);
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