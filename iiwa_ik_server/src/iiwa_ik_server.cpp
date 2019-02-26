#include <iiwa_ik_server/iiwa_ik_server.h>

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
        sensor_msgs::JointState joint_state;

        bool seeds_provided = request.seed_angles.size() == request.pose_stamp.size();

        for (size_t point = 0; point < request.pose_stamp.size(); ++point) {
            joint_state.position.clear();

            sva::PTransformd target_tf(Eigen::Quaterniond(request.pose_stamp[point].pose.orientation.w,
                                           request.pose_stamp[point].pose.orientation.x,
                                           request.pose_stamp[point].pose.orientation.y,
                                           request.pose_stamp[point].pose.orientation.z),
                Eigen::Vector3d(request.pose_stamp[point].pose.position.x,
                    request.pose_stamp[point].pose.position.y,
                    request.pose_stamp[point].pose.position.z));

            _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);
            if (seeds_provided) {
                sensor_msgs::JointState& js = request.seed_angles[point];
                for (size_t i = 0; i < js.position.size(); i++) {
                    size_t rbd_index = _rbd_indices[i];

                    _rbdyn_urdf.mbc.q[rbd_index][0] = js.position[i];
                }
            }

            int max_iterations = request.max_iterations;
            double threshold = request.tolerance;

            if (max_iterations > 0)
                _ik->max_iterations_ = max_iterations;
            if (threshold > 0.)
                _ik->threshold_ = threshold;

            bool valid = _ik->inverseKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc, target_tf);

            for (size_t joint = 0; joint < _rbd_indices.size(); ++joint) {
                size_t rbd_index = _rbd_indices[joint];

                joint_state.position.push_back(_rbdyn_urdf.mbc.q[rbd_index][0]);
            }

            response.joints.push_back(joint_state);
            response.isValid.push_back(valid);
        }
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