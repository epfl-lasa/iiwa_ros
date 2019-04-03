#include <Eigen/Core>

#include <pluginlib/class_list_macros.hpp>

#include <iiwa_control/custom_effort_controller.hpp>
#include <iiwa_control/custom_joint_state_controller.hpp>

namespace iiwa_control {

    CustomJointStateController::CustomJointStateController() {}

    CustomJointStateController::~CustomJointStateController() { }

    bool CustomJointStateController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh_root, ros::NodeHandle& nh_controller)
    {
        // List of controlled joints
        joint_names_ = hw->getNames();
        n_joints_ = joint_names_.size();
        for (unsigned i=0; i<n_joints_; i++)
            ROS_INFO("Got joint %s", joint_names_[i].c_str());

        // Get publishing period
        if (!nh_controller.getParam("publish_rate", publish_rate_)){
          ROS_ERROR("Parameter 'publish_rate' not set");
          return false;
        }

        // n_joints_ = joint_names_.size();

        if (n_joints_ == 0) {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }

        // Get URDF
        urdf::Model urdf;
        if (!urdf.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }

        iiwa_client_fk_ = nh_root.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
        iiwa_client_jacobian_ = nh_root.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");

        for (unsigned int i = 0; i < n_joints_; i++) {
            try {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf) {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }


        // Setup services
        jacobian_srv_.request.joint_angles.resize(n_joints_, 0.);
        jacobian_srv_.request.joint_velocities.resize(n_joints_, 0.);

        fk_srv_.request.joints.data.resize(n_joints_);
        fk_srv_.request.joints.layout.dim.resize(2);
        fk_srv_.request.joints.layout.dim[0].size = 1;
        fk_srv_.request.joints.layout.dim[1].size = n_joints_;

        // Setup publishers
        std::cerr << nh_root.getNamespace() << " "  << nh_controller.getNamespace() <<std::endl;

        pub_pose_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(nh_root,"ee_pose",1));
        pub_twist_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh_root,"ee_twist",1));
        pub_joints_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh_root,"joint_states",1));

        for (unsigned int i=0; i<n_joints_; i++) {
            pub_joints_->msg_.name.push_back(joint_names_[i]);
            pub_joints_->msg_.position.push_back(0.0);
            pub_joints_->msg_.velocity.push_back(0.0);
            pub_joints_->msg_.effort.push_back(0.0);
        }

        return true;
    }

    void CustomJointStateController::update(const ros::Time& time, const ros::Duration& period)
    {

        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {
            // Get current end effector pose
            bool fk_valid = false;
            for (size_t i = 0; i < n_joints_; i++) {
                fk_srv_.request.joints.data[i] = joints_[i].getPosition();
            }
            Eigen::Vector3d x;
            Eigen::Vector4d q;
            if (iiwa_client_fk_.call(fk_srv_)) {
                x_ << fk_srv_.response.poses[0].position.x, fk_srv_.response.poses[0].position.y, fk_srv_.response.poses[0].position.z;
                q_ << fk_srv_.response.poses[0].orientation.w, fk_srv_.response.poses[0].orientation.x, fk_srv_.response.poses[0].orientation.y, fk_srv_.response.poses[0].orientation.z;
                fk_valid = true;
            }
                
            // Get jacobian and end effector twist
            Eigen::MatrixXd jac(6, n_joints_);
            bool jac_valid = false;

            for (size_t i = 0; i < n_joints_; i++) {
                jacobian_srv_.request.joint_angles[i] = joints_[i].getPosition();
                jacobian_srv_.request.joint_velocities[i] = joints_[i].getVelocity();
            }

            if (iiwa_client_jacobian_.call(jacobian_srv_)) {
                assert(jacobian_srv_.response.jacobian.layout.dim.size() == 2); // we need a 2D array
                assert(jacobian_srv_.response.jacobian.layout.dim[0].size == 6); // check if Jacobian has proper dimensions
                assert(jacobian_srv_.response.jacobian.layout.dim[1].size == n_joints_);

                for (size_t r = 0; r < 6; r++) {
                    for (size_t c = 0; c < n_joints_; c++) {
                        jac(r, c) = get_multi_array(jacobian_srv_.response.jacobian, r, c);
                    }
                }

                jac_valid = true;
            }
            else {
                ROS_ERROR_STREAM("Could not get Jacobian!");
            }

            Eigen::Vector3d v, omega;
            Eigen::VectorXd twist, j, jv;
            twist.resize(6);
            j.resize(n_joints_);
            jv.resize(n_joints_);

            for (unsigned int i = 0; i < n_joints_; i++) {
                j(i) = joints_[i].getPosition();
                jv(i) = joints_[i].getVelocity();
            }

            if(jac_valid) {
                twist = jac * jv;
                omega_ = twist.segment(0,3);
                v_ = twist.segment(3,3);
            }

            // Publish pos/vel/acc
            if(pub_pose_->trylock()) {
                pub_pose_->msg_.position.x = x_(0);
                pub_pose_->msg_.position.y = x_(1);
                pub_pose_->msg_.position.z = x_(2);
                pub_pose_->msg_.orientation.w = q_(0);
                pub_pose_->msg_.orientation.x = q_(1);
                pub_pose_->msg_.orientation.y = q_(2);
                pub_pose_->msg_.orientation.z = q_(3);

                pub_pose_->unlockAndPublish();
            }

            if(pub_twist_->trylock())  {
                pub_twist_->msg_.linear.x = v_(0);
                pub_twist_->msg_.linear.y = v_(1);
                pub_twist_->msg_.linear.z = v_(2);

                pub_twist_->msg_.angular.x = omega_(0);
                pub_twist_->msg_.angular.y = omega_(1);
                pub_twist_->msg_.angular.z = omega_(2);

                pub_twist_->unlockAndPublish();
            }

            if(pub_joints_->trylock())  {

                last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

                pub_joints_->msg_.header.stamp = time;
                for(unsigned int i=0; i < n_joints_; i++) {
                    pub_joints_->msg_.position[i] = joints_[i].getPosition();
                    pub_joints_->msg_.velocity[i] = joints_[i].getVelocity();
                    pub_joints_->msg_.effort[i] = joints_[i].getEffort();
                }
                pub_joints_->unlockAndPublish();
            }
        }
    }
} // namespace iiwa_control

PLUGINLIB_EXPORT_CLASS(iiwa_control::CustomJointStateController, controller_interface::ControllerBase)
