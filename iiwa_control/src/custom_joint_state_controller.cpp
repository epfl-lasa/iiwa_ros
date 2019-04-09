#include <Eigen/Core>

#include <pluginlib/class_list_macros.hpp>

#include <iiwa_control/custom_joint_state_controller.hpp>

namespace iiwa_control {

    CustomJointStateController::CustomJointStateController() {}

    CustomJointStateController::~CustomJointStateController() { }

    bool CustomJointStateController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh_root, ros::NodeHandle& nh_controller)
    {
        // List of controlled joints
        joint_names_ = hw->getNames();
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0) {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }

        std::cerr << n_joints_<< std::endl;
        for (unsigned i=0; i<n_joints_; i++)
            ROS_INFO("Got joint %s", joint_names_[i].c_str());

        // Get publishing period
        if (!nh_controller.getParam("publish_rate", publish_rate_)){
          ROS_ERROR("Parameter 'publish_rate' not set");
          return false;
        }

        // Get URDF
        urdf::Model urdf;
        if (!urdf.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }

        // // Get the URDF XML from the parameter server
        std::string urdf_string, full_param;
        std::string robot_description = "robot_description";
        std::string end_effector;

        // // gets the location of the robot description on the parameter server
        if (!nh_controller.searchParam(robot_description, full_param)) {
            ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
            return false;
        }

        // // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("CustomJointStateController", "CustomJointStateController is waiting for model"
                                                          " URDF in parameter [%s] on the ROS param server.",
                robot_description.c_str());

            nh_controller.getParam(full_param, urdf_string);

            usleep(100000);
        }

        ROS_INFO_STREAM_NAMED("CustomJointStateController", "Received urdf from param server, parsing...");

        // // Get the end-effector
        nh_controller.param<std::string>("end_effector", end_effector, "iiwa_link_ee");

        // // Initialize iiwa tools
        // std::cerr << "AAAAA: " << urdf_string << std::endl;
        // std::cerr << "BBBBB: " << end_effector << std::endl;
        tools_.init_rbdyn(urdf_string, end_effector);




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
            iiwa_tools::RobotState robot_state;
            robot_state.position.resize(n_joints_);
            robot_state.velocity.resize(n_joints_);

            for (size_t i = 0; i < n_joints_; i++) {
                robot_state.position[i] = joints_[i].getPosition();
                robot_state.velocity[i] = joints_[i].getVelocity();
            }

            auto ee_state = tools_.perform_fk(robot_state);

            x_ = ee_state.translation;
            q_ << ee_state.orientation.w(), ee_state.orientation.x(), ee_state.orientation.y(), ee_state.orientation.z();
                
            // Get jacobian and end effector twist
            Eigen::MatrixXd jac(6, n_joints_);

            jac = tools_.jacobian(robot_state);

            Eigen::Vector3d v, omega;
            Eigen::VectorXd twist, j, jv;
            twist.resize(6);
            j.resize(n_joints_);
            jv.resize(n_joints_);

            for (unsigned int i = 0; i < n_joints_; i++) {
                j(i) = joints_[i].getPosition();
                jv(i) = joints_[i].getVelocity();
            }

            twist = jac * jv;
            omega_ = twist.segment(0,3);
            v_ = twist.segment(3,3);

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
