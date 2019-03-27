#include <Eigen/Dense>

#include <pluginlib/class_list_macros.hpp>

#include <iiwa_control/custom_effort_controller.hpp>

#include <robot_controllers/CascadeController.hpp>
#include <robot_controllers/SumController.hpp>

namespace iiwa_control {
    double get_multi_array(const std_msgs::Float64MultiArray& array, size_t i, size_t j)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        return array.data[offset + i * array.layout.dim[0].stride + j];
    }

    CustomEffortController::CustomEffortController() {}

    CustomEffortController::~CustomEffortController() { sub_command_.shutdown(); }

    bool CustomEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
    {
        // List of controlled joints
        std::string param_name = "joints";
        if (!n.getParam(param_name, joint_names_)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

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

        // Get controller's parameters
        // std::vector<double> values;
        std::vector<std::string> controller_names, controller_types;

        n.param<std::string>("params/space", operation_space_, "joint"); // Default operation space is task-space
        n.param<std::string>("params/gravity", gravity_comp_, "off"); // We do not use gravity compensation by default
        n.getParam("controller_names", controller_names);
        n.getParam("controller_types", controller_types);

        // Check the operational space
        if (operation_space_ == "task") {
            space_dim_ = 6;
            // TO-DO: Get those from parameters
            iiwa_client_jacobian_ = n.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");
            iiwa_client_fk_ = n.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
        }
        else
            space_dim_ = n_joints_;

        // Init Controllers
        unsigned int ctrl_size = controller_names.size();

        if (ctrl_size == 0) {
            ROS_ERROR_STREAM("No controllers specified! Exiting..!");
            return false;
        }
        if (ctrl_size != controller_types.size()) {
            ROS_ERROR_STREAM("Sizes of controller names and controller types do match! Exiting..!");
            return false;
        }

        std::vector<ControllerPtr> controllers;

        for (unsigned int i = 0; i < ctrl_size; i++) {
            std::string name = controller_names[i];
            std::string type = controller_types[i];

            // ROS_WARN_STREAM(name << " ---> " << type);

            if (type == "SumController" || type == "CascadeController") {
                std::vector<std::string> subcontrollers;
                n.getParam("subcontrollers/" + name, subcontrollers);

                unsigned int sub_ctrl_size = subcontrollers.size();

                if (sub_ctrl_size == 0) {
                    ROS_ERROR_STREAM("No subcontrollers specified (at least one needed)! Exiting..!");
                    return false;
                }

                ControllerPtr big_ctrl;

                if (type == "SumController")
                    big_ctrl.reset(new robot_controllers::SumController);
                else
                    big_ctrl.reset(new robot_controllers::CascadeController);

                unsigned int num_ctrls = 0;
                for (unsigned int j = 0; j < sub_ctrl_size; j++) {
                    std::string sub_name = subcontrollers[j];
                    auto ctrl = manager_.loadAndInstantiate(sub_name);
                    if (ctrl) {
                        robot_controllers::RobotParams params;
                        params.input_dim_ = space_dim_;
                        params.output_dim_ = space_dim_;

                        params.time_step_ = 0.01; // TO-DO: Get this from controller manager or yaml

                        n.getParam("params/" + name + sub_name, params.values_);

                        ctrl->SetParams(params);

                        if (type == "SumController")
                            static_cast<robot_controllers::SumController*>(big_ctrl.get())->AddController(std::move(ctrl));
                        else
                            static_cast<robot_controllers::CascadeController*>(big_ctrl.get())->AddController(std::move(ctrl));
                        num_ctrls++;
                    }
                }

                if (num_ctrls == 0) {
                    ROS_ERROR_STREAM("Could not load specified subcontrollers for " << name << "! Exiting..!");
                    return false;
                }

                robot_controllers::RobotParams params;
                params.input_dim_ = space_dim_;
                params.output_dim_ = space_dim_;

                params.time_step_ = 0.01; // TO-DO: Get this from controller manager or yaml
                big_ctrl->SetParams(params);

                controllers.emplace_back(std::move(big_ctrl));
            }
            else {
                auto ctrl = manager_.loadAndInstantiate(type);

                if (ctrl) {
                    robot_controllers::RobotParams params;
                    params.input_dim_ = space_dim_;
                    params.output_dim_ = space_dim_;

                    params.time_step_ = 0.01; // TO-DO: Get this from controller manager or yaml

                    n.getParam("params/" + name, params.values_);

                    ctrl->SetParams(params);

                    controllers.emplace_back(std::move(ctrl));
                }
            }
        }

        ctrl_size = controllers.size();

        if (ctrl_size == 0) {
            ROS_ERROR_STREAM("Could not load specified controllers! Exiting..!");
            return false;
        }

        if (ctrl_size == 1) {
            controller_ = std::move(controllers[0]);
        }
        else {
            controller_.reset(new robot_controllers::SumController);
            for (unsigned int i = 0; i < ctrl_size; i++) {
                static_cast<robot_controllers::SumController*>(controller_.get())->AddController(std::move(controllers[i]));
            }
        }

        // Initialize the controller(s)
        controller_->Init();

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

        fk_srv_.request.joints.layout.dim.resize(2);
        fk_srv_.request.joints.layout.dim[0].size = 1;
        fk_srv_.request.joints.layout.dim[0].stride = n_joints_;
        fk_srv_.request.joints.layout.dim[1].size = n_joints_;
        fk_srv_.request.joints.layout.dim[1].stride = 0;
        fk_srv_.request.joints.layout.data_offset = 0;
        fk_srv_.request.joints.data.resize(n_joints_);

        // Get controller command size!
        cmd_dim_ = 0;
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Position) {
            cmd_dim_ += space_dim_;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Velocity) {
            cmd_dim_ += space_dim_;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Acceleration) {
            cmd_dim_ += space_dim_;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Force) {
            cmd_dim_ += space_dim_;
        }

        std::vector<double> init_cmd(cmd_dim_, 0.0);
        if (operation_space_ == "task") {
            // if task space, we need to alter the initial command
            for (size_t i = 0; i < n_joints_; i++) {
                fk_srv_.request.joints.data[i] = joints_[i].getPosition();
            }

            if (iiwa_client_fk_.call(fk_srv_)) {
                assert(fk_srv_.response.poses.size() == 1);

                Eigen::Quaterniond quat(fk_srv_.response.poses[0].orientation.w, fk_srv_.response.poses[0].orientation.x, fk_srv_.response.poses[0].orientation.y, fk_srv_.response.poses[0].orientation.z);
                Eigen::AngleAxisd aa(quat);

                Eigen::VectorXd o = aa.axis() * aa.angle();
                Eigen::VectorXd p(3);
                p << fk_srv_.response.poses[0].position.x, fk_srv_.response.poses[0].position.y, fk_srv_.response.poses[0].position.z;

                for (size_t i = 0; i < 3; i++) {
                    init_cmd[i] = o(i);
                    init_cmd[i + 3] = p(i);
                }
            }
        }

        ROS_INFO_STREAM("Initial command: " << Eigen::VectorXd::Map(init_cmd.data(), init_cmd.size()).transpose());

        commands_buffer_.writeFromNonRT(init_cmd);

        sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &CustomEffortController::commandCB, this);

        return true;
    }

    void CustomEffortController::update(const ros::Time& time, const ros::Duration& period)
    {
        std::vector<double>& commands = *commands_buffer_.readFromRT();

        Eigen::MatrixXd jac(6, n_joints_);
        Eigen::VectorXd eef(6);
        bool jac_valid = false;
        bool fk_valid = false;

        if (operation_space_ == "task") {
            // Call iiwa tools service for jacobian
            for (size_t i = 0; i < n_joints_; i++) {
                jacobian_srv_.request.joint_angles[i] = joints_[i].getPosition();
                jacobian_srv_.request.joint_velocities[i] = joints_[i].getVelocity();

                fk_srv_.request.joints.data[i] = joints_[i].getPosition();
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

            if (iiwa_client_fk_.call(fk_srv_)) {
                assert(fk_srv_.response.poses.size() == 1);

                Eigen::Quaterniond quat(fk_srv_.response.poses[0].orientation.w, fk_srv_.response.poses[0].orientation.x, fk_srv_.response.poses[0].orientation.y, fk_srv_.response.poses[0].orientation.z);
                Eigen::AngleAxisd aa(quat);

                eef.head(3) = aa.axis() * aa.angle();
                eef.tail(3) << fk_srv_.response.poses[0].position.x, fk_srv_.response.poses[0].position.y, fk_srv_.response.poses[0].position.z;

                fk_valid = true;
            }
            else {
                ROS_ERROR_STREAM("Could not get Forward Kinematics!");
            }
        }

        Eigen::VectorXd cmd(n_joints_);

        cmd = Eigen::VectorXd::Map(commands.data(), commands.size());

        robot_controllers::RobotState curr_state;
        curr_state.position_ = Eigen::VectorXd::Zero(n_joints_);
        curr_state.velocity_ = Eigen::VectorXd::Zero(n_joints_);
        curr_state.acceleration_ = Eigen::VectorXd::Zero(n_joints_);
        curr_state.force_ = Eigen::VectorXd::Zero(n_joints_);

        for (unsigned int i = 0; i < n_joints_; i++) {
            curr_state.position_(i) = joints_[i].getPosition();
            curr_state.velocity_(i) = joints_[i].getVelocity();
            // curr_state.acceleration_(i) = joints_[i].getAcceleration();
            // TO-DO: Fill acceleration
            curr_state.force_(i) = joints_[i].getEffort();
        }

        if (operation_space_ == "task") {
            if (!jac_valid) {
                ROS_ERROR_STREAM("Jacobian not valid! Skipping this update!");
                return;
            }

            if (!fk_valid) {
                ROS_ERROR_STREAM("Forward Kinematics not valid! Skipping this update!");
                return;
            }

            curr_state.position_ = eef;
            curr_state.velocity_ = jac * curr_state.velocity_;
            curr_state.acceleration_ = jac * curr_state.acceleration_;
            curr_state.force_ = jac * curr_state.force_;
        }

        // Update desired state in controller
        robot_controllers::RobotState desired_state;
        unsigned int size = curr_state.position_.size();
        unsigned int index = 0;
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Position) {
            desired_state.position_ = cmd.segment(index, size);
            index += size;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Velocity) {
            desired_state.velocity_ = cmd.segment(index, size);
            index += size;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Acceleration) {
            desired_state.acceleration_ = cmd.segment(index, size);
            index += size;
        }
        if (controller_->GetInput().GetType() & robot_controllers::IOType::Force) {
            desired_state.force_ = cmd.segment(index, size);
            // index += size;
        }
        controller_->SetInput(desired_state);

        // Update control torques given current velocity
        controller_->Update(curr_state);

        Eigen::VectorXd output = controller_->GetOutput().desired_.force_;
        // Eigen::VectorXd output = 10. * (desired_vel - curr_vel).transpose().array();

        if (operation_space_ == "task" && jac_valid) {
            // output.head(3) = Eigen::VectorXd::Zero(3);
            output = jac.transpose() * output;
        }

        // ROS_INFO_STREAM("Effort: " << output.transpose());

        std::vector<double> commanded_effort(n_joints_, 0.);

        Eigen::VectorXd::Map(commanded_effort.data(), commanded_effort.size()) = output;

        for (unsigned int i = 0; i < n_joints_; i++) {
            enforceJointLimits(commanded_effort[i], i);
            joints_[i].setCommand(commanded_effort[i]);
        }
    }

    void CustomEffortController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        if (msg->data.size() != cmd_dim_) {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") is not correct! Not executing!");
            return;
        }

        commands_buffer_.writeFromNonRT(msg->data);
    }

    void CustomEffortController::enforceJointLimits(double& command, unsigned int index)
    {
        // Check that this joint has applicable limits
        if (command > joint_urdfs_[index]->limits->effort) // above upper limit
        {
            command = joint_urdfs_[index]->limits->effort;
        }
        else if (command < -joint_urdfs_[index]->limits->effort) // below lower limit
        {
            command = -joint_urdfs_[index]->limits->effort;
        }
    }
} // namespace iiwa_control

PLUGINLIB_EXPORT_CLASS(iiwa_control::CustomEffortController, controller_interface::ControllerBase)
