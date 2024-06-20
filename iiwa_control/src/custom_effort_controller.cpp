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
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

#include <pluginlib/class_list_macros.hpp>

#include <iiwa_control/custom_effort_controller.hpp>

#include <Corrade/Containers/PointerStl.h>

#include <robot_controllers/CascadeController.hpp>
#include <robot_controllers/SumController.hpp>

#define WEIGHT_DEFAULT 1.0  ///< Default weight for a controller output.

class WeightedSumController : public robot_controllers::SumController
{
 public:
    void Update(const robot_controllers::RobotState& state)
    {
        robot_controllers::RobotState result;
#define add_result(name)                                                                   \
    {                                                                                      \
        if (result.name.size() == 0) result.name = Eigen::VectorXd::Zero(out.name.size()); \
        result.name += weight * out.name;                                                  \
    }

        for (unsigned int i = 0; i < controllers_.size(); i++)
        {
            auto& ctrl = controllers_[i];
            ctrl->SetInput(input_.desired_);
            ctrl->Update(state);
            robot_controllers::RobotState out = ctrl->GetOutput().desired_;
            const float& weight = weights_[i];

            robot_controllers::IOTypes t = ctrl->GetOutput().GetType();
            if (t & robot_controllers::IOType::Position) add_result(position_);
            if (t & robot_controllers::IOType::Orientation) add_result(orientation_);
            if (t & robot_controllers::IOType::Velocity) add_result(velocity_);
            if (t & robot_controllers::IOType::AngularVelocity) add_result(angular_velocity_);
            if (t & robot_controllers::IOType::Acceleration) add_result(acceleration_);
            if (t & robot_controllers::IOType::AngularAcceleration) add_result(angular_acceleration_);
            if (t & robot_controllers::IOType::Force) add_result(force_);
            if (t & robot_controllers::IOType::Torque) add_result(torque_);
        }
#undef add_result
        output_.desired_ = result;
    }

    void AddController(std::unique_ptr<AbstractController> controller)
    {
        weights_.emplace_back(WEIGHT_DEFAULT);
        robot_controllers::SumController::AddController(std::move(controller));
    }

    template <typename T, typename... Args>
    void AddController(Args... args)
    {
        weights_.emplace_back(WEIGHT_DEFAULT);
        robot_controllers::SumController::AddController(std::forward<Args>(args)...);
    }

    void SetWeight(float weight, unsigned int index)
    {
        assert(index < weights_.size());
        weights_[index] = weight;
    }

    void SetWeights(const std::vector<float>& weights)
    {
        assert(weights.size() == weights_.size());
        weights_ = weights;
    }

 protected:
    std::vector<float> weights_;  ///< Weigths on the outputs of each controllers.
};  // WeightedSumController

namespace iiwa_control
{
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(
    const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4})  // choose appropriately
{
    // More efficient (~2x) and numerically stable than with jacobiSvd
    return mat.completeOrthogonalDecomposition().pseudoInverse();
}

std::vector<std::vector<std::string>> get_types(const std::string& input, const std::string& output)
{
    std::vector<std::vector<std::string>> result(2);

    // Input
    std::string s = input;
    std::string delimiter = "|";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        result[0].push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    // add the last one to the list
    result[0].push_back(s);

    // Output
    s = output;
    delimiter = "|";

    pos = 0;
    token = "";
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        result[1].push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    // add the last one to the list
    result[1].push_back(s);

    return result;
}

void set_types(CustomEffortController::ControllerPtr& ctrl,
               const std::vector<std::string>& input,
               const std::vector<std::string>& output)
{
    robot_controllers::IOTypes input_type, output_type;
    for (auto& s : input)
    {
        if (s == "Position")
            input_type = input_type | robot_controllers::IOType::Position;
        else if (s == "Orientation")
            input_type = input_type | robot_controllers::IOType::Orientation;
        else if (s == "Velocity")
            input_type = input_type | robot_controllers::IOType::Velocity;
        else if (s == "AngularVelocity")
            input_type = input_type | robot_controllers::IOType::AngularVelocity;
        else if (s == "Acceleration")
            input_type = input_type | robot_controllers::IOType::Acceleration;
        else if (s == "AngularAcceleration")
            input_type = input_type | robot_controllers::IOType::AngularAcceleration;
        else if (s == "Force")
            input_type = input_type | robot_controllers::IOType::Force;
        else if (s == "Torque")
            input_type = input_type | robot_controllers::IOType::Torque;
    }

    for (auto& s : output)
    {
        if (s == "Position")
            output_type = output_type | robot_controllers::IOType::Position;
        else if (s == "Orientation")
            output_type = output_type | robot_controllers::IOType::Orientation;
        else if (s == "Velocity")
            output_type = output_type | robot_controllers::IOType::Velocity;
        else if (s == "AngularVelocity")
            output_type = output_type | robot_controllers::IOType::AngularVelocity;
        else if (s == "Acceleration")
            output_type = output_type | robot_controllers::IOType::Acceleration;
        else if (s == "AngularAcceleration")
            output_type = output_type | robot_controllers::IOType::AngularAcceleration;
        else if (s == "Force")
            output_type = output_type | robot_controllers::IOType::Force;
        else if (s == "Torque")
            output_type = output_type | robot_controllers::IOType::Torque;
    }

    ctrl->SetIOTypes(input_type, output_type);
}

void set_input_space(CustomEffortController::ControllerPtr& ctrl, size_t space_dim)
{
    size_t input_dim = 0;

    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Position)
    {
        input_dim += space_dim;
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Orientation)
    {
        input_dim += 3;  // This is fixed to 3D
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Velocity)
    {
        input_dim += space_dim;
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::AngularVelocity)
    {
        input_dim += 3;  // This is fixed to 3D
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Acceleration)
    {
        input_dim += space_dim;
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::AngularAcceleration)
    {
        input_dim += 3;  // This is fixed to 3D
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Force)
    {
        input_dim += space_dim;
    }
    if (ctrl->GetInput().GetType() & robot_controllers::IOType::Torque)
    {
        input_dim += 3;  // This is fixed to 3D
    }

    robot_controllers::RobotParams params = ctrl->GetParams();
    params.input_dim_ = input_dim;
    // TO-DO: We assume same input/output
    params.output_dim_ = input_dim;

    ctrl->SetParams(params);
}

CustomEffortController::CustomEffortController() {}

CustomEffortController::~CustomEffortController() { sub_command_.shutdown(); }

bool CustomEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
{
    // List of controlled joints
    std::string ns = ros::names::parentNamespace(n.getNamespace()) + "/";
    std::string param_name = "joints";
    if (!n.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
        return false;
    }
    n_joints_ = joint_names_.size();

    if (n_joints_ == 0)
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam(ns + "robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    // Get basic parameters
    // Check the operational space
    n.param<std::string>("params/space", operation_space_,
                         "joint");  // Default operation space is task-space
    if (operation_space_ == "task")
    {
        space_dim_ = 3;

        // Get the URDF XML from the parameter server
        std::string urdf_string, full_param;
        std::string robot_description = ns + "robot_description";
        std::string end_effector;

        // gets the location of the robot description on the parameter server
        if (!n.searchParam(robot_description, full_param))
        {
            ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
            return false;
        }

        // search and wait for robot_description on param server
        while (urdf_string.empty())
        {
            ROS_INFO_ONCE_NAMED("CustomEffortController",
                                "CustomEffortController is waiting for model"
                                " URDF in parameter [%s] on the ROS param server.",
                                robot_description.c_str());

            n.getParam(full_param, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("CustomEffortController", "Received urdf from param server, parsing...");

        // Get the end-effector
        std::cerr << ns.substr(1, ns.length() - 2) << std::endl;
        n.param<std::string>("params/end_effector", end_effector, "link_ee");

        // Initialize iiwa tools
        tools_.init_rbdyn(urdf_string, end_effector);
    }
    else
        space_dim_ = n_joints_;

    // Read Controllers from Params
    std::map<std::string, ControllerPtr> controllers;
    std::vector<std::string> ctrl_names;

    XmlRpc::XmlRpcValue symbols;

    n.getParam("controllers", symbols);

    assert(symbols.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::iterator i = symbols.begin(); i != symbols.end(); ++i)
    {
        std::string name = i->first;
        ROS_INFO_STREAM("[CustomEffortController]: Trying to load controller '" << name << "'.");
        // ROS_WARN_STREAM(i->first << ": " << i->second.getType());
        std::string type, input, output;
        std::vector<double> param_values;
        n.getParam("controllers/" + name + "/type", type);
        n.getParam("controllers/" + name + "/params", param_values);
        n.getParam("controllers/" + name + "/input", input);
        n.getParam("controllers/" + name + "/output", output);

        if (type.size() == 0)
        {
            ROS_WARN_STREAM("Could not find type of controller '" << name << "'. Skipping this controller!");
            continue;
        }

        auto ctrl = manager_.loadAndInstantiate(type);

        if (ctrl)
        {
            robot_controllers::RobotParams params;
            params.input_dim_ = space_dim_;
            params.output_dim_ = space_dim_;

            params.time_step_ = 0.01;  // TO-DO: Get this from controller manager or yaml

            params.values_ = param_values;

            ctrl->SetParams(params);

            // TO-DO: Maybe separate input and output
            if (input.size() > 0 && output.size() > 0)
            {
                std::vector<std::vector<std::string>> tt = get_types(input, output);
                if (tt.size() == 2) set_types(ctrl, tt[0], tt[1]);
            }

            set_input_space(ctrl, space_dim_);

            controllers[name] = std::move(ctrl);
            ctrl_names.push_back(name);
        }
    }

    XmlRpc::XmlRpcValue symbols_structure;
    n.getParam("structure", symbols_structure);

    if (symbols_structure.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        for (XmlRpc::XmlRpcValue::iterator i = symbols_structure.begin(); i != symbols_structure.end(); ++i)
        {
            // ROS_WARN_STREAM(i->first << ": " << i->second.getType());
            std::string name = i->first;
            std::vector<std::string> sub;
            n.getParam("structure/" + name, sub);
            bool is_sum = false;
            if (name.find("Add") == 0)
            {
                controllers[name] = ControllerPtr(new WeightedSumController);
                is_sum = true;
            }
            else if (name.find("Cascade") == 0)
            {
                controllers[name] = ControllerPtr(new robot_controllers::CascadeController);
            }
            else
            {
                ROS_WARN_STREAM("Cannot identify the type of the controller by the name: '" << name << "'. Ignoring!");
                continue;
            }
            // std::cout << sub.size() << std::endl;
            for (size_t k = 0; k < sub.size(); k++)
            {
                // ROS_WARN_STREAM("    " << sub[k]);
                if (is_sum)
                    static_cast<WeightedSumController*>(controllers[name].get())
                        ->AddController(std::move(controllers[sub[k]]));
                else
                    static_cast<robot_controllers::CascadeController*>(controllers[name].get())
                        ->AddController(std::move(controllers[sub[k]]));
                ctrl_names.erase(std::remove(ctrl_names.begin(), ctrl_names.end(), sub[k]), ctrl_names.end());
                controllers.erase(sub[k]);
            }

            // Initialize parameters
            robot_controllers::RobotParams params;
            params.input_dim_ = space_dim_;
            params.output_dim_ = space_dim_;

            params.time_step_ = 0.01;  // TO-DO: Get this from controller manager or yaml
            controllers[name]->SetParams(params);

            set_input_space(controllers[name], space_dim_);

            ctrl_names.push_back(name);
        }
    }

    null_space_control_ = false;
    if (operation_space_ == "task")
    {
        std::vector<double> joints;
        n.getParam("params/null_space/joints", joints);
        null_space_control_ = (joints.size() == n_joints_);

        if (null_space_control_)
        {
            null_space_joint_config_ = Eigen::VectorXd::Map(joints.data(), joints.size());

            null_space_Kp_ = 20.;
            null_space_Kd_ = 0.1;
            null_space_max_torque_ = 10.;

            n.getParam("params/null_space/Kp", null_space_Kp_);
            n.getParam("params/null_space/Kp", null_space_Kd_);
            n.getParam("params/null_space/max_torque", null_space_max_torque_);
        }
    }

    unsigned int ctrl_size = ctrl_names.size();

    if (ctrl_size == 0)
    {
        ROS_ERROR_STREAM("Could not load specified controllers! Exiting..!");
        return false;
    }

    if (ctrl_size == 1)
    {
        controller_ = std::move(controllers[ctrl_names[0]]);

        set_input_space(controller_, space_dim_);
    }
    else
    {
        controller_.reset(new WeightedSumController);
        for (unsigned int i = 0; i < ctrl_size; i++)
        {
            static_cast<WeightedSumController*>(controller_.get())
                ->AddController(std::move(controllers[ctrl_names[i]]));
            ctrl_names_.emplace_back(ctrl_names[i]);
        }

        set_input_space(controller_, space_dim_);
    }

    // Update the weights of the controllers
    server_update_weight_ = n.advertiseService("update_weight", &CustomEffortController::updateWeight, this);
    ROS_INFO_STREAM("[CustomEffortController]: Started Update weight server..");

    // Initialize the controller(s)
    if (!controller_->Init())
    {
        ROS_ERROR("Controllers could not be initialized! Exiting!");
        return false;
    }

    // Info on the controller(s)
    ROS_INFO("Controller output:\n\t-has force: %d\n\t-has torque: %d",
             static_cast<bool>(controller_->GetOutput().GetType() & robot_controllers::IOType::Force),
             static_cast<bool>(controller_->GetOutput().GetType() & robot_controllers::IOType::Force));

    for (unsigned int i = 0; i < n_joints_; i++)
    {
        try
        {
            joints_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }
        joint_urdfs_.push_back(joint_urdf);
    }

    // Get controller command size
    cmd_dim_ = 0;

    if (controller_->GetInput().GetType() & robot_controllers::IOType::Position)
    {
        cmd_dim_ += space_dim_;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Orientation)
    {
        cmd_dim_ += 3;  // This is fixed to 3D
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Velocity)
    {
        cmd_dim_ += space_dim_;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::AngularVelocity)
    {
        cmd_dim_ += 3;  // This is fixed to 3D
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Acceleration)
    {
        cmd_dim_ += space_dim_;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::AngularAcceleration)
    {
        cmd_dim_ += 3;  // This is fixed to 3D
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Force)
    {
        cmd_dim_ += space_dim_;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Torque)
    {
        cmd_dim_ += 3;  // This is fixed to 3D
    }

    std::vector<double> init_cmd(cmd_dim_, 0.0);
    has_orientation_ = false;
    if (operation_space_ == "task")
    {
        has_orientation_ =
            ((controller_->GetInput().GetType() & robot_controllers::IOType::Orientation)) ? true : false;
        bool has_position = ((controller_->GetInput().GetType() & robot_controllers::IOType::Position)) ? true : false;
        if (has_position || has_orientation_)
        {
            // if task space, we need to alter the initial command
            iiwa_tools::RobotState robot_state;
            robot_state.position.resize(n_joints_);
            robot_state.velocity.resize(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                robot_state.position[i] = joints_[i].getPosition();
                robot_state.velocity[i] = joints_[i].getVelocity();
            }

            auto ee_state = tools_.perform_fk(robot_state);
            Eigen::AngleAxisd aa(ee_state.orientation);
            Eigen::VectorXd o = aa.axis() * aa.angle();
            Eigen::VectorXd p = ee_state.translation;

            size_t offset = 0;
            if (has_orientation_) offset = 3;

            for (size_t i = 0; i < 3; i++)
            {
                if (has_orientation_) init_cmd[i] = o(i);
                if (has_position) init_cmd[i + offset] = p(i);
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

    // Retrieve the current robot state

    robot_controllers::RobotState curr_robot_state, current_cont_state;
    curr_robot_state.position_ = Eigen::VectorXd::Zero(n_joints_);
    curr_robot_state.velocity_ = Eigen::VectorXd::Zero(n_joints_);
    curr_robot_state.acceleration_ = Eigen::VectorXd::Zero(n_joints_);
    curr_robot_state.force_ = Eigen::VectorXd::Zero(n_joints_);

    for (unsigned int i = 0; i < n_joints_; i++)
    {
        curr_robot_state.position_(i) = joints_[i].getPosition();
        curr_robot_state.velocity_(i) = joints_[i].getVelocity();
        // curr_robot_state.acceleration_(i) = joints_[i].getAcceleration();
        // TO-DO: Fill acceleration
        curr_robot_state.force_(i) = joints_[i].getEffort();
    }
    current_cont_state = curr_robot_state;  // default, control the joints

    // Task space, controls the end effector

    // Jacobians, used for output
    Eigen::MatrixXd jac(6, n_joints_);
    Eigen::MatrixXd jac_deriv(6, n_joints_);
    Eigen::MatrixXd jac_t_pinv(n_joints_, 6);
    if (operation_space_ == "task")
    {
        // Compute the current end effector state (joints)
        iiwa_tools::RobotState robot_state_tool(
            {curr_robot_state.position_, curr_robot_state.velocity_, curr_robot_state.torque_});
        // Forwards to compute EEF pose
        auto ee_state = tools_.perform_fk(robot_state_tool);
        // Jacobians
        // TODO(William) Jacobians computed here and in iiwa_tools/EefPublisher, not efficient. Regroup and share.
        std::tie(jac, jac_deriv) = tools_.jacobians(robot_state_tool);
        jac_t_pinv = pseudo_inverse(Eigen::MatrixXd(jac.transpose()));
        // Compute the twist and acceleration vectors using jacobian
        Eigen::Vector6d twist = jac * current_cont_state.velocity_,
                        accel = jac * current_cont_state.acceleration_ + jac_deriv * current_cont_state.velocity_;
        // Compute the end effector wrench using the inverse jacobian
        Eigen::VectorXd wrench = jac_t_pinv * current_cont_state.force_;

        // Control based on the current end effector state
        current_cont_state.position_ = ee_state.translation;
        current_cont_state.velocity_ = twist.tail(3);
        current_cont_state.acceleration_ = accel.tail(3);
        current_cont_state.force_ = wrench.tail(3);
        if (has_orientation_)
        {
            // Orientation target
            Eigen::AngleAxisd aa(ee_state.orientation);
            current_cont_state.orientation_ = aa.axis() * aa.angle();
            current_cont_state.angular_velocity_ = twist.head(3);
            current_cont_state.angular_acceleration_ = accel.head(3);
            current_cont_state.torque_ = wrench.head(3);
        }
    }

    // Decode current command to get the desired state
    Eigen::VectorXd cmd(n_joints_);
    robot_controllers::RobotState desired_state;
    unsigned int size = current_cont_state.position_.size();
    unsigned int index = 0;

    // Command data depends on controller type
    cmd = Eigen::VectorXd::Map(commands.data(), commands.size());
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Orientation)
    {
        desired_state.orientation_ = cmd.segment(index, 3);
        index += 3;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Position)
    {
        desired_state.position_ = cmd.segment(index, size);
        index += size;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::AngularVelocity)
    {
        desired_state.angular_velocity_ = cmd.segment(index, 3);
        index += 3;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Velocity)
    {
        desired_state.velocity_ = cmd.segment(index, size);
        index += size;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::AngularAcceleration)
    {
        desired_state.angular_acceleration_ = cmd.segment(index, 3);
        index += 3;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Acceleration)
    {
        desired_state.acceleration_ = cmd.segment(index, size);
        index += size;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Torque)
    {
        desired_state.torque_ = cmd.segment(index, 3);
        index += 3;
    }
    if (controller_->GetInput().GetType() & robot_controllers::IOType::Force)
    {
        desired_state.force_ = cmd.segment(index, size);
        // index += size;
    }

    // Update control torques given current and desired states
    controller_->SetInput(desired_state);
    controller_->Update(current_cont_state);

    // Retrieve and process the output from the controller
    Eigen::VectorXd output;  // = Eigen::VectorXd::Zero(space_dim_);
    if (operation_space_ == "task")
    {
        output = Eigen::VectorXd::Zero(2 * space_dim_);
        // Enable control for the EEF force
        if (controller_->GetOutput().GetType() & robot_controllers::IOType::Force)
            output.tail(3) = controller_->GetOutput().desired_.force_;
        // Enable control for the EEF torque
        if (controller_->GetOutput().GetType() & robot_controllers::IOType::Torque)
            output.head(3) = controller_->GetOutput().desired_.torque_;

        // Compute joints' effort, based on jacobian
        output = jac.transpose() * output;

        // Superpose null-space control
        if (null_space_control_)
        {
            Eigen::VectorXd null_space_signal =
                null_space_Kp_ * (null_space_joint_config_ - curr_robot_state.position_) -
                null_space_Kd_ * curr_robot_state.velocity_;
            Eigen::VectorXd null_space_force =
                (Eigen::MatrixXd::Identity(n_joints_, n_joints_) - jac.transpose() * jac_t_pinv) * null_space_signal;
            for (int i = 0; i < null_space_force.size(); i++)
            {
                if (null_space_force(i) > null_space_max_torque_)
                    null_space_force(i) = null_space_max_torque_;
                else if (null_space_force(i) < -null_space_max_torque_)
                    null_space_force(i) = -null_space_max_torque_;
            }
            output = output + null_space_force;
        }
    }
    else
    {  // regular controller, directly control joints' effort
        output = controller_->GetOutput().desired_.force_;
    }

    // Set the commanded effort to the robot
    for (unsigned int i = 0; i < n_joints_; i++)
    {
        double command = output[i];
        enforceJointLimits(command, i);
        joints_[i].setCommand(command);
    }
}

bool CustomEffortController::updateWeight(iiwa_tools::UpdateWeight::Request& request,
                                          iiwa_tools::UpdateWeight::Response& response)
{
    ROS_INFO_STREAM("[CustomEffortController]: Trying to update weight of controller '"
                    << request.controller_name.data << "' with value " << request.weight.data);
    WeightedSumController* weightedSumController = dynamic_cast<WeightedSumController*>(&*controller_);
    // Update weights if controller is of type weighted sum
    if (weightedSumController)
    {
        // Find controller index in sum controller
        auto it = find(ctrl_names_.begin(), ctrl_names_.end(), request.controller_name.data);
        if (it != ctrl_names_.end())
        {
            // Set weight
            weightedSumController->SetWeight(request.weight.data, it - ctrl_names_.begin());
            ROS_INFO("[CustomEffortController]: Controller weight updated.");
            return true;
        }
        else
            ROS_WARN_STREAM("[CustomEffortController]: Could not find controller with name "
                            << request.controller_name.data << ".");
    }
    else
        ROS_WARN(
            "[CustomEffortController]: Attempted to set weight but controller "
            "is not of type WeightedSumController");
    return false;
}

void CustomEffortController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if (msg->data.size() != cmd_dim_)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") is not correct! Not executing!");
        return;
    }

    commands_buffer_.writeFromNonRT(msg->data);
}

void CustomEffortController::enforceJointLimits(double& command, unsigned int index)
{
    // Check that this joint has applicable limits
    if (command > joint_urdfs_[index]->limits->effort)  // above upper limit
    {
        command = joint_urdfs_[index]->limits->effort;
    }
    else if (command < -joint_urdfs_[index]->limits->effort)  // below lower limit
    {
        command = -joint_urdfs_[index]->limits->effort;
    }
}
}  // namespace iiwa_control

PLUGINLIB_EXPORT_CLASS(iiwa_control::CustomEffortController, controller_interface::ControllerBase)
