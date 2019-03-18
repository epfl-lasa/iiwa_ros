#include <pluginlib/class_list_macros.hpp>
#include <iiwa_control/custom_effort_controller.hpp>

#include <Eigen/Core>

#include <iiwa_tools/iiwa_tools.h>

namespace iiwa_control
{
    double get_multi_array(const std_msgs::Float64MultiArray& array, size_t i, size_t j)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        return array.data[offset + i * array.layout.dim[0].stride + j];
    }

    CustomEffortController::CustomEffortController() {}

    CustomEffortController::~CustomEffortController() { sub_command_.shutdown(); }

    bool CustomEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        // List of controlled joints
        std::string param_name = "joints";
        if(!n.getParam(param_name, joint_names_))
        {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

        if(n_joints_ == 0){
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }

        // Get URDF
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }

        // Get controller's parameters
        std::vector<double> eigvals_vec;

        n.param<std::string>("params/space", operation_space_, "joint"); // Default operation space is task-space
        n.param<std::string>("params/gravity", gravity_comp_, "off"); // We do not use gravity compensation by default
        n.getParam("params/eigvals", eigvals_vec);

        // Check the operational space
        if (operation_space_ == "task"){
            space_dim_ = 6;
            iiwa_client_jacobian_ = n.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");
        }
        else
            space_dim_ = n_joints_;

        // Init Controller
        passive_ds_.SetParams(space_dim_, eigvals_vec);

        for(unsigned int i=0; i<n_joints_; i++)
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

        // Setup services
        jacobian_srv_.request.joint_angles.resize(n_joints_, 0.);
        jacobian_srv_.request.joint_velocities.resize(n_joints_, 0.);

        commands_buffer_.writeFromNonRT(std::vector<double>(space_dim_, 0.0));

        sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &CustomEffortController::commandCB, this);

        return true;
    }

    void CustomEffortController::update(const ros::Time& time, const ros::Duration& period)
    {
        std::vector<double> & commands = *commands_buffer_.readFromRT();
        
        Eigen::MatrixXd jac(6, n_joints_);
        bool jac_valid = false;

        if (operation_space_ == "task") {
            // Call iiwa tools service for jacobian
            for (size_t i = 0; i < n_joints_; i++) {
                jacobian_srv_.request.joint_angles[i] = joints_[i].getPosition();
                jacobian_srv_.request.joint_velocities[i] = joints_[i].getVelocity();
            }

            if (iiwa_client_jacobian_.call(jacobian_srv_)) {
                assert(jacobian_srv_.response.jacobian.layout.dim.size() == 2); // we need a 2D array
                assert(jacobian_srv_.response.jacobian.layout.dim[0].size == 6); // check if Jacobian has proper dimensions
                assert(jacobian_srv_.response.jacobian.layout.dim[1].size == n_joints_);

                for(size_t r = 0; r < 6; r++) {
                    for (size_t c = 0; c < n_joints_; c++) {
                        jac(r, c) = get_multi_array(jacobian_srv_.response.jacobian, r, c);
                    }
                }

                jac_valid = true;
            }
            else {
                ROS_ERROR_STREAM("Could not get Jacobian!");
            }
        }

        Eigen::VectorXd desired_vel(n_joints_), curr_vel(n_joints_);

        desired_vel = Eigen::VectorXd::Map(commands.data(), commands.size());
        for(unsigned int i=0; i<n_joints_; i++)
        {
            curr_vel(i) = joints_[i].getVelocity();
        }

        if (operation_space_ == "task" && jac_valid) {
            curr_vel = jac * curr_vel;
        }

        passive_ds_.SetInput(curr_vel, desired_vel);

        Eigen::VectorXd output = passive_ds_.GetOutput().effort_;
        // Eigen::VectorXd output = 10. * (desired_vel - curr_vel).transpose().array();

        if (operation_space_ == "task" && jac_valid) {
            // output.head(3) = Eigen::VectorXd::Zero(3);
            output = jac.transpose() * output;
        }

        // ROS_INFO_STREAM("Effort: " << output.transpose());

        std::vector<double> commanded_effort(n_joints_, 0.);

        Eigen::VectorXd::Map(commanded_effort.data(), commanded_effort.size()) = output;

        for(unsigned int i=0; i<n_joints_; i++) {
            enforceJointLimits(commanded_effort[i], i);
            joints_[i].setCommand(commanded_effort[i]);
        }
    }

    void CustomEffortController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        if(msg->data.size() != n_joints_ && msg->data.size() != 6)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") is not correct! Not executing!");
            return;
        }

        commands_buffer_.writeFromNonRT(msg->data);
    }

    void CustomEffortController::enforceJointLimits(double &command, unsigned int index)
    {
        // Check that this joint has applicable limits
        if(command > joint_urdfs_[index]->limits->effort) // above upper limit
        {
            command = joint_urdfs_[index]->limits->effort;
        }
        else if(command < -joint_urdfs_[index]->limits->effort) // below lower limit
        {
            command = -joint_urdfs_[index]->limits->effort;
        }
    }
} // namespace

PLUGINLIB_EXPORT_CLASS(iiwa_control::CustomEffortController, controller_interface::ControllerBase)
