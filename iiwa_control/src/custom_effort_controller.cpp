#include <pluginlib/class_list_macros.hpp>
#include <iiwa_control/custom_effort_controller.hpp>

namespace iiwa_control
{
    CustomEffortController::CustomEffortController() {}

    CustomEffortController::~CustomEffortController() {sub_command_.shutdown();}

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

        ros::NodeHandle n_p("~/params");
        n_p.param<std::string>("space", operation_space_, "task"); // Default operation space is task-space
        n_p.param<std::string>("gravity", gravity_comp_, "off"); // We do not use gravity compensation by default
        n_p.getParam("eigvals", eigvals_vec);

        // Check the operational space
        if (operation_space_.compare("task")){
            space_dim_ = 7;
            iiwa_client_jacobian_ = n.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");
        }
        else
            space_dim_ = n_joints_;

        // Check if gravity compensation is requested
        if (operation_space_.compare("task"))
            iiwa_client_gravity_ = n.serviceClient<iiwa_tools::GetGravity>("/iiwa/iiwa_gravity_server");

        // Init Controller
        passive_ds_.SetParams(n_joints_, eigvals_vec);

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

        commands_buffer_.writeFromNonRT(std::vector<double>(space_dim_, 0.0));

        sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &CustomEffortController::commandCB, this);

        return true;
    }

    void CustomEffortController::update(const ros::Time& time, const ros::Duration& period)
    {
        std::vector<double> & commands = *commands_buffer_.readFromRT();

        std::vector<double> desired_velocity,
                            current_velocity;

        for(unsigned int i=0; i<n_joints_; i++)
        {
            desired_velocity.push_back(commands[i]);
            current_velocity.push_back(joints_[i].getVelocity());
        }

        passive_ds_.SetInput(Eigen::Map<Eigen::VectorXd>(current_velocity.data(),current_velocity.size()),
                             Eigen::Map<Eigen::VectorXd>(desired_velocity.data(), desired_velocity.size()));

        auto output = passive_ds_.GetOutput();

        std::vector<double> commanded_effort(output.effort_.data(),output.effort_.data() + output.effort_.size());

        if (operation_space_.compare("task"))
            // Call the service for the Jacobian

        for(unsigned int i=0; i<n_joints_; i++)
            joints_[i].setCommand(commanded_effort[i]);
    }

    void CustomEffortController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        if(msg->data.size()!=n_joints_)
        {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
        }
        commands_buffer_.writeFromNonRT(msg->data);
    }

    void CustomEffortController::enforceJointLimits(double &command, unsigned int index)
    {
        // Check that this joint has applicable limits
        if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
        {
        if( command > joint_urdfs_[index]->limits->upper ) // above upper limit
        {
            command = joint_urdfs_[index]->limits->upper;
        }
        else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
        {
            command = joint_urdfs_[index]->limits->lower;
        }
        }
    }
} // namespace

PLUGINLIB_EXPORT_CLASS(iiwa_control::CustomEffortController, controller_interface::ControllerBase)
