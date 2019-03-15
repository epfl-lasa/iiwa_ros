#ifndef EFFORT_CONTROLLERS__EFFORT_CUSTOM_CONTROLLER_H
#define EFFORT_CONTROLLERS__EFFORT_CUSTOM_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

#include <control_stack/controllers/passive_ds.hpp>

namespace iiwa_control
{
    class CustomEffortController : public  controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        CustomEffortController();
        ~CustomEffortController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);

        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

        std::vector< std::string > joint_names_;

        std::vector< hardware_interface::JointHandle > joints_;

        realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;

        unsigned int n_joints_;

    private:
        ros::Subscriber sub_command_;
        
        unsigned int loop_count_;

        control_stack::controllers::PassiveDS passive_ds_;

        std::vector<urdf::JointConstSharedPtr> joint_urdfs_; // ?

        void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // ?

        void enforceJointLimits(double &command, unsigned int index);

    };  

} // namespace

#endif // EFFORT_CONTROLLERS__EFFORT_CUSTOM_CONTROLLER_H