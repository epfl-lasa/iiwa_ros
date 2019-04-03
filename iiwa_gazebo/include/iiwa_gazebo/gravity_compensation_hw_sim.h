#include <gazebo_ros_control/default_robot_hw_sim.h>

// Iiwa service headers
#include <iiwa_service/GetGravity.h>

// std headers
#include <vector>

namespace iiwa_gazebo {
    class GravityCompensationHWSim : public gazebo_ros_control::DefaultRobotHWSim {
    public:
        virtual bool initSim(
            const std::string& robot_namespace,
            ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model,
            const urdf::Model* const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions) override;

        virtual void readSim(ros::Time time, ros::Duration period) override;
        virtual void writeSim(ros::Time time, ros::Duration period) override;

    protected:
        // ROS related
        ros::ServiceClient _iiwa_gravity_client;
        iiwa_service::GetGravity _gravity_srv;
    };

} // namespace iiwa_gazebo