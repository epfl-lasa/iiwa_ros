#include <gazebo_ros_control/default_robot_hw_sim.h>

// RBDyn headers
#include <RBDyn/FD.h>
#include <mc_rbdyn_urdf/urdf.h>

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
        size_t _rbd_index(const std::string& joint_name) const;

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        rbd::ForwardDynamics _fd;

        std::vector<size_t> _rbd_indices;
    };

} // namespace iiwa_gazebo