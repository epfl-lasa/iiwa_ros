#ifndef IIWA_GAZEBO_GRAVITY_COMPENSATION_PLUGIN
#define IIWA_GAZEBO_GRAVITY_COMPENSATION_PLUGIN

// ROS headers
#include <ros/ros.h>

// Gazebo headers
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// RBDyn headers
#include <RBDyn/FD.h>
#include <mc_rbdyn_urdf/urdf.h>

// std headers
#include <vector>

namespace gazebo {
    class GravityCompensationPlugin : public ModelPlugin {
    public:
        GravityCompensationPlugin();
        ~GravityCompensationPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
        void update_child();

    private:
        std::string _get_urdf(const std::string& param_name) const;
        Eigen::VectorXd _update_robot();
        size_t _rbd_index(const std::string& joint_name) const;

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        rbd::ForwardDynamics _fd;

        // ROS related
        ros::NodeHandle _model_nh;

        // Parameters/helpers
        std::string _robot_namespace, _robot_description;
        size_t _num_joints;
        std::vector<size_t> _rbd_indices;

        // Pointers to the joints
        std::vector<physics::JointPtr> _joints;

        // Pointer to the model
        physics::ModelPtr _model;

        // Pointer to the world
        physics::WorldPtr _world;

        // Pointer to the update event connection
        event::ConnectionPtr _update_connection;
    };
} // namespace gazebo

#endif