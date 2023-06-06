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
#include <iiwa_gazebo/gravity_compensation_hw_sim.h>

namespace {
    double clamp(const double val, const double min_val, const double max_val)
    {
        return std::min(std::max(val, min_val), max_val);
    }
} // namespace

namespace iiwa_gazebo {
    bool GravityCompensationHWSim::initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model* const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        // Initialize Gazebo related things
        if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions))
            return false;

        _iiwa_gravity_client = model_nh.serviceClient<iiwa_tools::GetGravity>("/iiwa/iiwa_gravity_server");

        // Initialize service message
        auto gravity = parent_model->GetWorld()->Gravity();
        _gravity_srv.request.joint_angles.resize(n_dof_);
        _gravity_srv.request.joint_velocities.resize(n_dof_);
        _gravity_srv.request.joint_torques.resize(n_dof_);

        _gravity_srv.request.gravity = {gravity[0], gravity[1], gravity[2]};

        return true;
    }

    void GravityCompensationHWSim::readSim(ros::Time time, ros::Duration period)
    {
        for (unsigned int j = 0; j < n_dof_; j++) {
            // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
            double position = sim_joints_[j]->Position(0);
#else
            double position = sim_joints_[j]->GetAngle(0).Radian();
#endif
            if (joint_types_[j] == urdf::Joint::PRISMATIC) {
                joint_position_[j] = position;
            }
            else {
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                    position);
            }
            joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
            joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));

            // Pass values to gravity compensation service
            _gravity_srv.request.joint_angles[j] = joint_position_[j];
            _gravity_srv.request.joint_velocities[j] = joint_velocity_[j];
            _gravity_srv.request.joint_torques[j] = joint_effort_[j];
        }
    }

    void GravityCompensationHWSim::writeSim(ros::Time time, ros::Duration period)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
        // Get gravity and Coriolis forces
        std::vector<double> C(n_dof_, 0.);
        // Call iiwa tools service for gravity compensation
        if (_iiwa_gravity_client.call(_gravity_srv)) {
            for (size_t i = 0; i < n_dof_; i++) {
                C[i] = _gravity_srv.response.compensation_torques[i];
            }
        }

        // If the E-stop is active, joints controlled by position commands will maintain their positions.
        if (e_stop_active_) {
            if (!last_e_stop_active_) {
                last_joint_position_command_ = joint_position_;
                last_e_stop_active_ = true;
            }
            joint_position_command_ = last_joint_position_command_;
        }
        else {
            last_e_stop_active_ = false;
        }

        ej_sat_interface_.enforceLimits(period);
        ej_limits_interface_.enforceLimits(period);
        pj_sat_interface_.enforceLimits(period);
        pj_limits_interface_.enforceLimits(period);
        vj_sat_interface_.enforceLimits(period);
        vj_limits_interface_.enforceLimits(period);

        for (unsigned int j = 0; j < n_dof_; j++) {
            switch (joint_control_methods_[j]) {
            case EFFORT: {
                const double effort_limit = joint_effort_limits_[j];
                const double effort = e_stop_active_ ? 0 : clamp(joint_effort_command_[j] + C[j], -effort_limit, effort_limit);
                sim_joints_[j]->SetForce(0, effort);
            } break;

            case POSITION:
#if GAZEBO_MAJOR_VERSION >= 9
                sim_joints_[j]->SetPosition(0, joint_position_command_[j], true);
#else
                ROS_WARN_ONCE("The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.");
                ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
                ROS_WARN_ONCE("Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.");
                ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
                sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#endif
                break;

            case POSITION_PID: {
                double error;
                switch (joint_types_[j]) {
                case urdf::Joint::REVOLUTE:
                    angles::shortest_angular_distance_with_limits(joint_position_[j],
                        joint_position_command_[j],
                        joint_lower_limits_[j],
                        joint_upper_limits_[j],
                        error);
                    break;
                case urdf::Joint::CONTINUOUS:
                    error = angles::shortest_angular_distance(joint_position_[j],
                        joint_position_command_[j]);
                    break;
                default:
                    error = joint_position_command_[j] - joint_position_[j];
                }

                const double effort_limit = joint_effort_limits_[j];
                const double effort = clamp(pid_controllers_[j].computeCommand(error, period) + C[j],
                    -effort_limit, effort_limit);
                sim_joints_[j]->SetForce(0, effort);
            } break;

            case VELOCITY:
#if GAZEBO_MAJOR_VERSION > 2
                if (physics->GetType().compare("ode") == 0) {
                    sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
                }
                else {
                    sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
                }
#else
                sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
#endif
                break;

            case VELOCITY_PID:
                double error;
                if (e_stop_active_)
                    error = -joint_velocity_[j];
                else
                    error = joint_velocity_command_[j] - joint_velocity_[j];
                const double effort_limit = joint_effort_limits_[j];
                const double effort = clamp(pid_controllers_[j].computeCommand(error, period) + C[j],
                    -effort_limit, effort_limit);
                sim_joints_[j]->SetForce(0, effort);
                break;
            }
        }
    }
} // namespace iiwa_gazebo

PLUGINLIB_EXPORT_CLASS(iiwa_gazebo::GravityCompensationHWSim, gazebo_ros_control::RobotHWSim)