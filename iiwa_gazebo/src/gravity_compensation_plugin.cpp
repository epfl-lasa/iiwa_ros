#include <iiwa_gazebo/gravity_compensation_plugin.h>

// URDF headers
#include <urdf/model.h>

// RBDyn headers
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace gazebo {

    GravityCompensationPlugin::GravityCompensationPlugin() {}

    GravityCompensationPlugin::~GravityCompensationPlugin()
    {
        this->_update_connection.reset();
    }

    void GravityCompensationPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        _model = parent;

        // Error message if the model couldn't be found
        if (!_model) {
            ROS_ERROR("Parent model is NULL! GravityCompensationPlugin could not be loaded.");
            return;
        }

        _world = _model->GetWorld();

        // Check that ROS has been initialized
        if (!ros::isInitialized()) {
            ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
            return;
        }

        // Check for robot namespace
        _robot_namespace = "";
        if (sdf->HasElement("robotNamespace")) {
            _robot_namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
        }

        _model_nh = ros::NodeHandle(_robot_namespace);

        // Get robot_description ROS param name
        if (sdf->HasElement("robotParam")) {
            _robot_description = sdf->GetElement("robotParam")->Get<std::string>();
        }
        else {
            _robot_description = "robot_description"; // default
        }

        // Read urdf from ros parameter server
        // This call will block if ROS is not properly initialized.
        const std::string urdf_string = _get_urdf(_robot_description);

        // Convert URDF to RBDyn
        _rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);
        math::Vector3d gravity = _world->Gravity();
        _rbdyn_urdf.mbc.gravity = {gravity[0], gravity[1], gravity[2]};
        _fd = rbd::ForwardDynamics(_rbdyn_urdf.mb);
        // _coriolis.reset(new rbd::Coriolis(_rbdyn_urdf.mb));

        // Link Gazebo and URDF information
        urdf::Model urdf_model;
        const urdf::Model* const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

        if (!urdf_model_ptr) {
            ROS_ERROR("A URDF::Model could not be initialized, unable to load plugin.");
            return;
        }

        size_t i = 0;
        _num_joints = 0;
        _rbd_indices.clear();
        _joints.clear();

        for (std::map<std::string, urdf::JointSharedPtr>::const_iterator it = urdf_model_ptr->joints_.begin(); it != urdf_model_ptr->joints_.end(); it++) {
            gazebo::physics::JointPtr joint = _model->GetJoint(it->second->name);
            bool fixed = it->second->type == urdf::Joint::FIXED;
            if (!joint || fixed) {
                if (!fixed) {
                    ROS_ERROR_STREAM("This robot has a joint named \"" << it->second->name << "\" which is not in the gazebo model.");
                    return;
                }
                continue;
            }

            _joints.push_back(joint);
            _num_joints++;

            _rbd_indices.push_back(_rbd_index(it->second->name));
        }

        _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);

        if (_num_joints != _rbd_indices.size()) {
            ROS_ERROR("URDF joints did not successfully link to Gazebo joints, unable to load plugin.");
            return;
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->_update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GravityCompensationPlugin::update_child, this));

        // Output some confirmation
        ROS_INFO_STREAM("GravityCompensationPlugin loaded!");
    }

    void GravityCompensationPlugin::update_child()
    {
        Eigen::VectorXd dq = _update_robot();

        // Compute Forward Dynamics
        rbd::forwardKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
        rbd::forwardVelocity(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
        _fd.computeC(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
        // _fd.forwardDynamics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

        // Get gravity and Coriolis forces
        Eigen::VectorXd C = _fd.C();

        for (size_t j = 0; j < _num_joints; j++) {
            _joints[j]->SetForce(0, -C(j));
        }
    }

    // Get the URDF XML from the parameter server
    std::string GravityCompensationPlugin::_get_urdf(const std::string& param_name) const
    {
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            std::string search_param_name;
            if (_model_nh.searchParam(param_name, search_param_name)) {
                ROS_INFO_ONCE_NAMED("iiwa_gazebo", "iiwa_gazebo_gravity_compensation plugin is waiting for model"
                                                   " URDF in parameter [%s] on the ROS param server.",
                    search_param_name.c_str());

                _model_nh.getParam(search_param_name, urdf_string);
            }
            else {
                ROS_INFO_ONCE_NAMED("iiwa_gazebo", "iiwa_gazebo_gravity_compensation plugin is waiting for model"
                                                   " URDF in parameter [%s] on the ROS param server.",
                    _robot_description.c_str());

                _model_nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }
        ROS_DEBUG_STREAM_NAMED("iiwa_gazebo", "Recieved urdf from param server, parsing...");

        return urdf_string;
    }

    Eigen::VectorXd GravityCompensationPlugin::_update_robot()
    {
        _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);

        Eigen::VectorXd dq = Eigen::VectorXd::Zero(_num_joints);

#if GAZEBO_MAJOR_VERSION >= 8
        double dt = _world->Physics()->GetMaxStepSize();
#else
        double dt = _world->GetPhysicsEngine()->GetMaxStepSize();
#endif

        for (size_t j = 0; j < _num_joints; j++) {
            size_t rbd_index = _rbd_indices[j];
            // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
            double position = _joints[j]->Position(0);
#else
            double position = _joints[j]->GetAngle(0).Radian();
#endif

            // ROS_INFO_STREAM("Got position " << position << " for joint " << _joints[j]->GetName());
            double velocity = _joints[j]->GetVelocity(0);
            // ROS_INFO_STREAM("Got velocity " << velocity << " for joint " << _joints[j]->GetName());
            // // ROS_INFO_STREAM(_rbdyn_urdf.mbc.q[rbd_index].size());
            // double acceleration = (velocity - _rbdyn_urdf.mbc.alpha[rbd_index][0]) / dt; // TO-DO: Check some filter
            // ROS_INFO_STREAM("Got acceleration " << velocity << " for joint " << _joints[j]->GetName());

            _rbdyn_urdf.mbc.q[rbd_index][0] = position;
            _rbdyn_urdf.mbc.alpha[rbd_index][0] = velocity;
            // _rbdyn_urdf.mbc.alphaD[rbd_index][0] = acceleration;
            _rbdyn_urdf.mbc.jointTorque[rbd_index][0] = _joints[j]->GetForce(0);

            dq[j] = velocity;
        }

        return dq;
    }

    size_t GravityCompensationPlugin::_rbd_index(const std::string& joint_name) const
    {
        // if (_rbdyn_urdf.mb.joint(i++).type() != rbd::Joint::Fixed)
        for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++) {
            if (_rbdyn_urdf.mb.joint(i).name() == joint_name) {
                return i;
            }
        }

        // TO-DO: Should never reach here
        return 0;
    }

    GZ_REGISTER_MODEL_PLUGIN(GravityCompensationPlugin);
} // namespace gazebo