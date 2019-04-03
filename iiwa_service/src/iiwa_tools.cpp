#include <iiwa_service/iiwa_tools.h>
#include <vector>

namespace iiwa_service {
    IiwaTools::IiwaTools(/* args */)
    {

    }

    IiwaTools::~IiwaTools()
    {
        
    }

    void IiwaTools::InitRBDyn(const std::string& urdf_string)
    {
        // Convert URDF to RBDyn
        _rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);
        // _fd = rbd::InverseKinematics(_rbdyn_urdf.mb);

        _rbd_indices.clear();

        for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++) {
            if (_rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
                _rbd_indices.push_back(i);
        }

        _ef_index = _rbd_index(_end_effector);

        _ik.reset(new rbd::InverseKinematics(_rbdyn_urdf.mb, _ef_index));
    }

    size_t IiwaTools::_rbd_index(const std::string& body_name) const
    {
        for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++) {
            if (_rbdyn_urdf.mb.body(i).name() == body_name) {
                return i;
            }
        }

        // TO-DO: Should never reach here
        return 0;
    }

    void IiwaTools::gravity(const std::vector<double>& gravity, const RobotState& robot_state)
    {
        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        rbdyn_urdf.mbc.gravity = {gravity[0], gravity[1], gravity[2]};

        update_urdf_state(const RobotState& robot_state);

        // Forward Dynamics
        rbd::ForwardDynamics fd(rbdyn_urdf.mb);

        // Compute gravity compensation
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        fd.computeC(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        // Get gravity and Coriolis forces
        C_ = -fd.C();
    }

    void IiwaTools::jacobian(const RobotState& robot_state)
    {
         // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

        update_urdf_state(const RobotState& robot_state);

        // Compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        // TO-DO: Check if we need this
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        J_ = jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);
    }

    void IiwaTools::update_urdf_state(const RobotState& robot_state)
    {
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i];

            rbdyn_urdf.mbc.q[rbd_index][0] = robot_state.position[i];
            rbdyn_urdf.mbc.alpha[rbd_index][0] = robot_state.velocity[i];
            rbdyn_urdf.mbc.jointTorque[rbd_index][0] = robot_state.torque[i];
        }
    }

} // namespace iiwa_service