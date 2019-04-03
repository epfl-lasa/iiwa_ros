#ifndef IIWA_SERVICE_IIWA_TOOLS_H
#define IIWA_SERVICE_IIWA_TOOLS_H

namespace iiwa_service {
    struct RobotState
    {
        std::vector<double> position,
                            velocity,
                            torque;
    }
    class IiwaTools
    {
    public:
        IiwaTools(/* args */);
        ~IiwaTools();
        
        void InitRBDyn(const std::string& urdf_string);

        std::vector<size_t> get_indices() {return _rbd_indices}

        Eigen::VectorXd get_C_matrix() {return C_}

        Eigen::MatrixXd get_J_matrix() {return J_}

        void perform_fk();
        void perform_ik();
        void jacobian(const RobotState& robot_state);
        void gravity(const std::vector<double>& gravity, const RobotState& robot_state);

    protected:
        size_t _rbd_index(const std::string& body_name) const
        void update_urdf_state(const RobotState& robot_state)

        // RBDyn related
        mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
        std::unique_ptr<rbd::InverseKinematics> _ik;
        std::vector<double> _gravity;

        //
        std::vector<size_t> _rbd_indices;
        std::string _end_effector;
        size_t _ef_index;

        //
        Eigen::VectorXd C_;
        Eigen::MatrixXd J_;
    
    }; // class IiwaTools
} // namespace iiwa_service

#endif // IIWA_SERVICE_IIWA_TOOLS_H