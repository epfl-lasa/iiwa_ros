#include <iiwa_gazebo/mc_rbdyn_urdf.h>

// RBDyn headers
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_rbdyn_urdf {
    std::string parse_multi_body_graph_from_urdf(URDFParserResult& res, const urdf::Model* const urdf_model, const std::vector<std::string>& filteredLinksIn, bool transformInertia, const std::string& baseLinkIn, bool withVirtualLinks, const std::string sphericalSuffix)
    {
        std::vector<urdf::LinkSharedPtr> links;
        std::vector<std::string> filtered_links = filteredLinksIn;

        // Extract link elements from the document, remove filtered links
        for (std::map<std::string, urdf::LinkSharedPtr>::const_iterator it = urdf_model->links_.begin(); it != urdf_model->links_.end(); it++) {
            urdf::LinkSharedPtr link = it->second;
            std::string link_name = link->name;
            if (std::find(filtered_links.begin(), filtered_links.end(), link_name) == filtered_links.end()) {
                if (!withVirtualLinks && !link->inertial) {
                    filtered_links.push_back(link_name);
                }
                else {
                    links.push_back(link);
                }
            }
        }

        if (links.size() == 0) {
            std::cerr << "Failed to extract any link information from the URDF, parsing will stop now" << std::endl;
            return "";
        }

        std::string baseLink = baseLinkIn == "" ? links[0]->name : baseLinkIn;

        for (size_t i = 0; i < links.size(); i++) {
            urdf::LinkSharedPtr link = links[i];

            std::string link_name = link->name;
            double mass = 0.0;
            Eigen::Vector3d com = Eigen::Vector3d::Zero();
            Eigen::Matrix3d inertia_o = Eigen::Matrix3d::Zero();

            if (link->inertial) {
                com << link->inertial->origin.position.x, link->inertial->origin.position.y, link->inertial->origin.position.z;

                double r, p, y;
                link->inertial->origin.rotation.getRPY(r, p, y);

                Eigen::Matrix3d com_frame = mc_rbdyn_urdf::RPY(r, p, y);
                mass = link->inertial->mass;
                Eigen::Matrix3d inertia;
                inertia << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz,
                    link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
                    link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
                if (transformInertia) {
                    inertia_o = sva::inertiaToOrigin(inertia, mass, com, com_frame);
                }
                else {
                    inertia_o = inertia;
                }
            }

            for (size_t j = 0; j < link->visual_array.size(); j++) {
                urdf::VisualSharedPtr visual = link->visual_array[j];

                Visual v;

                {
                    sva::PTransformd tf = sva::PTransformd::Identity();
                    Eigen::Vector3d T;
                    T << visual->origin.position.x, visual->origin.position.y, visual->origin.position.z;

                    double r, p, y;
                    visual->origin.rotation.getRPY(r, p, y);

                    Eigen::Matrix3d R = RPY(r, p, y);
                    tf = sva::PTransformd(R, T);

                    v.origin = tf;
                }

                {
                    switch (visual->geometry->type) {
                    case urdf::Geometry::MESH: {
                        v.geometry.type = Geometry::Type::MESH;
                        auto& mesh = boost::get<Geometry::Mesh>(v.geometry.data);
                        mesh.filename = boost::static_pointer_cast<urdf::Mesh>(visual->geometry)->filename;
                        mesh.scale = boost::static_pointer_cast<urdf::Mesh>(visual->geometry)->scale.x; // TO-DO: This is not correct; scale is 3D vector
                        v.name = visual->name;
                        res.visual[link_name].push_back(v);
                        break;
                    }
                    default:
                        std::cerr << "Warning: only mesh geometry is supported, visual element has been ignored" << std::endl;
                    }
                }
            }

            for (size_t j = 0; j < link->collision_array.size(); j++) {
                urdf::CollisionSharedPtr collision = link->collision_array[j];

                {
                    sva::PTransformd tf = sva::PTransformd::Identity();
                    Eigen::Vector3d T;
                    T << collision->origin.position.x, collision->origin.position.y, collision->origin.position.z;

                    double r, p, y;
                    collision->origin.rotation.getRPY(r, p, y);

                    Eigen::Matrix3d R = RPY(r, p, y);
                    tf = sva::PTransformd(R, T);

                    res.collision_tf[link_name] = tf;
                }
            }

            rbd::Body b(mass, com, inertia_o, link_name);
            res.mbg.addBody(b);
        }

        std::vector<urdf::JointSharedPtr> joints;

        // Extract joint elements from the document, remove joints that link with filtered links
        for (std::map<std::string, urdf::JointSharedPtr>::const_iterator it = urdf_model->joints_.begin(); it != urdf_model->joints_.end(); it++) {
            urdf::JointSharedPtr joint = it->second;
            std::string parent_link = joint->parent_link_name;
            std::string child_link = joint->child_link_name;

            if (std::find(filtered_links.begin(), filtered_links.end(), child_link) == filtered_links.end() && std::find(filtered_links.begin(), filtered_links.end(), parent_link) == filtered_links.end()) {
                joints.push_back(joint);
            }
        }

        for (size_t i = 0; i < joints.size(); i++) {
            urdf::JointSharedPtr joint = joints[i];

            // Static transformation
            sva::PTransformd static_transform = sva::PTransformd::Identity();

            {
                Eigen::Vector3d T;
                T << joint->parent_to_joint_origin_transform.position.x, joint->parent_to_joint_origin_transform.position.y, joint->parent_to_joint_origin_transform.position.z;

                double r, p, y;
                joint->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);

                Eigen::Matrix3d R = RPY(r, p, y);
                static_transform = sva::PTransformd(R, T);
            }

            // Read the joint axis
            Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
            axis << joint->axis.x, joint->axis.y, joint->axis.z;

            std::string joint_name = joint->name;
            std::string joint_type = "revolute";
            if (joint->type == urdf::Joint::REVOLUTE)
                joint_type = "revolute";
            else if (joint->type == urdf::Joint::CONTINUOUS)
                joint_type = "continuous";
            else if (joint->type == urdf::Joint::PRISMATIC)
                joint_type = "prismatic";
            else if (joint->type == urdf::Joint::FLOATING)
                joint_type = "floating";
            else if (joint->type == urdf::Joint::FIXED)
                joint_type = "fixed";
            rbd::Joint::Type type = rbdynFromUrdfJoint(joint_type, (joint_name.length() >= sphericalSuffix.length() && joint_name.substr(joint_name.length() - sphericalSuffix.length(), sphericalSuffix.length()) == sphericalSuffix));

            std::string joint_parent = joint->parent_link_name;
            std::string joint_child = joint->child_link_name;

            rbd::Joint j(type, axis, true, joint_name);

            if (joint->mimic) {
                std::string mimic_joint = joint->mimic->joint_name;
                double multiplier = joint->mimic->multiplier;
                double offset = joint->mimic->offset;
                j.makeMimic(mimic_joint, multiplier, offset);
            }

            res.mbg.addJoint(j);
            res.mbg.linkBodies(joint_parent, static_transform, joint_child, sva::PTransformd::Identity(), joint_name);

            // Articular limit
            std::vector<double> lower(static_cast<size_t>(j.dof()), -INFINITY);
            std::vector<double> upper(static_cast<size_t>(j.dof()), INFINITY);
            std::vector<double> effort(static_cast<size_t>(j.dof()), INFINITY);
            std::vector<double> velocity(static_cast<size_t>(j.dof()), INFINITY);

            if (joint->limits && j.type() != rbd::Joint::Fixed) {
                if (joint_type != "continuous") {
                    lower = std::vector<double>(static_cast<size_t>(j.dof()), joint->limits->lower);
                    upper = std::vector<double>(static_cast<size_t>(j.dof()), joint->limits->upper);
                }

                effort = std::vector<double>(static_cast<size_t>(j.dof()), joint->limits->effort);
                velocity = std::vector<double>(static_cast<size_t>(j.dof()), joint->limits->velocity);
            }

            auto check_limit = [&j](const std::string& name, const std::vector<double>& limit) {
                if (limit.size() != static_cast<size_t>(j.dof())) {
                    std::cerr << "Joint " << name << " limit for " << j.name() << ": size missmatch, expected: " << j.dof() << ", got: " << limit.size() << std::endl;
                }
            };

            check_limit("lower", lower);
            check_limit("upper", upper);
            check_limit("effort", effort);
            check_limit("velocity", velocity);

            res.limits.lower[joint_name] = lower;
            res.limits.upper[joint_name] = upper;
            res.limits.torque[joint_name] = effort;
            res.limits.velocity[joint_name] = velocity;
        }

        return baseLink;
    }

    URDFParserResult rbdyn_from_urdf(const urdf::Model* const urdf_model, bool fixed, const std::vector<std::string>& filteredLinksIn, bool transformInertia, const std::string& baseLinkIn, bool withVirtualLinks, const std::string sphericalSuffix)
    {
        URDFParserResult res;

        std::string baseLink = parse_multi_body_graph_from_urdf(res, urdf_model, filteredLinksIn, transformInertia, baseLinkIn, withVirtualLinks, sphericalSuffix);

        res.mb = res.mbg.makeMultiBody(baseLink, fixed);
        res.mbc = rbd::MultiBodyConfig(res.mb);
        res.mbc.zero(res.mb);

        rbd::forwardKinematics(res.mb, res.mbc);
        rbd::forwardVelocity(res.mb, res.mbc);

        return res;
    }
} // namespace mc_rbdyn_urdf