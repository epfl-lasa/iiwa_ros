#ifndef IIWA_GAZEBO_MC_RBDYN_URDF_H
#define IIWA_GAZEBO_MC_RBDYN_URDF_H

// URDF headers
#include <urdf/model.h>

// RBDyn headers
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_rbdyn_urdf {
    std::string parse_multi_body_graph_from_urdf(URDFParserResult& res, const urdf::Model* const urdf_model, const std::vector<std::string>& filteredLinksIn = {}, bool transformInertia = true, const std::string& baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");

    URDFParserResult rbdyn_from_urdf(const urdf::Model* const urdf_model, bool fixed = true, const std::vector<std::string>& filteredLinksIn = {}, bool transformInertia = true, const std::string& baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");
} // namespace mc_rbdyn_urdf

#endif