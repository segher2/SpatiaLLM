#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace pcg { namespace recon {

struct PoissonParams {
    int spacing_neighbors = 6;
    int normal_neighbors  = 18;
    // Minimum oriented points and fraction required to attempt Poisson
    int    min_points = 0; // no minimum by default
    double min_oriented_fraction = 0.3; // oriented/total >= this
    // If true, only accept Poisson result when mesh is closed; otherwise treat as failure
    bool   require_closed_output = true;
};

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernel::Point_3;
using Vec3   = Kernel::Vector_3;
using Mesh   = CGAL::Surface_mesh<Point3>;

// Returns true on success; fills mesh
bool poisson_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                         Mesh& mesh,
                         const PoissonParams& params);

} } // namespace pcg::recon
