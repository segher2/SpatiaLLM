#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace pcg { namespace recon {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernel::Point_3;
using Mesh   = CGAL::Surface_mesh<Point3>;

struct AFParams {
    int  min_points = 3;          // minimum points required to attempt AF
    bool require_closed_output = false; // if true, reject mesh when not closed
};

bool af_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    Mesh& mesh);

// Overload with parameters
bool af_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    Mesh& mesh,
                    const AFParams& params);

} } // namespace pcg::recon
