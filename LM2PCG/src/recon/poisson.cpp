#include "pcg/recon/poisson.hpp"

#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/boost/graph/helpers.h>

#include <vector>
#include <utility>
#include <iostream>

namespace pcg { namespace recon {

using PointWithNormal = std::pair<Point3, Vec3>;

bool poisson_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                         Mesh& output_mesh,
                         const PoissonParams& p)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "Poisson: empty input cloud" << std::endl;
        return false;
    }

    // Convert to CGAL points
    std::vector<Point3> points; points.reserve(cloud->size());
    for (const auto& q : cloud->points) {
        if (!std::isnan(q.x) && !std::isnan(q.y) && !std::isnan(q.z))
            points.emplace_back(q.x, q.y, q.z);
    }
    if (points.size() < 3) return false;

    // Average spacing
    const auto avg = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, p.spacing_neighbors);

    // Prepare point-normal pairs (init normals to zero)
    std::vector<PointWithNormal> pwn; pwn.reserve(points.size());
    for (const auto& pt : points) pwn.emplace_back(pt, Vec3(0,0,0));

    // Jet normals
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
        pwn, p.normal_neighbors,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointWithNormal>())
                         .normal_map(CGAL::Second_of_pair_property_map<PointWithNormal>()));

    // Orient normals (MST)
    auto unoriented_begin = CGAL::mst_orient_normals(
        pwn, p.normal_neighbors,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointWithNormal>())
                         .normal_map(CGAL::Second_of_pair_property_map<PointWithNormal>()));
    const std::size_t oriented = static_cast<std::size_t>(std::distance(pwn.begin(), unoriented_begin));
    const std::size_t total    = pwn.size();
    const double frac = total > 0 ? static_cast<double>(oriented) / static_cast<double>(total) : 0.0;
    // Keep only oriented points for reconstruction
    pwn.erase(unoriented_begin, pwn.end());
    if (pwn.size() < 3) return false;
    if (frac < p.min_oriented_fraction) return false;

    // Poisson reconstruction
    output_mesh.clear();
    const bool ok = CGAL::poisson_surface_reconstruction_delaunay(
        pwn.begin(), pwn.end(),
        CGAL::First_of_pair_property_map<PointWithNormal>(),
        CGAL::Second_of_pair_property_map<PointWithNormal>(),
        output_mesh, avg);
    if (!ok) return false;

    // Ensure outward orientation
    namespace PMP = CGAL::Polygon_mesh_processing;
    if (!PMP::is_outward_oriented(output_mesh)) {
        PMP::reverse_face_orientations(output_mesh);
    }
    // Optionally enforce closedness: if not closed, reject to allow AF fallback
    if (p.require_closed_output && !CGAL::is_closed(output_mesh)) {
        return false;
    }
    return !output_mesh.is_empty();
}

} } // namespace pcg::recon
