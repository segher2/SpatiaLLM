#include "pcg/recon/af.hpp"

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <vector>
#include <array>

namespace pcg { namespace recon {

static bool build_mesh_from_facets(const std::vector<Point3>& pts,
                                   const std::vector<std::array<std::size_t,3>>& facets,
                                   Mesh& mesh)
{
    mesh.clear();
    std::vector<Mesh::Vertex_index> vmap;
    vmap.reserve(pts.size());
    for (const auto& p : pts) vmap.push_back(mesh.add_vertex(p));
    std::size_t ok = 0;
    for (const auto& f : facets) {
        if (f[0] >= vmap.size() || f[1] >= vmap.size() || f[2] >= vmap.size()) continue;
        const auto a = vmap[f[0]], b = vmap[f[1]], c = vmap[f[2]];
        if (mesh.add_face(a,b,c) != Mesh::null_face()) ++ok;
    }
    return ok > 0;
}

bool af_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    Mesh& mesh,
                    const AFParams& params)
{
    if (!cloud || cloud->empty()) return false;
    std::vector<Point3> points; points.reserve(cloud->size());
    for (const auto& q : cloud->points) {
        if (!std::isnan(q.x) && !std::isnan(q.y) && !std::isnan(q.z))
            points.emplace_back(q.x, q.y, q.z);
    }
    if (static_cast<int>(points.size()) < std::max(3, params.min_points)) return false;
    std::vector<std::array<std::size_t,3>> facets;
    CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(facets));
    if (facets.empty()) return false;
    if (!build_mesh_from_facets(points, facets, mesh)) return false;
    if (params.require_closed_output && !CGAL::is_closed(mesh)) return false;
    return true;
}

bool af_reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    Mesh& mesh)
{
    AFParams defaults{};
    return af_reconstruct(cloud, mesh, defaults);
}

} } // namespace pcg::recon
