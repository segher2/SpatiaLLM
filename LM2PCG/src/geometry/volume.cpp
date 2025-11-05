#include "pcg/geometry/volume.hpp"

#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Bbox_3.h>
#include <queue>
#include <cmath>

namespace pcg { namespace geom {

double mesh_signed_volume(const Mesh& mesh) {
    if (mesh.is_empty()) return 0.0;
    namespace PMP = CGAL::Polygon_mesh_processing;
    // volume() returns a signed volume; requires closed mesh for meaningful result
    try {
        double v = PMP::volume(mesh);
        return v >= 0.0 ? v : -v;
    } catch (...) {
        return 0.0;
    }
}

double convex_hull_volume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    if (!cloud || cloud->size() < 4) return 0.0;
    std::vector<Point3> pts; pts.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            pts.emplace_back(p.x, p.y, p.z);
    }
    if (pts.size() < 4) return 0.0;
    Mesh hull;
    try {
        CGAL::convex_hull_3(pts.begin(), pts.end(), hull);
    } catch (...) {
        return 0.0;
    }
    return mesh_signed_volume(hull);
}

double mesh_volume_adaptive_voxel(const Mesh& mesh, double base_voxel_size, int max_depth) {
    if (mesh.is_empty()) return 0.0;

    // Build AABB tree for fast ray-mesh intersection queries
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits = CGAL::AABB_traits<Kernel, Primitive>;
    using Tree = CGAL::AABB_tree<Traits>;
    
    Tree tree;
    try {
        tree.insert(faces(mesh).first, faces(mesh).second, mesh);
        tree.build();
    } catch (...) {
        return 0.0;
    }

    // Get mesh bounding box
    CGAL::Bbox_3 bbox = tree.bbox();
    double dx = bbox.xmax() - bbox.xmin();
    double dy = bbox.ymax() - bbox.ymin();
    double dz = bbox.zmax() - bbox.zmin();
    
    // If base_voxel_size not specified, auto-compute from bbox
    if (base_voxel_size <= 0.0) {
        double max_dim = std::max({dx, dy, dz});
        base_voxel_size = max_dim / 20.0; // 20 voxels along longest axis
    }

    // Voxel structure for adaptive refinement
    struct Voxel {
        double xmin, ymin, zmin;
        double size;
        int depth;
        
        Voxel(double x, double y, double z, double s, int d)
            : xmin(x), ymin(y), zmin(z), size(s), depth(d) {}
        
        Point3 center() const {
            return Point3(xmin + size * 0.5, ymin + size * 0.5, zmin + size * 0.5);
        }
    };

    // Check if voxel center is inside mesh using ray casting
    auto is_inside = [&](const Point3& p) -> bool {
        // Cast ray from point along +X direction
        Point3 far_point(bbox.xmax() + dx, p.y(), p.z());
        auto ray = Kernel::Ray_3(p, far_point);
        
        std::vector<Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type> intersections;
        try {
            tree.all_intersections(ray, std::back_inserter(intersections));
        } catch (...) {
            return false;
        }
        
        // Odd number of intersections = inside
        return (intersections.size() % 2) == 1;
    };

    // Check if voxel is at boundary (partially inside/outside)
    auto is_boundary = [&](const Voxel& v) -> bool {
        // Sample 8 corners
        std::vector<Point3> corners = {
            Point3(v.xmin, v.ymin, v.zmin),
            Point3(v.xmin + v.size, v.ymin, v.zmin),
            Point3(v.xmin, v.ymin + v.size, v.zmin),
            Point3(v.xmin + v.size, v.ymin + v.size, v.zmin),
            Point3(v.xmin, v.ymin, v.zmin + v.size),
            Point3(v.xmin + v.size, v.ymin, v.zmin + v.size),
            Point3(v.xmin, v.ymin + v.size, v.zmin + v.size),
            Point3(v.xmin + v.size, v.ymin + v.size, v.zmin + v.size)
        };
        
        bool first_inside = is_inside(corners[0]);
        for (size_t i = 1; i < corners.size(); ++i) {
            if (is_inside(corners[i]) != first_inside) {
                return true; // Boundary voxel
            }
        }
        return false;
    };

    double total_volume = 0.0;
    std::queue<Voxel> queue;
    
    // Initialize with base-level voxels covering the bounding box
    int nx = static_cast<int>(std::ceil(dx / base_voxel_size));
    int ny = static_cast<int>(std::ceil(dy / base_voxel_size));
    int nz = static_cast<int>(std::ceil(dz / base_voxel_size));
    
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                queue.emplace(
                    bbox.xmin() + i * base_voxel_size,
                    bbox.ymin() + j * base_voxel_size,
                    bbox.zmin() + k * base_voxel_size,
                    base_voxel_size,
                    0
                );
            }
        }
    }

    // Process voxels with adaptive refinement
    while (!queue.empty()) {
        Voxel v = queue.front();
        queue.pop();
        
        // Check if voxel center is inside
        if (!is_inside(v.center())) {
            continue; // Outside mesh, skip
        }
        
        // If at boundary and can subdivide, refine
        if (v.depth < max_depth && is_boundary(v)) {
            // Subdivide into 8 sub-voxels
            double half = v.size * 0.5;
            for (int di = 0; di < 2; ++di) {
                for (int dj = 0; dj < 2; ++dj) {
                    for (int dk = 0; dk < 2; ++dk) {
                        queue.emplace(
                            v.xmin + di * half,
                            v.ymin + dj * half,
                            v.zmin + dk * half,
                            half,
                            v.depth + 1
                        );
                    }
                }
            }
        } else {
            // Fully inside or max depth reached, count volume
            total_volume += v.size * v.size * v.size;
        }
    }

    return total_volume;
}

} } // namespace pcg::geom
