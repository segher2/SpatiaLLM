#include "pcg/bbox.hpp"

#include <Eigen/Eigenvalues>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace pcg {

static inline void minmax_z(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                            const std::vector<int>& idx,
                            float& zmin, float& zmax) {
    zmin = std::numeric_limits<float>::infinity();
    zmax = -std::numeric_limits<float>::infinity();
    for (int i : idx) {
        const float z = cloud->points[static_cast<std::size_t>(i)].z;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
    }
}

// Cross product for 2D vectors (returns z component)
static inline float cross2d(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// Andrew's monotone chain algorithm for 2D convex hull
static std::vector<Eigen::Vector2f> convex_hull_2d(std::vector<Eigen::Vector2f> points) {
    if (points.size() < 3) return points;
    
    // Sort points lexicographically (first by x, then by y)
    std::sort(points.begin(), points.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    
    // Remove duplicates
    points.erase(std::unique(points.begin(), points.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return (a - b).norm() < 1e-6f;
    }), points.end());
    
    if (points.size() < 3) return points;
    
    std::vector<Eigen::Vector2f> hull;
    
    // Build lower hull
    for (const auto& p : points) {
        while (hull.size() >= 2) {
            const Eigen::Vector2f v1 = hull[hull.size() - 1] - hull[hull.size() - 2];
            const Eigen::Vector2f v2 = p - hull[hull.size() - 1];
            if (cross2d(v1, v2) <= 0) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(p);
    }
    
    // Build upper hull
    const size_t lower_size = hull.size();
    for (int i = static_cast<int>(points.size()) - 2; i >= 0; --i) {
        const auto& p = points[i];
        while (hull.size() > lower_size) {
            const Eigen::Vector2f v1 = hull[hull.size() - 1] - hull[hull.size() - 2];
            const Eigen::Vector2f v2 = p - hull[hull.size() - 1];
            if (cross2d(v1, v2) <= 0) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(p);
    }
    
    // Remove the last point (duplicate of first)
    hull.pop_back();
    
    return hull;
}

// Rotating calipers to find minimum area bounding rectangle
struct MinRectResult {
    float theta;      // rotation angle
    float min_u, max_u, min_v, max_v;
    float area;
};

static MinRectResult rotating_calipers(const std::vector<Eigen::Vector2f>& hull) {
    if (hull.empty()) {
        return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    if (hull.size() == 1) {
        return {0.0f, hull[0].x(), hull[0].x(), hull[0].y(), hull[0].y(), 0.0f};
    }
    
    MinRectResult best;
    best.area = std::numeric_limits<float>::infinity();
    
    const size_t n = hull.size();
    
    // Try each edge direction
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector2f edge = hull[(i + 1) % n] - hull[i];
        const float edge_len = edge.norm();
        if (edge_len < 1e-9f) continue;
        
        // Unit vectors for this edge direction
        const Eigen::Vector2f u = edge / edge_len;
        const Eigen::Vector2f v(-u.y(), u.x());
        
        const float theta = std::atan2(u.y(), u.x());
        
        // Project all hull points onto u and v
        float min_u = std::numeric_limits<float>::infinity();
        float max_u = -std::numeric_limits<float>::infinity();
        float min_v = std::numeric_limits<float>::infinity();
        float max_v = -std::numeric_limits<float>::infinity();
        
        for (const auto& p : hull) {
            const float proj_u = u.dot(p);
            const float proj_v = v.dot(p);
            min_u = std::min(min_u, proj_u);
            max_u = std::max(max_u, proj_u);
            min_v = std::min(min_v, proj_v);
            max_v = std::max(max_v, proj_v);
        }
        
        const float L = max_u - min_u;
        const float W = max_v - min_v;
        const float area = L * W;
        
        if (area < best.area) {
            best.area = area;
            best.theta = theta;
            best.min_u = min_u;
            best.max_u = max_u;
            best.min_v = min_v;
            best.max_v = max_v;
        }
    }
    
    return best;
}

UOBB compute_uobb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                  const std::vector<int>& indices) {
    UOBB box{};
    if (!cloud || indices.empty()) return box;

    // Project points to XY plane
    std::vector<Eigen::Vector2f> points_2d;
    points_2d.reserve(indices.size());
    for (int i : indices) {
        const auto& p = cloud->points[static_cast<std::size_t>(i)];
        points_2d.emplace_back(p.x, p.y);
    }
    
    // Compute 2D convex hull
    std::vector<Eigen::Vector2f> hull = convex_hull_2d(points_2d);
    
    // Find minimum area rectangle using rotating calipers
    MinRectResult rect = rotating_calipers(hull);
    
    // Ensure L >= W (length along e1 >= width along e2)
    float yaw = rect.theta;
    float lx = rect.max_u - rect.min_u;
    float ly = rect.max_v - rect.min_v;
    float min_u = rect.min_u;
    float max_u = rect.max_u;
    float min_v = rect.min_v;
    float max_v = rect.max_v;
    
    if (ly > lx) {
        // Swap L and W, rotate by 90 degrees
        std::swap(lx, ly);
        std::swap(min_u, min_v);
        std::swap(max_u, max_v);
        yaw += static_cast<float>(M_PI) / 2.0f;
    }
    
    // Normalize yaw to [-pi, pi]
    while (yaw > M_PI) yaw -= 2.0f * static_cast<float>(M_PI);
    while (yaw < -M_PI) yaw += 2.0f * static_cast<float>(M_PI);
    
    // Compute rotation matrix
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    const Eigen::Vector2f u(cos_yaw, sin_yaw);
    const Eigen::Vector2f v(-sin_yaw, cos_yaw);
    
    // Compute center in XY
    const float cx = u.x() * (min_u + max_u) * 0.5f + v.x() * (min_v + max_v) * 0.5f;
    const float cy = u.y() * (min_u + max_u) * 0.5f + v.y() * (min_v + max_v) * 0.5f;
    
    // Height from z-range
    float zmin, zmax;
    minmax_z(cloud, indices, zmin, zmax);
    const float lz = (zmax - zmin);
    const float cz = (zmin + zmax) * 0.5f;

    box.center = Eigen::Vector3f(cx, cy, cz);
    box.size   = Eigen::Vector3f(lx, ly, lz);
    box.yaw    = yaw;

    // Rotation matrix with up = Z
    box.R.setIdentity();
    box.R(0,0) = cos_yaw;  box.R(1,0) = sin_yaw;
    box.R(0,1) = -sin_yaw; box.R(1,1) = cos_yaw;
    // R.col(2) remains (0,0,1)

    // Corners
    const Eigen::Vector3f e1 = box.R.col(0);
    const Eigen::Vector3f e2 = box.R.col(1);
    const Eigen::Vector3f ez(0.f, 0.f, 1.f);
    const Eigen::Vector3f h = 0.5f * box.size;
    std::array<Eigen::Vector3f, 4> base{
        box.center + e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() - e2*h.y() - ez*h.z(),
        box.center + e1*h.x() - e2*h.y() - ez*h.z()
    };
    std::array<Eigen::Vector3f, 4> top{
        base[0] + ez*box.size.z(),
        base[1] + ez*box.size.z(),
        base[2] + ez*box.size.z(),
        base[3] + ez*box.size.z()
    };
    box.corners = { base[0], base[1], base[2], base[3], top[0], top[1], top[2], top[3] };

    return box;
}

// Minimal PLY writer for a box mesh (8 vertices, 12 triangles)
static bool write_box_ply_binary(const std::string& path, const std::array<Eigen::Vector3f,8>& V) {
    struct Tri { uint8_t n; uint32_t a,b,c; };
    const uint32_t nverts = 8;
    const Tri F[12] = {
        {3,0,1,2},{3,0,2,3}, // bottom
        {3,4,5,6},{3,4,6,7}, // top
        {3,0,1,5},{3,0,5,4},
        {3,1,2,6},{3,1,6,5},
        {3,2,3,7},{3,2,7,6},
        {3,3,0,4},{3,3,4,7}
    };
    const uint32_t nfaces = 12;

    std::ofstream os(path, std::ios::binary);
    if (!os) return false;
    os << "ply\n";
    os << "format binary_little_endian 1.0\n";
    os << "element vertex " << nverts << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "element face " << nfaces << "\n";
    os << "property list uchar int vertex_indices\n";
    os << "end_header\n";
    // verts
    for (const auto& v : V) {
        float x=v.x(), y=v.y(), z=v.z();
        os.write(reinterpret_cast<const char*>(&x), sizeof(float));
        os.write(reinterpret_cast<const char*>(&y), sizeof(float));
        os.write(reinterpret_cast<const char*>(&z), sizeof(float));
    }
    // faces
    for (const auto& t : F) {
        os.write(reinterpret_cast<const char*>(&t.n), sizeof(uint8_t));
        os.write(reinterpret_cast<const char*>(&t.a), sizeof(uint32_t));
        os.write(reinterpret_cast<const char*>(&t.b), sizeof(uint32_t));
        os.write(reinterpret_cast<const char*>(&t.c), sizeof(uint32_t));
    }
    return true;
}

bool save_uobb_ply(const std::string& filepath, const UOBB& box) {
    return write_box_ply_binary(filepath, box.corners);
}

} // namespace pcg
