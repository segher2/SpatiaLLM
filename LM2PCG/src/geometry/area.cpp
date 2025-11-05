#include "pcg/geometry/area.hpp"

#include <CGAL/Polygon_mesh_processing/measure.h>

namespace pcg { namespace geom {

double mesh_surface_area(const Mesh& mesh) {
    if (mesh.is_empty()) return 0.0;
    namespace PMP = CGAL::Polygon_mesh_processing;
    try {
        return PMP::area(mesh);
    } catch (...) {
        return 0.0;
    }
}

} } // namespace pcg::geom
