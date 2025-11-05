#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace pcg { namespace geom {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernel::Point_3;
using Mesh   = CGAL::Surface_mesh<Point3>;

// Compute surface area of a polygon mesh (sum of face areas).
// Works for open or closed meshes; returns 0.0 for empty or on failure.
double mesh_surface_area(const Mesh& mesh);

} } // namespace pcg::geom
