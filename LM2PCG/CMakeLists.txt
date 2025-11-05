cmake_minimum_required(VERSION 3.16)
project(indoor_pc_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree)

add_library(pcg
    src/geometry/clustering.cpp
    src/geometry/bbox.cpp
    src/geometry/area.cpp
    src/geometry/volume.cpp
    src/io/ply_io.cpp
    src/io/csv_writer.cpp
    src/io/params.cpp
    src/color/color_gmm.cpp
    src/recon/poisson.cpp
    src/recon/af.cpp
)

target_include_directories(pcg PUBLIC include ${PCL_INCLUDE_DIRS})

# On macOS, suppress some PCL deprecation warnings
if(APPLE)
    target_compile_definitions(pcg PRIVATE _GLIBCXX_USE_CXX11_ABI=1)
endif()

# PCL definitions and libraries
add_definitions(${PCL_DEFINITIONS})
if(PCL_FOUND)
    message(STATUS "PCL found: ${PCL_VERSION}")
endif()

target_link_libraries(pcg PUBLIC ${PCL_LIBRARIES})

add_executable(pcg_room src/apps/pcg_room.cpp)

target_link_libraries(pcg_room PRIVATE pcg ${PCL_LIBRARIES})

target_include_directories(pcg_room PRIVATE include ${PCL_INCLUDE_DIRS})

## test utilities removed: pcg_synth

# Color analysis app (standalone CLI)
add_executable(pcg_color src/apps/pcg_color.cpp)
target_include_directories(pcg_color PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(pcg_color PRIVATE pcg ${PCL_LIBRARIES})

# BBox distance app (standalone CLI)
add_executable(pcg_bbox src/apps/pcg_bbox.cpp)
target_include_directories(pcg_bbox PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(pcg_bbox PRIVATE pcg ${PCL_LIBRARIES})

# BBox generator app (for testing)


# Optional: CGAL for reconstruction app
# Require CGAL 5.3 or higher for unified IO headers
find_package(CGAL 5.3 QUIET COMPONENTS Core)
if(CGAL_FOUND)
    message(STATUS "CGAL found: ${CGAL_VERSION} - building pcg_reconstruct, pcg_volume, pcg_area")
    add_executable(pcg_reconstruct src/apps/pcg_reconstruct.cpp)
    target_include_directories(pcg_reconstruct PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcg_reconstruct PRIVATE pcg ${PCL_LIBRARIES} CGAL::CGAL)

    # Mesh volume CLI
    add_executable(pcg_volume src/apps/pcg_volume.cpp)
    target_include_directories(pcg_volume PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcg_volume PRIVATE pcg ${PCL_LIBRARIES} CGAL::CGAL)

    # Mesh surface area CLI
    add_executable(pcg_area src/apps/pcg_area.cpp)
    target_include_directories(pcg_area PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcg_area PRIVATE pcg ${PCL_LIBRARIES} CGAL::CGAL)
else()
    message(WARNING "CGAL >= 5.3 not found: pcg_reconstruct, pcg_volume, and pcg_area will not be built")
    message(STATUS "Install CGAL 5.3+ to enable mesh processing features:")
    message(STATUS "  Ubuntu/Debian: sudo apt install libcgal-dev")
    message(STATUS "  macOS: brew install cgal")
    message(STATUS "  Or build from source: https://github.com/CGAL/cgal/releases")
endif()

# Installation rules (optional)
install(TARGETS pcg pcg_room RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
