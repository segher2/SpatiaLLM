cmake_minimum_required(VERSION 3.15)
project(mask2cluster VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(PCL 1.10 REQUIRED COMPONENTS common io search)
find_package(Eigen3 REQUIRED)

# Option to enable PDAL support for LAS files
option(M2C_WITH_PDAL "Enable PDAL support for LAS file loading" OFF)

# Include nlohmann/json (header-only)
# Assume it's available in the system or add as subdirectory
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found, will try to use system headers")
endif()

# Point to LM2PCG for FEC implementation
set(LM2PCG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../LM2PCG" CACHE PATH "Path to LM2PCG root")
set(FEC_INCLUDE_DIR "${LM2PCG_ROOT}/include")

# Check if FEC source exists
if(NOT EXISTS "${FEC_INCLUDE_DIR}/pcg/FEC.hpp")
  message(FATAL_ERROR "FEC.hpp not found at ${FEC_INCLUDE_DIR}/pcg/FEC.hpp. Please set LM2PCG_ROOT correctly.")
endif()

# Build mask2cluster library
add_library(mask2cluster STATIC
  src/validator.cpp
  src/pipeline.cpp
  src/kdtree.cpp
  src/io_pose.cpp
  src/io_las.cpp
)

target_include_directories(mask2cluster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${FEC_INCLUDE_DIR}
)

target_link_libraries(mask2cluster PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

if(M2C_WITH_PDAL)
  find_package(PDAL REQUIRED)
  target_compile_definitions(mask2cluster PUBLIC M2C_HAS_PDAL)
  target_link_libraries(mask2cluster PUBLIC ${PDAL_LIBRARIES})
  target_include_directories(mask2cluster PUBLIC ${PDAL_INCLUDE_DIRS})
endif()

# Build mask2cluster_cli executable
add_executable(mask2cluster_cli
  src/mask2cluster_cli.cpp
)

target_link_libraries(mask2cluster_cli PRIVATE mask2cluster)

# Installation
install(TARGETS mask2cluster mask2cluster_cli
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
