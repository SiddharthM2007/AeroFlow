cmake_minimum_required(VERSION 3.20)
project(aeroflow_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(AEROFLOW_BUILD_TESTS "Build the AeroFlow engine unit tests" ON)

include(FetchContent)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# --- Core library -----------------------------------------------------------
add_library(aeroflow_grid
  src/grid3d.cpp
  src/box.cpp
  src/config.cpp
  src/metadata.cpp
  src/base64.cpp
  src/mesh.cpp
  src/ray.cpp
  src/bvh.cpp
  src/voxelize.cpp
  src/mac_grid.cpp
  src/divergence.cpp
  src/field_io.cpp
  src/field_export.cpp
  src/boundary.cpp
)
target_include_directories(aeroflow_grid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aeroflow_grid PUBLIC nlohmann_json::nlohmann_json)
target_compile_definitions(aeroflow_grid PUBLIC AEROFLOW_VERSION="${PROJECT_VERSION}")
if(MSVC)
  target_compile_options(aeroflow_grid PRIVATE /W4)
else()
  target_compile_options(aeroflow_grid PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

# --- CLI --------------------------------------------------------------------
add_executable(aeroflow-grid src/main.cpp)
target_link_libraries(aeroflow-grid PRIVATE aeroflow_grid)

# --- Tests ------------------------------------------------------------------
if(AEROFLOW_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(aeroflow_tests
    tests/test_grid3d.cpp
    tests/test_box.cpp
    tests/test_config.cpp
    tests/test_metadata.cpp
    tests/test_mesh.cpp
    tests/test_voxelize.cpp
    tests/test_mac_grid.cpp
    tests/test_divergence.cpp
    tests/test_fields.cpp
    tests/test_boundary.cpp
  )
  target_link_libraries(aeroflow_tests PRIVATE aeroflow_grid GTest::gtest_main)
  target_compile_definitions(aeroflow_tests PRIVATE AEROFLOW_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../fixtures")
  include(GoogleTest)
  gtest_discover_tests(aeroflow_tests)
endif()
