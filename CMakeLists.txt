cmake_minimum_required(VERSION 3.16)
project(m_detector_offline LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations)
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(NANO_GICP_SOURCES
  direct_lidar_odometry/third_party/nano_gicp/src/nano_gicp.cc
  direct_lidar_odometry/third_party/nano_gicp/src/lsq_registration.cc
)

add_library(direct_lidar_odometry_lib
  direct_lidar_odometry/src/lidar_odometry.cpp
  ${NANO_GICP_SOURCES}
)
target_include_directories(direct_lidar_odometry_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/direct_lidar_odometry/include
  ${CMAKE_CURRENT_SOURCE_DIR}/direct_lidar_odometry/third_party/nano_gicp/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(direct_lidar_odometry_lib PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::filesystem
  Boost::system
  OpenMP::OpenMP_CXX
)

add_library(m_detector_core
  src/DynObjFilter.cpp
  src/DynObjCluster.cpp
  src/offline_config.cpp
  src/offline_utils.cpp
)
target_include_directories(m_detector_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(m_detector_core PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::filesystem
  Boost::system
  OpenMP::OpenMP_CXX
  TBB::tbb
  ${YAML_CPP_TARGET}
)

add_executable(m_detector_offline src/offline_runner.cpp)
target_link_libraries(m_detector_offline PRIVATE
  m_detector_core
  direct_lidar_odometry_lib
)

enable_testing()
add_executable(test_offline_utils tests/test_offline_utils.cpp)
target_link_libraries(test_offline_utils PRIVATE m_detector_core)
add_test(NAME offline_utils COMMAND test_offline_utils)
