cmake_minimum_required(VERSION 3.10)
project(gps_imu_fusion)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

set(ALL_TARGET_LIBRARIES "")

include(cmake/eigen.cmake)
include(cmake/YAML.cmake)
include(cmake/global_defination.cmake)

include_directories(${PROJECT_SOURCE_DIR}/3rd/GeographicLib/include/)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(DEPS SHARED
    src/eskf.cpp
    src/gps_flow.cpp
    src/imu_flow.cpp
    src/eskf_flow.cpp
    src/observability_analysis.cpp
    3rd/GeographicLib/src/LocalCartesian.cpp
    3rd/GeographicLib/src/Geocentric.cpp
    3rd/GeographicLib/src/Math.cpp
)

target_link_libraries(DEPS
    ${ALL_TARGET_LIBRARIES}
)

add_executable(gps_imu_fusion app/gps_imu_fusion.cpp)
target_link_libraries(gps_imu_fusion DEPS)

#################  TEST  #################
add_executable(test_gps test/test_gps.cpp)
target_link_libraries(test_gps DEPS)

add_executable(test_imu test/test_imu.cpp)
target_link_libraries(test_imu DEPS)
