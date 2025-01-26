cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


# add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/esdf_server.cc
  src/interactive_slider.cc
  src/simulation_server.cc
  src/intensity_server.cc
  src/transformer.cc
  src/tsdf_server.cc
)

############
# BINARIES #
############
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

cs_add_executable(voxblox_eval
  src/voxblox_eval.cc
)
target_link_libraries(voxblox_eval ${PROJECT_NAME})
set_property(TARGET voxblox_eval PROPERTY CXX_STANDARD 17)

cs_add_executable(tsdf_server
  src/tsdf_server_node.cc
)
target_link_libraries(tsdf_server ${PROJECT_NAME})
set_property(TARGET tsdf_server PROPERTY CXX_STANDARD 17)

cs_add_executable(esdf_server
  src/esdf_server_node.cc
)
target_link_libraries(esdf_server ${PROJECT_NAME})
set_property(TARGET esdf_server PROPERTY CXX_STANDARD 17)

cs_add_executable(intensity_server
  src/intensity_server_node.cc
)
target_link_libraries(intensity_server ${PROJECT_NAME})
set_property(TARGET intensity_server PROPERTY CXX_STANDARD 17)

cs_add_executable(simulation_eval
  src/simulation_eval.cc
)
target_link_libraries(simulation_eval ${PROJECT_NAME})
set_property(TARGET simulation_eval PROPERTY CXX_STANDARD 17)

cs_add_executable(visualize_tsdf
  src/visualize_tsdf.cc
)
target_link_libraries(visualize_tsdf ${PROJECT_NAME})
set_property(TARGET visualize_tsdf PROPERTY CXX_STANDARD 17)

##########
# EXPORT #
##########
cs_install()
cs_export()
