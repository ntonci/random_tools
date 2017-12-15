cmake_minimum_required(VERSION 2.8.3)
project(bag_extractor)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# LIBRARY
# cs_add_library(${PROJECT_NAME}
#   src/bag_extractor.cc
# )

cs_add_executable(${PROJECT_NAME}_node
  src/bag_extractor_node.cc
)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_install()
cs_export()
