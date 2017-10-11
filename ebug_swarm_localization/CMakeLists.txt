cmake_minimum_required(VERSION 2.8.3)
project(ebug_swarm_localization)

find_package(catkin_simple REQUIRED COMPONENTS system roscpp std_msgs message_generation)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ebug_swarm
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

catkin_simple()

# find_package(Boost REQUIRED COMPONENTS system thread)
# find_package(REQUIRED COMPONENTS roscpp std_msgs message_generation)
#cs_add_library(my_lib src/my_lib.cpp)

cs_add_executable(localization_node src/localization_node.cpp)
# target_link_libraries(my_exec my_lib)

target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  )

cs_install()

cs_install_scripts(scripts/my_script.py)

cs_export()
