cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate dependencies
#################################

find_package(Boost)

#################################
# Locate MLR libs
#################################

include_directories( ${MLR_INCLUDE_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

#################################
# Define executable
#################################

file(GLOB PROJECT_SRC "*.cpp")

add_executable(${project_name} ${PROJECT_SRC})
target_link_libraries(
 ${project_name}
 Core
 Geo
 Gui
 Optim
 Logic
 MCTS
 KOMO
 POMTP_interfaces
 ObservationTasks
 CollisionAvoidance
 MultiAgentTaskPlanning
 MotionPlanning
 GeometricUtilityTasks
 )
