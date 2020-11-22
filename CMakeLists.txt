cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})
project(${project_name})

add_definitions(-std=c++14)

set(MLR_LIBRARIES_DIR "/home/camille/Dev/mlr/rai/lib")
set(MLR_INCLUDE_DIR "/home/camille/Dev/mlr/rai/rai")

#set(DLIB_LIBRARIES_DIR "/usr/local/lib")
#set(DLIB_INCLUDE_DIR "/usr/local/inlcude/")

add_subdirectory("libs/HessianDecomposition")
add_subdirectory("libs/GeometricUtilityTasks")
add_subdirectory("libs/ObservationTasks")
add_subdirectory("libs/CollisionAvoidance")
add_subdirectory("libs/MathUtility")
add_subdirectory("libs/Optimization")
add_subdirectory("libs/MultiAgentTaskPlanning")
add_subdirectory("libs/MotionPlanning")
add_subdirectory("libs/POMTP_interfaces")
add_subdirectory("libs/POMTP_controllers")
add_subdirectory("apps")

#add_subdirectory("libs/TaskPlanning")
