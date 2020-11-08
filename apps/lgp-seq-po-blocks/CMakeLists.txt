cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################

include_directories( ${MLR_INCLUDE_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

#################################
# Define executable
#################################

add_executable(${project_name} main.cpp)
target_link_libraries(
 ${project_name}
 Core
 #Motion
 Geo
 Gui
 Optim
 POMTP_interfaces
 ObservationTasks
 CollisionAvoidance
 )
