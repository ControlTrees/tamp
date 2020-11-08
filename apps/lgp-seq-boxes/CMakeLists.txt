cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################

find_path( MLR_LIBRARIES_DIR
           NAMES libCore.so
           PATHS "/home/camille/Dev/Phd/mlr_sparse/rai/rai" )

#message( "path Camille: /home/mtoussai/git/copy/mlr/share/rai/rai" )

include_directories( ${MLR_LIBRARIES_DIR} )
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
