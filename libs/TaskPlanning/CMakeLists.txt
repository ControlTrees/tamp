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

include_directories( ${MLR_LIBRARIES_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
 Core
 Logic
 MCTS
 POMTP_interfaces
 )

add_subdirectory(test)
