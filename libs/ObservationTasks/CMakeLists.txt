cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

#################################
# Locate MLR libs
#################################

#################################
# Define executable
#################################
add_definitions(-std=c++14)

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
 Core
 #Motion
 Geo
 Kin
 Gui
 Optim
 KOMO
 MathUtility
 )
