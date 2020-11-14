cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################


#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} ${SOURCES})
target_include_directories(${project_name} PUBLIC "include")
target_link_libraries(
 ${project_name}
 Core
 pthread
 dlib
)

add_subdirectory(test)
