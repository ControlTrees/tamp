cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################

include_directories( ${MLR_INCLUDE_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

################################
# External libs
################################
#find_package(fcl REQUIRED)

#find OpenCV
find_package(OpenCV 3.3.1 REQUIRED)

#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
#message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "    libs path: ${OPENCV_LIBRARY_DIRS}")

#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
 Core
 Geo
 Kin
 KOMO
 MathUtility
 ${OpenCV_LIBS}
 #${fcl_LIBRARIES}
 )

add_subdirectory(test)
