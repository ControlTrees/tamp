cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate dependencies
#################################

find_package(Boost COMPONENTS system filesystem REQUIRED)

#################################
# Locate MLR libs
#################################

find_path( MLR_LIBRARIES_DIR
           NAMES libCore.so
           PATHS "/home/camille/Dev/Phd/mlr_sparse/rai/rai" )

#################################
# Locate AMINO libs
#################################

find_path( AMINO_INLCUDE_DIR
           NAMES amino.h
           PATHS "/usr/local/include/amino-1.0/" )

find_path( AMINO_LIBRARIES_DIR
           NAMES amino.so
           PATHS "/usr/local/lib" )


include_directories( ${MLR_LIBRARIES_DIR} ${AMINO_INLCUDE_DIR} "external")
link_directories(    ${MLR_LIBRARIES_DIR} ${AMINO_LIBRARIES_DIR} )

#################################
# Define executable
#################################

file(GLOB_RECURSE PROJECT_SRC "*.cpp" "external/*.c")

add_executable(${project_name} ${PROJECT_SRC})
target_link_libraries(
 ${project_name}
 ${Boost_LIBRARIES}
 Core
 Geo
 Gui
 Kin
 KOMO
 MathUtility
 MotionPlanning
 amino
 )
