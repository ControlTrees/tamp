cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})

# list all tests
file(GLOB_RECURSE tests *.cpp)

#find Boost
find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

# specify target for each test
foreach(test ${tests})
    get_filename_component(test_name ${test} NAME_WE)
    message(STATUS ${test_name} )

    add_executable(${test_name} ${test})
    target_link_libraries(${test_name}
                          TaskPlanning
                          ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                          -lboost_system
                          )
endforeach()

# copy directory data
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/data" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
