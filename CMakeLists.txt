cmake_minimum_required(VERSION 3.8)
project(foray_auto_aim)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/filter_friendly.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_filter_friendly test/test_filter_friendly.cpp)
    target_link_libraries(test_filter_friendly ${PROJECT_NAME})
endif()

ament_package()
