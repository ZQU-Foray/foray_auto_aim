cmake_minimum_required(VERSION 3.8)
project(foray_auto_aim)

# C++17（与参考实现 sp_vision_25 保持一致）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# 源码与测试尚未落地：新增模块时在此追加构建与测试目标，并同步更新 tree.md
#   add_library(${PROJECT_NAME} SHARED src/<module>.cpp)
#   target_include_directories(${PROJECT_NAME} PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
#   if(BUILD_TESTING)
#     find_package(ament_cmake_gtest REQUIRED)
#     ament_add_gtest(test_<module> test/test_<module>.cpp)
#     target_link_libraries(test_<module> ${PROJECT_NAME})
#   endif()

ament_package()
