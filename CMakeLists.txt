cmake_minimum_required(VERSION 3.8)
project(dualq_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#ROS2
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#MoveIt
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

#Eigen
find_package(Eigen3 REQUIRED)


add_library(${PROJECT_NAME} SHARED src/dualq_kinematics.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    moveit_core
    moveit_msgs
    moveit_ros_planning
    pluginlib
    Eigen3
)

add_executable(dualq_kinematics_demo 
  src/dualq_kinematics_demo.cpp
)

ament_target_dependencies(dualq_kinematics_demo
  rclcpp
  moveit_core
)

target_link_libraries(dualq_kinematics_demo
  ${PROJECT_NAME}
)

install(TARGETS 
  ${PROJECT_NAME}
  dualq_kinematics_demo
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  Eigen3
  moveit_core
  moveit_msgs
  moveit_ros_planning
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  #First DualQuaternionTest and PadenKahan Test
  ament_add_gtest(${PROJECT_NAME}_DualQuaternionTest test/DualQuaternionTest.cpp)
  target_include_directories(${PROJECT_NAME}_DualQuaternionTest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_DualQuaternionTest 
    ${PROJECT_NAME} 
  )

  ament_add_gtest(${PROJECT_NAME}_PadenKahanTest test/PadenKahanTest.cpp)
  target_include_directories(${PROJECT_NAME}_PadenKahanTest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_PadenKahanTest
    ${PROJECT_NAME} 
  )

  #Second ScrewCoordinatesTest, only used in an integration test context only
  ament_add_gtest_executable(${PROJECT_NAME}_ScrewCoordinatesTest  test/ScrewCoordinatesTest.cpp)
  target_include_directories(${PROJECT_NAME}_ScrewCoordinatesTest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_ScrewCoordinatesTest 
    ${PROJECT_NAME} 
  )

  #Integration Tests
  find_package(moveit_resources_panda_description REQUIRED)
  find_package(moveit_resources_panda_moveit_config REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ros_testing REQUIRED)
  # function(add_ros_isolated_launch_test path)
  #   set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  #   add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  # endfunction()
  # add_ros_isolated_launch_test(test/launch/ScrewCoordinates.test.py)
  add_ros_test(test/launch/ScrewCoordinates.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  install(TARGETS ${PROJECT_NAME}_ScrewCoordinatesTest DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME})

endif()

ament_package()
