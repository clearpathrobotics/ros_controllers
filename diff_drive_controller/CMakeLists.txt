cmake_minimum_required(VERSION 3.5.1)
project(diff_drive_controller)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_generation
    dynamic_reconfigure
    controller_interface
    urdf
    realtime_tools
    tf2
    tf2_msgs
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    trajectory_msgs
    roslint)

find_package(Boost REQUIRED
  COMPONENTS
    timer)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

add_message_files(
  FILES
    DiffDriveControllerState.msg)

generate_messages(
  DEPENDENCIES
    std_msgs
    trajectory_msgs)

generate_dynamic_reconfigure_options(cfg/DiffDriveController.cfg)

# We intentionally only export the non-plugin library. Packages which find_package this package
# should still only be linking against the non-plugin library (for example, to compile a
# derived controller which is itself a plugin).
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    dynamic_reconfigure
    message_runtime
    realtime_tools
    std_msgs
    tf2_msgs
    trajectory_msgs
    urdf
  DEPENDS
    Boost
    EIGEN3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

roslint_cpp()

add_library(${PROJECT_NAME}
  src/diff_drive_controller.cpp
  src/meas_covariance_model.cpp
  src/linear_meas_covariance_model.cpp
  src/quadratic_meas_covariance_model.cpp
  src/integrate_function.cpp
  src/odometry.cpp
  src/speed_limiter.cpp)
# Note that the entry for ${Ceres_LIBRARIES} was removed as we only used headers from that package
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(${PROJECT_NAME}_plugin
  src/diff_drive_controller_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(FILES diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin
    COMPONENTS
      rostest
      std_srvs
      controller_manager
      control_toolbox
      tf2
      tf2_ros)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

  catkin_add_gtest(integrate_functor_test test/integrate_functor_test.cpp)
  target_link_libraries(integrate_functor_test ${PROJECT_NAME})

  catkin_add_gtest(joint_feedback_checker_test test/joint_feedback_checker_test.cpp)
  target_link_libraries(joint_feedback_checker_test ${PROJECT_NAME})

  catkin_add_gtest(odometry_test test/odometry_test.cpp)
  target_link_libraries(odometry_test ${PROJECT_NAME})

  add_executable(diffbot test/diffbot.cpp)
  target_link_libraries(diffbot ${catkin_LIBRARIES})

  add_executable(skidsteerbot test/skidsteerbot.cpp)
  target_link_libraries(skidsteerbot ${catkin_LIBRARIES})

  add_dependencies(tests ${PROJECT_NAME} diffbot skidsteerbot)

  add_rostest_gtest(diff_drive_test
    test/diff_drive_controller.test
    test/diff_drive_test.cpp)
  target_link_libraries(diff_drive_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_nan_test
    test/diff_drive_controller_nan.test
    test/diff_drive_nan_test.cpp)
  target_link_libraries(diff_drive_nan_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_nan_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_limits_test
    test/diff_drive_controller_limits.test
    test/diff_drive_limits_test.cpp)
  target_link_libraries(diff_drive_limits_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_limits_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_timeout_test
    test/diff_drive_timeout.test
    test/diff_drive_timeout_test.cpp)
  target_link_libraries(diff_drive_timeout_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_timeout_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest(test/diff_drive_multipliers.test)
  add_rostest_gtest(diff_drive_fail_test
    test/diff_drive_wrong.test
    test/diff_drive_fail_test.cpp)
  target_link_libraries(diff_drive_fail_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_fail_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_odom_tf_test
    test/diff_drive_odom_tf.test
    test/diff_drive_odom_tf_test.cpp)
  target_link_libraries(diff_drive_odom_tf_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_odom_tf_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_publish_state_test
    test/diff_drive_publish_state.test
    test/diff_drive_publish_state_test.cpp)
  target_link_libraries(diff_drive_publish_state_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_publish_state_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest_gtest(diff_drive_publish_cmd_vel_limited_test
    test/diff_drive_publish_cmd_vel_limited.test
    test/diff_drive_publish_cmd_vel_limited_test.cpp)
  target_link_libraries(diff_drive_publish_cmd_vel_limited_test ${catkin_LIBRARIES})
  add_dependencies(diff_drive_publish_cmd_vel_limited_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest(test/diff_drive_open_loop.test)
  add_rostest(test/diff_drive_velocity_feedback.test)
  add_rostest(test/skid_steer_controller.test)
  add_rostest(test/skid_steer_no_wheels.test)
  add_rostest(test/diff_drive_radius_param.test)
  add_rostest(test/diff_drive_radius_param_fail.test)
  add_rostest(test/diff_drive_separation_param.test)
  add_rostest(test/diff_drive_bad_urdf.test)
endif()
