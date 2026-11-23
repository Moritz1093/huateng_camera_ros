cmake_minimum_required(VERSION 3.10)
project(huateng_camera_ros)

# One package for ROS 1 (catkin) and ROS 2 (ament_cmake). ROS_VERSION is set by the ROS setup script.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall)

# --- Huateng / MindVision SDK (install with scripts/install_sdk.sh) ---
if(DEFINED ENV{HUATENG_SDK_DIR})
  set(HUATENG_SDK_DIR_DEFAULT "$ENV{HUATENG_SDK_DIR}")
else()
  set(HUATENG_SDK_DIR_DEFAULT "/opt/huatengvision_SDK")
endif()
set(HUATENG_SDK_DIR "${HUATENG_SDK_DIR_DEFAULT}" CACHE PATH "Directory of the Huateng SDK (include/, lib/)")
find_path(HUATENG_SDK_INCLUDE_DIR CameraApi.h HINTS "${HUATENG_SDK_DIR}/include" /usr/include)
find_library(HUATENG_SDK_LIBRARY NAMES MVSDK mvsdk HINTS "${HUATENG_SDK_DIR}/lib" /lib /usr/lib)
if(NOT HUATENG_SDK_INCLUDE_DIR OR NOT HUATENG_SDK_LIBRARY)
  message(FATAL_ERROR "Huateng SDK not found in ${HUATENG_SDK_DIR}. Install it with: sudo scripts/install_sdk.sh "
                      "(or set HUATENG_SDK_DIR)")
endif()
message(STATUS "Huateng SDK: ${HUATENG_SDK_LIBRARY}")

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Threads REQUIRED)

# --- Core without ROS (shared by both wrappers) ---
add_library(huateng_camera_core STATIC src/config.cpp src/driver.cpp)
set_target_properties(huateng_camera_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(huateng_camera_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(huateng_camera_core PRIVATE ${HUATENG_SDK_INCLUDE_DIR})
target_link_libraries(huateng_camera_core PUBLIC ${OpenCV_LIBS} Threads::Threads ${HUATENG_SDK_LIBRARY})

# Simulation test of the time stamp logic (no camera, no ROS): build with "make test_trigger_clock"
add_executable(test_trigger_clock EXCLUDE_FROM_ALL test/test_trigger_clock.cpp)
target_include_directories(test_trigger_clock PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================================== ROS 2 =====================================
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(camera_info_manager REQUIRED)

  add_executable(camera_driver src/ros2_node.cpp)
  target_link_libraries(camera_driver huateng_camera_core)
  if(TARGET camera_info_manager::camera_info_manager)
    target_link_libraries(camera_driver rclcpp::rclcpp ${sensor_msgs_TARGETS}
                          camera_info_manager::camera_info_manager)
  else()
    ament_target_dependencies(camera_driver rclcpp sensor_msgs camera_info_manager)
  endif()

  install(TARGETS camera_driver DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  install(FILES launch/camera.launch.py DESTINATION share/${PROJECT_NAME}/launch)
  ament_package()
else()
  # ===================================== ROS 1 =====================================
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs camera_info_manager)
  catkin_package(CATKIN_DEPENDS roscpp sensor_msgs camera_info_manager)

  add_executable(camera_driver src/ros1_node.cpp)
  target_include_directories(camera_driver PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(camera_driver huateng_camera_core ${catkin_LIBRARIES})
  add_dependencies(camera_driver ${catkin_EXPORTED_TARGETS})

  install(TARGETS camera_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  install(FILES launch/camera.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
endif()
