cmake_minimum_required(VERSION 3.8)
project(people_velocity_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core video)
find_package(people_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(riskam_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_component
  SHARED
  src/confirmation_filter.cpp
  src/debug_visualizer.cpp
  src/depth_reader.cpp
  src/people_estimation_node.cpp
  src/people_map_transform.cpp
  src/person_tracker.cpp
  src/plausibility_filter.cpp
  src/track_coaster.cpp
  src/uncertainty_model.cpp
)
target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
  cv_bridge::cv_bridge
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  image_geometry::image_geometry
  message_filters::message_filters
  opencv_core
  opencv_video
  ${people_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${riskam_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)

# Provides both the composable component and a standalone
# `people_estimation_node` executable.
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "people_velocity_estimation::PeopleEstimationNode"
  EXECUTABLE people_estimation_node
)

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Bench tooling: dummy_tf.py fakes the whole TF tree a localisation stack and a
# robot description normally provide; bag_camera_shim.py fills in a colour
# camera_info a replay may not carry.
install(PROGRAMS scripts/bag_camera_shim.py scripts/dummy_tf.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  cv_bridge
  Eigen3
  geometry_msgs
  image_geometry
  message_filters
  OpenCV
  people_msgs
  rclcpp
  rclcpp_components
  riskam_msgs
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
