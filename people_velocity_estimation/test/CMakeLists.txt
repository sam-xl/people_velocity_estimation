find_package(ament_cmake_ros REQUIRED)

# Every test_*.py here, in one ROS domain of its own.
ament_add_ros_isolated_pytest_test(test_people_estimation ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
