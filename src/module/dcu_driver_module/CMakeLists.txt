# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Find packages
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/test/*_test.cc)
list(REMOVE_ITEM src ${test_files})

# Add SDK
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/xyber_controller)

# Add target
add_library(${CUR_TARGET_NAME} STATIC)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common/include>
)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC xyber_controller
         aimrt::interface::aimrt_module_cpp_interface
         aimrt::interface::aimrt_module_ros2_interface
         my_ros2_proto::my_ros2_proto__rosidl_generator_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_fastrtps_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_introspection_cpp
         my_ros2_proto::my_ros2_proto__rosidl_generator_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_fastrtps_cpp
         my_ros2_proto::my_ros2_proto__rosidl_typesupport_introspection_cpp
         std_msgs::std_msgs__rosidl_generator_cpp
         std_msgs::std_msgs__rosidl_typesupport_cpp
         std_msgs::std_msgs__rosidl_typesupport_fastrtps_cpp
         std_msgs::std_msgs__rosidl_typesupport_introspection_cpp
         sensor_msgs::sensor_msgs__rosidl_generator_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_cpp
         geometry_msgs::geometry_msgs__rosidl_generator_cpp
         geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
         geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_cpp
         geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp
         common::common 
)

# Set test of target
if(XYBER_X1_INFER_BUILD_TESTS AND test_files)
  add_gtest_target(TEST_TARGET ${CUR_TARGET_NAME} TEST_SRC ${test_files})
endif()

# Install config files
install(
  DIRECTORY cfg/
  DESTINATION bin/cfg/${CUR_DIR}
  USE_SOURCE_PERMISSIONS
)

# install config files to build dir
install(
  DIRECTORY cfg/
  DESTINATION ${CMAKE_BINARY_DIR}/cfg/${CUR_DIR}
  USE_SOURCE_PERMISSIONS
)