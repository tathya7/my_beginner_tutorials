cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(integration_test test/test.cpp)
ament_target_dependencies(integration_test rclcpp std_msgs)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker geometry_msgs rclcpp tf2 tf2_ros std_msgs rosbag2_cpp)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(talker "${cpp_typesupport_target}")

target_link_libraries(integration_test
  catch_ros2::catch_ros2_with_node_main
  )

find_package(Doxygen)
if ( DOXYGEN_FOUND )
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  )

install(TARGETS
  integration_test
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # disable these tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

catch_ros2_add_integration_test(ExampleIntegration_TestPython
LAUNCH_FILE test.launch.py  # example in Python
)

ament_package()
