cmake_minimum_required(VERSION 3.5)
project(weedbot_msgs)

# find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# message dependency packages (must be found before rosidl_generate_interfaces)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
set(weedbot_msgs_MEMBER_OF_GROUPS "rosidl_interface_packages" CACHE INTERNAL "")

# --- FORCE the member-of-group variable so rosidl_generate_interfaces accepts this package
# Some CMake/rosidl setups don't propagate <member_of_group> correctly into the
# generated package.cmake in certain environments; setting the variable here
# ensures the generator recognizes this as a rosidl interface package.
set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

# list message files
set(msg_files
  "msg/MicroSensorPacket.msg"
  "msg/MotionCmd.msg"
  "msg/ToolCmd.msg"
  "msg/SafetyCmd.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs sensor_msgs
)

ament_package()