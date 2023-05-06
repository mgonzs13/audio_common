cmake_minimum_required(VERSION 3.8)
project(audio_common_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/AudioInfo.msg"
  "msg/Audio.msg"
  "msg/AudioStamped.msg"
  DEPENDENCIES std_msgs
)

ament_package()
