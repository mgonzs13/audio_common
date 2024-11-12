cmake_minimum_required(VERSION 3.8)
project(audio_common)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_library(PORTAUDIO_LIB portaudio REQUIRED)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
)

add_executable(audio_capturer_node
  src/audio_common/audio_capturer_node.cpp
  src/audio_capturer_main.cpp
)
ament_target_dependencies(audio_capturer_node rclcpp audio_common_msgs)
target_link_libraries(audio_capturer_node ${PORTAUDIO_LIB})
install(TARGETS
  audio_capturer_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(audio_player_node
  src/audio_common/audio_player_node.cpp
  src/audio_player_main.cpp
)
ament_target_dependencies(audio_player_node rclcpp audio_common_msgs)
target_link_libraries(audio_player_node ${PORTAUDIO_LIB})
install(TARGETS
  audio_player_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(music_node
  src/audio_common/wave_file.cpp
  src/audio_common/music_node.cpp
  src/music_main.cpp
)
ament_target_dependencies(music_node rclcpp std_srvs audio_common_msgs)
target_link_libraries(music_node ${PORTAUDIO_LIB})
install(TARGETS
  music_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(tts_node
  src/audio_common/wave_file.cpp
  src/audio_common/tts_node.cpp
  src/tts_main.cpp
)
ament_target_dependencies(tts_node rclcpp rclcpp_action audio_common_msgs)
target_link_libraries(tts_node ${PORTAUDIO_LIB})
install(TARGETS
tts_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
