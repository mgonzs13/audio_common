cmake_minimum_required(VERSION 3.8)
project(audio_common)

# find dependencies
find_package(ament_cmake REQUIRED)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  audio_common/audio_capturer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME audio_capturer_node
)

install(PROGRAMS
  audio_common/audio_player_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME audio_player_node
)

install(PROGRAMS
  audio_common/tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tts_node
)

ament_package()
