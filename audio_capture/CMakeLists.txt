cmake_minimum_required(VERSION 3.8)
project(audio_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Python
install(PROGRAMS
  audio_capture/audio_capturer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME audio_capturer
)

install(PROGRAMS
  audio_capture/audio_player.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME audio_player
)

ament_package()
