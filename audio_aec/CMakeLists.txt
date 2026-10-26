cmake_minimum_required(VERSION 3.10)
project(audio_aec)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(audio_common_dsp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing)
if(NOT WEBRTC_APM_FOUND)
  pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing-1)
  if(WEBRTC_APM_FOUND)
    set(AUDIO_AEC_WEBRTC_NEW_API TRUE)
  endif()
endif()
if(NOT WEBRTC_APM_FOUND)
  message(FATAL_ERROR
    "WebRTC AudioProcessing not found. Install libwebrtc-audio-processing-dev. "
    "The accepted pkg-config modules are 'webrtc-audio-processing' (0.3.1) and "
    "'webrtc-audio-processing-1' (1.3).")
endif()

add_library(audio_aec_core STATIC
  src/webrtc_aec.cpp
  src/aec_processor.cpp
)
target_include_directories(audio_aec_core PUBLIC include)
target_link_libraries(audio_aec_core PUBLIC
  ${audio_common_msgs_TARGETS}
  audio_common_dsp::audio_common_dsp
)
target_include_directories(audio_aec_core PRIVATE ${WEBRTC_APM_INCLUDE_DIRS})
target_link_libraries(audio_aec_core PUBLIC ${WEBRTC_APM_LINK_LIBRARIES})
target_compile_options(audio_aec_core PRIVATE ${WEBRTC_APM_CFLAGS_OTHER})
if(AUDIO_AEC_WEBRTC_NEW_API)
  target_compile_definitions(audio_aec_core PRIVATE AUDIO_AEC_WEBRTC_NEW_API=1)
endif()

add_executable(aec_node
  src/aec_node.cpp
  src/aec_main.cpp
)
target_link_libraries(aec_node
  audio_aec_core
  rclcpp::rclcpp
  ${audio_common_msgs_TARGETS}
)
install(TARGETS aec_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(webrtc_aec_test test/webrtc_aec_test.cpp)
  target_link_libraries(webrtc_aec_test audio_aec_core)
  ament_add_gtest(aec_processor_test test/aec_processor_test.cpp)
  target_link_libraries(aec_processor_test audio_aec_core)
endif()

ament_package()
