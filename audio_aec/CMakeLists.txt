cmake_minimum_required(VERSION 3.10)
project(audio_aec)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(audio_common_dsp REQUIRED)

find_package(PkgConfig REQUIRED)
# The 0.3.1 high-suppression AEC is the default because it is the best tested
# configuration for this pipeline. Systems that only provide the 1.x AEC3 API
# still build automatically. Set AUDIO_AEC_USE_AEC3=ON to prefer AEC3 when both
# libraries are installed.
option(AUDIO_AEC_USE_AEC3 "Prefer WebRTC AudioProcessing 1.x (AEC3)" OFF)

# pkg_check_modules must run on every configure: FindPkgConfig does not cache
# WEBRTC_APM_LINK_LIBRARIES, so guarding these calls with `if(NOT ..._FOUND)`
# leaves the link line empty when reconfiguring an existing build directory.
if(AUDIO_AEC_USE_AEC3)
  pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing-1)
  if(WEBRTC_APM_FOUND)
    set(AUDIO_AEC_WEBRTC_NEW_API TRUE)
  else()
    pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing)
  endif()
else()
  pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing)
  if(NOT WEBRTC_APM_FOUND)
    pkg_check_modules(WEBRTC_APM QUIET webrtc-audio-processing-1)
    if(WEBRTC_APM_FOUND)
      set(AUDIO_AEC_WEBRTC_NEW_API TRUE)
    endif()
  endif()
endif()
if(NOT WEBRTC_APM_FOUND)
  message(FATAL_ERROR
    "WebRTC AudioProcessing not found. Install libwebrtc-audio-processing-dev. "
    "The accepted pkg-config modules are 'webrtc-audio-processing' (0.3.1) and "
    "'webrtc-audio-processing-1' (1.3).")
endif()

add_library(audio_aec_core STATIC
  src/audio_aec/webrtc_aec.cpp
  src/audio_aec/aec_processor.cpp
)
target_include_directories(audio_aec_core PUBLIC include)
target_link_libraries(audio_aec_core PUBLIC
  ${audio_common_msgs_TARGETS}
  audio_common_dsp::audio_common_dsp
)
target_include_directories(audio_aec_core PRIVATE
  ${WEBRTC_APM_INCLUDE_DIRS}
  # The 1.x pkg-config file points at the WebRTC headers but not at the
  # bundled Abseil headers, which live one level above them.
  ${WEBRTC_APM_INCLUDEDIR}
)
target_link_libraries(audio_aec_core PUBLIC ${WEBRTC_APM_LINK_LIBRARIES})
target_compile_options(audio_aec_core PRIVATE ${WEBRTC_APM_CFLAGS_OTHER})
if(AUDIO_AEC_WEBRTC_NEW_API)
  target_compile_definitions(audio_aec_core PRIVATE AUDIO_AEC_WEBRTC_NEW_API=1)
endif()

add_executable(aec_node
  src/audio_aec/aec_node.cpp
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
