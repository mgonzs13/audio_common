cmake_minimum_required(VERSION 3.10)
project(audio_common_dsp)

find_package(ament_cmake REQUIRED)
find_package(audio_common_msgs REQUIRED)

add_library(audio_common_dsp STATIC
  src/audio_converter.cpp
  src/multi_channel_resampler.cpp
  src/resampler.cpp
)
target_include_directories(audio_common_dsp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(audio_common_dsp PUBLIC ${audio_common_msgs_TARGETS})

install(TARGETS audio_common_dsp
  EXPORT export_audio_common_dsp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT export_audio_common_dsp
  NAMESPACE audio_common_dsp::
  DESTINATION share/audio_common_dsp/cmake
)
ament_export_targets(export_audio_common_dsp HAS_LIBRARY_TARGET)
ament_export_dependencies(audio_common_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(audio_converter_test test/audio_converter_test.cpp)
  target_link_libraries(audio_converter_test audio_common_dsp)
  ament_add_gtest(resampler_test test/resampler_test.cpp)
  target_link_libraries(resampler_test audio_common_dsp)
endif()

ament_package()
