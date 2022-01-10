

#interface target for CImg
add_library(CImg INTERFACE)
target_include_directories(CImg INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps>)


add_executable(v4l2_webcam v4l2_webcam.cpp)
target_link_libraries(v4l2_webcam PRIVATE v4l2_helper CImg)
