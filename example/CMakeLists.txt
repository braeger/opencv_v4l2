

#interface target for CImg
add_library(CImg INTERFACE)
target_include_directories(CImg INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps>)
target_link_options(CImg INTERFACE -pthread)
target_link_libraries(CImg INTERFACE X11)

add_executable(v4l2helper_webcam v4l2helper_webcam.cpp)
target_link_libraries(v4l2helper_webcam PRIVATE v4l2helper CImg)
