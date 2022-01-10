cmake_minimum_required (VERSION 3.10)

project(v4l2helper VERSION 0.2.0) #Version vars are now autoset by project command.

add_library (v4l2helper
	src/v4l2helper.c
	include/v4l2helper.h
)

target_include_directories (v4l2helper PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties (
	v4l2helper PROPERTIES
	PUBLIC_HEADER include/v4l2helper.h
)

install (TARGETS v4l2helper
	LIBRARY
		COMPONENT lib
		NAMELINK_COMPONENT dev
	PUBLIC_HEADER
		COMPONENT dev
)

if(v4l2_helper_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	add_subdirectory(example)
endif()
