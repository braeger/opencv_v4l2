cmake_minimum_required (VERSION 3.10)

project(v4l2_helper VERSION 0.2.0) #Version vars are now autoset by project command.

add_library (v4l2_helper
	src/v4l2_helper.c
	include/v4l2_helper.h
)

target_include_directories (v4l2_helper PUBLIC include)

set_target_properties (
	v4l2_helper PROPERTIES
	PUBLIC_HEADER include/v4l2_helper.h
)
install (TARGETS v4l2_helper
	LIBRARY
		COMPONENT lib
		NAMELINK_COMPONENT dev
	PUBLIC_HEADER
		COMPONENT dev
)
