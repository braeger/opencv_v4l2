/*
 * opencv_v4l2 - v4l2_helper.h file
 *
 * Copyright (c) 2017-2018, e-con Systems India Pvt. Ltd.  All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

 - Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 - Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 - Neither the name of the copyright holder nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
// Header file for v4l2_helper functions.

#ifndef V4L2_HELPER_H
#define V4L2_HELPER_H

#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C" {
#endif

enum io_method {
	IO_METHOD_READ = 1,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR
};

/**
 * FIXME: All the state for the library is maintained via global variables.
 * So, it's not possible to use this library to access multiple devices
 * simultaneously. This is done to simplify the process of assessment.
 *
 * Hint: To access multiple devices at the same time using this application,
 * the public helper functions can be made to accept a new structure, that holds
 * the global state, as parameter.
 */

/*
 * All functions return 0 on success and a negative value in case of failure.
 */

int helper_init_cam(const char* devname, unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth);

int helper_get_cam_frame(unsigned char** pointer_to_cam_data, int *size);
int helper_get_cam_frame_with_framebuf(unsigned char** pointer_to_cam_data, int *size, struct v4l2_buffer* buf);

int helper_release_cam_frame();

int helper_deinit_cam();

//int helper_change_cam_res(unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth);

int helper_ctrl(unsigned int id, int64_t val);

//int helper_queryctrl(unsigned int,struct v4l2_queryctrl* );

#ifdef __cplusplus
}
#endif

#endif
