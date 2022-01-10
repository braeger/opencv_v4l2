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

#ifndef V4L2_v4l2helper_H
#define V4L2_v4l2helper_H

#include <linux/videodev2.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum io_method {
	IO_METHOD_READ = 1,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR
};

typedef struct _v4l2helper_cam_s v4l2helper_cam_t;


/*
 * All functions except the create return 0 on success and a negative value in case of failure.
 */

v4l2helper_cam_t* v4l2helper_init_cam(const char* devname, unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth);

int v4l2helper_get_cam_frame(v4l2helper_cam_t* cam,unsigned char** pointer_to_cam_data, int *size);
int v4l2helper_get_cam_frame_with_framebuf(v4l2helper_cam_t* cam,unsigned char** pointer_to_cam_data, int *size, struct v4l2_buffer* buf);

int v4l2helper_release_cam_frame(v4l2helper_cam_t* cam);

int v4l2helper_deinit_cam(v4l2helper_cam_t* cam);

//int v4l2helper_change_cam_res(unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth);

int v4l2helper_ctrl(v4l2helper_cam_t* cam,unsigned int id, int64_t val);

//int v4l2helper_queryctrl(unsigned int,struct v4l2_queryctrl* );

#ifdef __cplusplus
}
#endif

#endif