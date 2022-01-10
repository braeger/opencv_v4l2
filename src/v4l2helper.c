/*
 * opencv_v4l2 - v4l2_helper.c file
 *
 * Copyright (c) 2017-2018, e-con Systems India Pvt. Ltd.  All rights reserved.
 *

Redistribution and use in source and binary forms, with or without modification,
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include "v4l2helper.h"
#include <stdbool.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define ERR -128 /* TODO: errors should be more specific */

struct buffer {
	void   *start;
	size_t  length;
};


/*
struct _v4l2helper_frame_vtable{
	int (*init)(v4l2helper_frame_t* frame,int fd,uint32_t caps,unsigned int index);
	int
};

struct _v4l2helper_capture_vtable{

};*/

struct _v4l2helper_frame_s{
	struct v4l2_buffer frame_buf;
	struct buffer buf;
	const v4l2helper_capture_t* const cam; //todo: frames can be hosted in the capture queue OR the output queue.
	bool is_released;
};
struct _v4l2helper_capture_s
{
	struct v4l2_capability cap;
	enum io_method   io;
	int              fd;
	v4l2helper_frame_t    *frames;
	unsigned int     n_frames;	//the number of actual frames in the frame queue
	bool is_initialised;
};



/**
 * Start of static (internal) helper functions
 */
static int xioctl(int fh, unsigned long request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int set_io_method(v4l2helper_capture_t* cam,enum io_method io_meth)
{
	switch (io_meth)
	{
		case IO_METHOD_READ:
		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			cam->io = io_meth;
			return 0;
		default:
			fprintf(stderr, "Invalid I/O method\n");
			return ERR;
	}
}

static int stop_capturing(v4l2helper_capture_t* cam)
{
	enum v4l2_buf_type type;
	struct v4l2_requestbuffers req;

	switch (cam->io) {
		case IO_METHOD_READ:
			/* Nothing to do. */
			break;

		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (-1 == xioctl(cam->fd, VIDIOC_STREAMOFF, &type))
			{
				fprintf(stderr, "Error occurred when streaming off\n");
				return ERR;
			}
			break;
	}

	CLEAR(req);

	/*
	 * Request to clear the buffers. This helps with changing resolution.
	 */
	req.count = 0;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(cam->fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "The device does not support "
					"memory mapping\n");
		}
		return ERR;
	}

	return 0;
}

static int start_capturing(v4l2helper_capture_t* cam)
{
	unsigned int i;
	enum v4l2_buf_type type;

	switch (cam->io) {
		case IO_METHOD_READ:
			/* Nothing to do. */
			break;

		case IO_METHOD_MMAP:
			for (i = 0; i < cam->n_frames; ++i) {
				struct v4l2_buffer buf;

				CLEAR(buf);
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_MMAP;
				buf.index = i;

				if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf))
				{
					fprintf(stderr, "Error occurred when queueing buffer\n");
					return ERR;
				}
			}
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (-1 == xioctl(cam->fd, VIDIOC_STREAMON, &type))
			{
				fprintf(stderr, "Error occurred when turning on stream\n");
				return ERR;
			}
			break;

		case IO_METHOD_USERPTR:
			for (i = 0; i < cam->n_frames; ++i) {
				struct v4l2_buffer buf;

				CLEAR(buf);
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_USERPTR;
				buf.index = i;
				buf.m.userptr = (unsigned long)(cam->buffers[i].start);
				buf.length = cam->buffers[i].length;

				if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf))
				{
					fprintf(stderr, "Error occurred when queueing buffer\n");
					return ERR;
				}
			}
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (-1 == xioctl(cam->fd, VIDIOC_STREAMON, &type))
			{
				fprintf(stderr, "Error when turning on stream\n");
				return ERR;
			}
			break;
	}

	return 0;
}

static int uninit_device(v4l2helper_capture_t* cam)
{
	unsigned int i;
	int ret = 0;

	switch (cam->io) {
		case IO_METHOD_READ:
			free(cam->buffers[0].start);
			break;

		case IO_METHOD_MMAP:
			for (i = 0; i < cam->n_frames; ++i)
				if (-1 == munmap(cam->buffers[i].start, cam->buffers[i].length))
					ret = ERR;
			break;

		case IO_METHOD_USERPTR:
			for (i = 0; i < cam->n_frames; ++i)
				free(cam->buffers[i].start);
			break;
	}

	free(cam->buffers);
	return ret;
}

static int init_read(v4l2helper_capture_t* cam,unsigned int buffer_size,unsigned int num_requested_buffers)
{
	cam->buffers = (v4l2helper_frame_t *) calloc(1, sizeof(v4l2helper_frame_t));

	if (!cam->buffers) {
		fprintf(stderr, "Out of memory\n");
		return ERR;
	}
	cam->n_frames = 1;

	cam->buffers[0].length = buffer_size;
	cam->buffers[0].start = malloc(buffer_size);

	if (!cam->buffers[0].start) {
		fprintf(stderr, "Out of memory\n");
		return ERR;
	}

	return 0;
}

static int init_mmap_unwind_errors(v4l2helper_capture_t* cam)
{
	unsigned int curr_buf_to_free;
	for (curr_buf_to_free = 0;
		curr_buf_to_free < cam->n_frames;
		curr_buf_to_free++)
	{
		if (
			munmap(cam->buffers[curr_buf_to_free].start,
			cam->buffers[curr_buf_to_free].length) != 0
		)
		{
			/*
			 * Errors ignored as mapping itself
			 * failed for a buffer
			 */
		}
	}
	free(cam->buffers);
	return ERR;
}

static int init_mmap(v4l2helper_capture_t* cam,unsigned int num_requested_buffs)
{
	struct v4l2_requestbuffers req;
	int ret = 0;

	CLEAR(req);

	req.count = num_requested_buffs;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(cam->fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "The device does not support "
					"memory mapping\n");
		}
		return ERR;
	}

	if (req.count < 1) {
		fprintf(stderr, "Insufficient memory to allocate "
				"cam->buffers");
		return ERR;
	}

	cam->buffers = (v4l2helper_frame_t *) calloc(req.count, sizeof(v4l2helper_frame_t));

	if (!cam->buffers) {
		fprintf(stderr, "Out of memory\n");
		return ERR;
	}

	for (cam->n_frames = 0; cam->n_frames < req.count; ++cam->n_frames) {
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = cam->n_frames;

		if (-1 == xioctl(cam->fd, VIDIOC_QUERYBUF, &buf))
		{
			fprintf(stderr, "Error occurred when querying buffer\n");
			return init_mmap_unwind_errors(cam);
		}

		cam->buffers[cam->n_frames].length = buf.length;
		cam->buffers[cam->n_frames].start =
			mmap(NULL /* start anywhere */,
					buf.length,
					PROT_READ | PROT_WRITE /* required */,
					MAP_SHARED /* recommended */,
					cam->fd, buf.m.offset);

		if (MAP_FAILED == cam->buffers[cam->n_frames].start) {
			fprintf(stderr, "Error occurred when mapping memory\n");
			return init_mmap_unwind_errors(cam);
		}
	}

	return ret;
}

static int init_userp(v4l2helper_capture_t* cam,unsigned int buffer_size,unsigned int num_requested_bufs)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count  = num_requested_bufs;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(cam->fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "The device does not "
					"support user pointer i/o\n");
		}
		return ERR;
	}

	cam->buffers = (v4l2helper_frame_t *) calloc(req.count, sizeof(v4l2helper_frame_t));

	if (!cam->buffers) {
		fprintf(stderr, "Out of memory\n");
		return ERR;
	}

	for (cam->n_frames = 0; cam->n_frames < req.count; ++cam->n_frames) {
		cam->buffers[cam->n_frames].length = buffer_size;
		if(posix_memalign(&cam->buffers[cam->n_frames].start,getpagesize(),buffer_size) != 0)
		{
			/*
			 * This happens only in case of ENOMEM
			 */
			unsigned int curr_buf_to_free;
			for (curr_buf_to_free = 0;
				curr_buf_to_free < cam->n_frames;
				curr_buf_to_free++
			)
			{
				free(cam->buffers[curr_buf_to_free].start);
			}
			free(cam->buffers);
			fprintf(stderr, "Error occurred when allocating memory for cam->buffers\n");
			return ERR;
		}
	}

	return 0;
}

static int init_device(v4l2helper_capture_t* cam,unsigned int width, unsigned int height, unsigned int format,unsigned int num_requested_buffers)
{
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == xioctl(cam->fd, VIDIOC_QUERYCAP, &cam->cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "Given device is no V4L2 device\n");
		}
		fprintf(stderr, "Error occurred when querying capabilities\n");
		return ERR;
	}

	if (!(cam->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "Given device is no video capture device\n");
		return ERR;
	}

	switch (cam->io) {
		case IO_METHOD_READ:
			if (!(cam->cap.capabilities & V4L2_CAP_READWRITE)) {
				fprintf(stderr, "Given device does not "
						"support read i/o\n");
				return ERR;
			}
			break;

		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			if (!(cam->cap.capabilities & V4L2_CAP_STREAMING)) {
				fprintf(stderr, "Given device does not "
						"support streaming i/o\n");
				return ERR;
			}
			break;
	}


	/* Select video input, video standard and tune here. */

	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(cam->fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(cam->fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
				case EINVAL:
					/* Cropping not supported. */
					break;
				default:
					/* Errors ignored. */
					break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = format;
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

	if (-1 == xioctl(cam->fd, VIDIOC_S_FMT, &fmt))
	{
		fprintf(stderr, "Error occurred when trying to set format\n");
		return ERR;
	}

	/* Note VIDIOC_S_FMT may change width and height. */

	fprintf(stderr,"pixfmt = %c %c %c %c \n", (fmt.fmt.pix.pixelformat & 0x000000ff) , (fmt.fmt.pix.pixelformat & 0x0000ff00) >>8 , (fmt.fmt.pix.pixelformat & 0x00ff0000) >>16, (fmt.fmt.pix.pixelformat & 0xff000000) >>24 );
	fprintf(stderr,"width = %d height = %d\n",fmt.fmt.pix.width,fmt.fmt.pix.height);

	if (
		fmt.fmt.pix.width != width ||
		fmt.fmt.pix.height != height ||
		fmt.fmt.pix.pixelformat != format
	)
	{
		fprintf(stderr, "Warning: The current format does not match requested format/resolution!\n");
		return ERR;
	}

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	switch (cam->io) {
		case IO_METHOD_READ:
			return init_read(cam,fmt.fmt.pix.sizeimage,num_requested_buffers);
			break;

		case IO_METHOD_MMAP:
			return init_mmap(cam,num_requested_buffers);
			break;

		case IO_METHOD_USERPTR:
			return init_userp(cam,fmt.fmt.pix.sizeimage,num_requested_buffers);
			break;
	}

	return 0;
}

static int close_device(v4l2helper_capture_t* cam)
{
	if (-1 == close(cam->fd))
	{
		fprintf(stderr, "Error occurred when closing device\n");
		return ERR;
	}

	cam->fd = -1;

	return 0;
}

static int open_device(v4l2helper_capture_t* cam,const char *dev_name)
{
	struct stat st;

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		return ERR;
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		return ERR;
	}

	cam->fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == cam->fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		return ERR;
	}

	return cam->fd;
}
/**
 * End of static (internal) helper functions
 */


/**
 * Start of public helper functions
 */
v4l2helper_capture_t* v4l2helper_cam_open(const char* devname, unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth,unsigned int num_requested_buffers)
{

	if(num_requested_buffers == 0)
	{
		num_requested_buffers=4;
	}
	v4l2helper_capture_t* cam=(v4l2helper_capture_t*)malloc(sizeof(v4l2helper_capture_t));
	cam->is_initialised=0;
	//cam->is_released=1;
	cam->fd = -1;

	if(
		set_io_method(cam,io_meth) < 0 ||
		open_device(cam,devname) < 0 ||
		init_device(cam,width,height,format,num_requested_buffers) < 0 ||
		start_capturing(cam) < 0
	)
	{
		fprintf(stderr, "Error occurred when initialising camera\n");
		return NULL;
	}

	cam->is_initialised = 1;
	return cam;
}

int v4l2helper_cam_close(v4l2helper_capture_t* cam)
{
	if (cam == NULL || !cam->is_initialised)
	{
		fprintf(stderr, "Error: trying to de-initialise without initialising camera\n");
		return ERR;
	}

	/*
	 * It's better to turn off cam->is_initialised even if the
	 * de-initialisation fails as it shouldn't have affect
	 * re-initialisation a lot.
	 */
	cam->is_initialised = 0;

	if(
		stop_capturing(cam) < 0 ||
		uninit_device(cam) < 0 ||
		close_device(cam) < 0
	)
	{
		fprintf(stderr, "Error occurred when de-initialising camera\n");
		return ERR;
	}
	if(cam!=NULL)
	{
		free(cam);
	}

	return 0;
}

int v4l2helper_cam_get_frame(v4l2helper_capture_t* cam,unsigned char **pointer_to_cam_data, int *size)
{
	return v4l2helper_cam_get_frame_with_framebuf(cam,pointer_to_cam_data, size, NULL);
}

int v4l2helper_cam_get_frame_with_framebuf(v4l2helper_capture_t* cam,unsigned char **pointer_to_cam_data, int *size, struct v4l2_buffer* buf)
{
	static const unsigned int max_timeout_retries = 10;
	unsigned int timeout_retries = 0;

	if (!cam->is_initialised)
	{
		fprintf (stderr, "Error: trying to get frame without successfully initialising camera\n");
		return ERR;
	}

	/* TODO, should this error or stall? I think it should definitely stall instead (timeout)
	if (!cam->is_released)
	{
		fprintf (stderr, "Error: trying to get another frame without releasing already obtained frame\n");
		return ERR;
	}*/

	for (;;) {
		fd_set fds;
		struct timeval tv;
		int r;

		FD_ZERO(&fds);
		FD_SET(cam->fd, &fds);

		/* Timeout. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		r = select(cam->fd + 1, &fds, NULL, NULL, &tv);

		if (-1 == r) {
			if (EINTR == errno)
				continue;
		}

		if (0 == r) {
			fprintf(stderr, "select timeout\n");
			timeout_retries++;

			if (timeout_retries == max_timeout_retries)
			{
				fprintf(stderr, "Could not get frame after multiple retries\n");
				return ERR;
			}
		}

		CLEAR(cam->frame_buf);
		cam->frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl(cam->fd, VIDIOC_DQBUF, &cam->frame_buf)) {
			switch (errno) {
				case EAGAIN:
					continue;

				case EIO:
					/* Could ignore EIO, see spec. */

					/* fall through */

				default:
					continue;
			}
		}
		if (buf != NULL) {
			*buf = cam->frame_buf;
		}
		*pointer_to_cam_data = (unsigned char*) cam->buffers[cam->frame_buf.index].start;
		*size = cam->frame_buf.bytesused;
		break;
		/* EAGAIN - continue select loop. */
	}

	//cam->is_released = 0;
	return 0;
}

int v4l2helper_frame_release(v4l2helper_frame_t* frame)
{
	v4l2helper_capture_t* cam=frame->cam;
	if (!cam->is_initialised)
	{
		fprintf (stderr, "Error: trying to release frame without successfully initialising camera\n");
		return ERR;
	}

	if (frame->is_released)
	{
		fprintf (stderr, "Error: trying to release already released frame\n");
		return ERR;
	}

	if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &cam->frame_buf))
	{
		fprintf(stderr, "Error occurred when queueing frame for re-capture\n");
		return ERR;
	}

	/*
	 * We assume the frame hasn't been released if an error occurred as
	 * we couldn't queue the frame for streaming.
	 *
	 * Assuming it to be released in case an error occurs causes issues
	 * such as the loss of a buffer, etc.
	 */
	frame->is_released = 1;
	return 0;
}

/**
 * End of public helper functions
 */

/**
 * Untested for misusages
 */


/*
int v4l2helper_change_cam_res(unsigned int width, unsigned int height, unsigned int format, enum io_method io_meth)
{
	if (!cam->is_initialised)
	{
		fprintf(stderr, "Error: trying to de-initialise without initialising camera\n");
		return ERR;
	}

	if (
		stop_capturing() < 0 ||
		uninit_device() < 0
	)
	{
		fprintf(stderr, "Error occurred when ude-initializing device to change camera resolution\n");
		return ERR;
	}

	cam->is_initialised = 0;

	if (
		set_io_method(io_meth) < 0 ||
		init_device(width,height,format) < 0 ||
		start_capturing() < 0
	)
	{
		fprintf(stderr, "Error occurred when changing camera resolution\n");
		return ERR;
	}

	cam->is_initialised = 1;

	return 0;
}

int v4l2helper_queryctrl(unsigned int id,struct v4l2_queryctrl* qctrl)
{
	if (!cam->is_initialised)
	{
		fprintf(stderr, "Error: trying to query control without initialising camera\n");
		return ERR;
	}

	qctrl->id = id;
	if (-1 == xioctl(cam->fd, VIDIOC_QUERYCTRL, qctrl)) {
		fprintf(stderr, "Error QUERYCTRL\n");
		return -1;
	}

	return 0;
}

int v4l2helper_ctrl(unsigned int id, int flag,int* value)
{
	if (!cam->is_initialised)
	{
		fprintf(stderr, "Error: trying to get/set control without initialising camera\n");
		return ERR;
	}

	unsigned int ioctl_num = 0;
	struct v4l2_control ctrl;
	ctrl.id = id;

	if (flag == GET) {
		ioctl_num = VIDIOC_G_CTRL;
	} else if (flag == SET) {
		ioctl_num = VIDIOC_S_CTRL;
		ctrl.value = *value;
	}
	if (-1 == xioctl(cam->fd, ioctl_num, &ctrl)) {
		fprintf(stderr,"Error GET/SET\n");
		return -1;
	}

	if (flag == GET) {
		*value = ctrl.value;
	}

	return 0;
}
*/
