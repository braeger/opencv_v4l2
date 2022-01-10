#include "v4l2helper.h"
#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>


#include "CImg.h"
using namespace cimg_library;
using namespace std;

unsigned int GetTickCount()
{
	struct timeval tv;
	if(gettimeofday(&tv, NULL) != 0)
		return 0;

	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

int main(int argc, char **argv)
{
	std::string videodev = "/dev/video0";
	unsigned int pix_format = V4L2_PIX_FMT_YUYV; // Or: V4L2_PIX_FMT_MJPEG.  see all available with: v4l2-ctl --list-formats-ext
	// TODO: YUYV conversion for display issue: let's just write as an image file just to confirm.  
	// But for live display adopt to v4l2_helper from : https://gist.github.com/MacDue/36199c3f3ca04bd9fd40a1bc2067ef72
	size_t width = 1280, height = 720; // default for YUYV;
	enum io_method IO_method = IO_METHOD_MMAP; // Also: IO_METHOD_USERPTR and IO_METHOD_MMAP

	CImg<unsigned char> visu(width,height,1,4,0);
	CImgDisplay main_disp(visu,"preview");

	v4l2helper_capture_t* cam=v4l2helper_cam_open(videodev.c_str(), width, height, pix_format, IO_method,0);

	if (!cam)
	{
		throw std::runtime_error("Could not init cam with parameters.");
	}

	unsigned int start = GetTickCount(), end, fps = 0;
	unsigned char* ptr_data;
	main_disp.show();
	while(!main_disp.is_closed())
	{
		main_disp.wait(33);
		int bytes_used;
		v4l2helper_frame_t* frame;
		if (v4l2helper_cam_get_frame(cam,&frame) < 0)
		{
			break;
		}
		v4l2helper_frame_get_data(frame,&ptr_data,&bytes_used);
		cout << "BYTES USED: " << bytes_used << std::endl;


		visu.assign(ptr_data,3,width,height,1);
		if(visu.empty())
		{
			cout << "Img load failed" << endl;
			break;
		}
		visu.permute_axes("yzcx");
//		visu.save("frame.jpg");
		main_disp.display(visu);

		// Must be called for every frame
		if (v4l2helper_frame_release(frame) < 0)
		{
			break;
		}

		// Get fps:
		fps++;
		end = GetTickCount();
		if ((end - start) >= 1000) {
			cout << "fps = " << fps << endl ;
			fps = 0;
			start = end;
		}
	}
	main_disp.close();
	if (v4l2helper_cam_close(cam) < 0)
	{
		throw std::runtime_error("Could not close connection to camera.");
	}

	return 0;
}
