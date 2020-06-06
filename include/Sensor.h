#if ENABLE_KINECT_V2
#include <kinect.h>
#endif
#include <windows.h>
#include <wrl/client.h>
#include <iostream>

class RGBD_Sensor {
public:
	void openInitDevice();

	void getImageSizes(int& color_width_,int& color_height_,int& depth_width_,int& depth_height_){
		color_width_ = color_width; color_height_ = color_height;
		depth_width_ = depth_width; depth_height_ = depth_height;
	};

	int color_width, color_height, depth_width, depth_height;
#if ENABLE_KINECT_V2
	Microsoft::WRL::ComPtr<IKinectSensor> iKinect;

#endif


};