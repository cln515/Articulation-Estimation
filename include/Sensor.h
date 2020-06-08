#if ENABLE_KINECT_V2
#include <kinect.h>
#endif
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <wrl/client.h>
#include <iostream>
#include <Eigen/Core>

class RGBD_Sensor {
public:
	void openInitDevice();

	void getImageSizes(int& color_width_,int& color_height_,int& depth_width_,int& depth_height_){
		color_width_ = color_width; color_height_ = color_height;
		depth_width_ = depth_width; depth_height_ = depth_height;
	};

	int color_width, color_height, depth_width, depth_height;

	void getColorImage(cv::Mat& outmat);
	void getDepthImage(cv::Mat& outmat);
	bool Depth2ColorPixel(Eigen::Vector2d pix,uint pixValue,Eigen::Vector2d& ret);	
	bool ColorFrame2Camera(Eigen::Vector2d pix, Eigen::Vector3d& ret);

#if ENABLE_KINECT_V2
	Microsoft::WRL::ComPtr<IKinectSensor> iKinect;
	IColorFrameSource* p_color_source;
	IDepthFrameSource* p_depth_source;
	IColorFrameReader* p_color_reader;
	IDepthFrameReader* p_depth_reader;
	IFrameDescription* p_frame_desc;
	ICoordinateMapper* coordinateMapper;
	CameraSpacePoint* csps = NULL;
	void SetColorFrame2Camera(unsigned short* depth_buffer){
		coordinateMapper->MapColorFrameToCameraSpace(depth_width*depth_height, depth_buffer, color_width*color_height, csps);
	}
#endif


};