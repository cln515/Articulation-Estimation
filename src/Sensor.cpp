#include <Sensor.h>


//Open device and obtain information
void RGBD_Sensor::openInitDevice() {
#if ENABLE_KINECT_V2
	HRESULT result = GetDefaultKinectSensor(&iKinect);
	if (FAILED(result) || !iKinect) {
		return;
	}
	result = iKinect->Open();
	if (FAILED(result)) {
		return;
	}
	iKinect->get_ColorFrameSource(&p_color_source);
	iKinect->get_DepthFrameSource(&p_depth_source);
	p_color_source->OpenReader(&p_color_reader);
	p_depth_source->OpenReader(&p_depth_reader);
	p_color_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&color_width);
	p_frame_desc->get_Height(&color_height);
	p_depth_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&depth_width);
	p_frame_desc->get_Height(&depth_height);



	std::cout << "color_width : " << color_width << std::endl;
	std::cout << "color_height : " << color_height << std::endl;
	std::cout << "depth_width : " << depth_width << std::endl;
	std::cout << "depth_height : " << depth_height << std::endl;

#endif

};

void  RGBD_Sensor::getColorImage(cv::Mat& outmat){
	IColorFrame* p_color_frame = nullptr;
	HRESULT color_result = p_color_reader->AcquireLatestFrame(&p_color_frame);
	int color_buffer_size = color_width * color_height * 4 * sizeof(unsigned char);
	if (SUCCEEDED(color_result)) {
		color_result = p_color_frame->CopyConvertedFrameDataToArray(color_buffer_size, reinterpret_cast<BYTE*>(outmat.data), ColorImageFormat_Bgra);
	}


}