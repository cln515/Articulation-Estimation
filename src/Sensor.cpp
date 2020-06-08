#include <Sensor.h>


//Open device and obtain information
void RGBD_Sensor::openInitDevice() {
#if ENABLE_KINECT_V2
	HRESULT result = GetDefaultKinectSensor(&iKinect);
	if (FAILED(result) || !iKinect) {
		std::cout << "failed to open Kinect V2!!" << std::endl;
		std::exit(-1);
		return;
	}
	result = iKinect->Open();
	if (FAILED(result)) {
		std::cout << "failed to open Kinect V2!!"<<std::endl;
		std::exit(-1);
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

	iKinect->get_CoordinateMapper(&coordinateMapper);

	std::cout << "color_width : " << color_width << std::endl;
	std::cout << "color_height : " << color_height << std::endl;
	std::cout << "depth_width : " << depth_width << std::endl;
	std::cout << "depth_height : " << depth_height << std::endl;


	csps = new CameraSpacePoint[color_width*color_height];
#endif

};

bool  RGBD_Sensor::getColorImage(cv::Mat& outmat){
#if ENABLE_KINECT_V2
	IColorFrame* p_color_frame = nullptr;
	HRESULT color_result = p_color_reader->AcquireLatestFrame(&p_color_frame);
	int color_buffer_size = color_width * color_height * 4 * sizeof(unsigned char);
	if (SUCCEEDED(color_result)) {
		color_result = p_color_frame->CopyConvertedFrameDataToArray(color_buffer_size, reinterpret_cast<BYTE*>(outmat.data), ColorImageFormat_Bgra);
		if (p_color_frame != nullptr) {
			p_color_frame->Release();
		}
		if(SUCCEEDED(color_result)){
			return true;
		}else{return false;}

	}else{
		if (p_color_frame != nullptr) {
			p_color_frame->Release();
		}
	}
	return false;
#endif
}

bool RGBD_Sensor::getDepthImage(cv::Mat& outmat) {
#if ENABLE_KINECT_V2
	IDepthFrame* p_depth_frame = nullptr;
	HRESULT depth_result = p_depth_reader->AcquireLatestFrame(&p_depth_frame);
	int depth_buffer_size = depth_width * depth_height;

	if (SUCCEEDED(depth_result)) {
		p_depth_frame->CopyFrameDataToArray(depth_buffer_size, (UINT16*)(outmat.data));
		if (p_depth_frame != nullptr) {
			p_depth_frame->Release();
		}
		return true;
	}
	if (p_depth_frame != nullptr) {
		p_depth_frame->Release();
	}
	return false;
#endif
}

//
bool RGBD_Sensor::Depth2ColorPixel(Eigen::Vector2d pix,uint pixValue,Eigen::Vector2d& ret){
#if ENABLE_KINECT_V2
//	for (int y = 0; y < depth_buffer_size; y++) {
	DepthSpacePoint dsp; dsp.X = pix(0); dsp.Y = pix(1);
	ColorSpacePoint csp;
	HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, pixValue, &csp);
	if(SUCCEEDED(hr)){
		ret<<csp.X,csp.Y;
		return true;
	}else{
		return false;
	}
//	}

#endif
}

//bool RGBD_Sensor::ColorFrame2Camera(Eigen::Vector2d pix, uint pixValue, Eigen::Vector2d& ret) {
//#if ENABLE_KINECT_V2
//	//	for (int y = 0; y < depth_buffer_size; y++) {
//	DepthSpacePoint dsp; dsp.X = pix(0); dsp.Y = pix(1);
//	ColorSpacePoint csp;
//	HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, pixValue, &csp);
//	ret << csp.X, csp.Y;
//	return true;
//	//	}
//
//#endif
//}

#if ENABLE_KINECT_V2
void RGBD_Sensor::ColorFrame2Camera(Eigen::Vector2d pix, Eigen::Vector3d& ret){
	CameraSpacePoint csp = csps[(int)pix(0) + ((int)pix(1))*color_width];
	ret << csp.X, csp.Y, csp.Z;
}
#endif

