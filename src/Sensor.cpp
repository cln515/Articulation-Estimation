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
	IColorFrameSource* p_color_source;
	iKinect->get_ColorFrameSource(&p_color_source);

	IDepthFrameSource* p_depth_source;
	iKinect->get_DepthFrameSource(&p_depth_source);

	IColorFrameReader* p_color_reader;
	p_color_source->OpenReader(&p_color_reader);

	IDepthFrameReader* p_depth_reader;
	p_depth_source->OpenReader(&p_depth_reader);

	IFrameDescription* p_frame_desc;
	p_color_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&color_width);
	p_frame_desc->get_Height(&color_height);
	p_depth_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&depth_width);
	p_frame_desc->get_Height(&depth_height);

	int color_width, color_height, depth_width, depth_height;

	cout << "color_width : " << color_width << endl;
	cout << "color_height : " << color_height << endl;
	cout << "depth_width : " << depth_width << endl;
	cout << "depth_height : " << depth_height << endl;

#endif











};
