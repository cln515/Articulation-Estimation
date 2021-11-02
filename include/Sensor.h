#if ENABLE_KINECT_V2
#include <kinect.h>
#elif ENABLE_AZURE_KINECT
#include <k4a/k4a.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <wrl/client.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

class RGBD_Sensor {
public:
	void openInitDevice();

	void getImageSizes(int& color_width_,int& color_height_,int& depth_width_,int& depth_height_){
		color_width_ = color_width; color_height_ = color_height;
		depth_width_ = depth_width; depth_height_ = depth_height;
	};

	int color_width, color_height, depth_width, depth_height;

	bool getColorImage(cv::Mat& outmat);
	bool getDepthImage(cv::Mat& outmat);
	bool Depth2ColorPixel(Eigen::Vector2d pix,uint pixValue,Eigen::Vector2d& ret);	
	bool Depth2CameraSpace(Eigen::Vector2d pix, uint pixValue, Eigen::Vector3d& ret);
	void ColorFrame2Camera(Eigen::Vector2d pix, Eigen::Vector3d& ret);
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
#elif ENABLE_AZURE_KINECT
	k4a::device device;
	k4a::capture cap;
	k4a::calibration calib;
	k4a::transformation transformation;
	k4a::image depthImage, transformedDepthImage;
	
	void capture(){
		device.get_capture(&cap, std::chrono::milliseconds(0));
	}
	void SetColorFrame2Camera(unsigned short* depth_buffer) {
		std::cout << "p";
depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,depth_width,depth_height,depth_width*sizeof(unsigned short));
std::cout << "q";		const uchar* buf = depthImage.get_buffer();
		memcpy((void*)buf,depth_buffer,depth_width*depth_height*sizeof(unsigned short));
		
		transformedDepthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height, color_width * sizeof(unsigned short));

		transformation.depth_image_to_color_camera(depthImage,&transformedDepthImage);
		//delete depthImage;
		std::cout << "r";
	}

	void saveSensorData(std::string filepath) {
		std::vector<uchar> rawcalib = device.get_raw_calibration();
		std::ofstream ofs(filepath, std::ios::binary);
		ofs.write((char*)&color_width,sizeof(int)); 
		ofs.write((char*)&color_height, sizeof(int));
		ofs.write((char*)&depth_width, sizeof(int));
		ofs.write((char*)&depth_height, sizeof(int));
		ofs.write((char*)rawcalib.data(), rawcalib.size());
		ofs.close();

	};

	bool loadSensorData(std::string filepath) {
		std::ifstream ifs(filepath, std::ios::binary);
		ifs.seekg(0, std::ios_base::end);
		int filesize = ifs.tellg();
		ifs.seekg(0, std::ios_base::beg);
		if (filesize == 0) {
			return false;
		}
		else {
			uchar* calibdata = (uchar*)malloc(filesize- sizeof(int)* 4);
			ifs.read((char*)&color_width, sizeof(int));
			ifs.read((char*)&color_height, sizeof(int));
			ifs.read((char*)&depth_width, sizeof(int));
			ifs.read((char*)&depth_height, sizeof(int));
			ifs.read((char*)calibdata, filesize- sizeof(int) * 4);
			calib = k4a::calibration::get_from_raw(calibdata, filesize, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);
			transformation = k4a::transformation(calib);
		}
		return true;
		
	};

#endif


};