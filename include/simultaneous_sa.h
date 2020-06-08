#pragma once
#include "main.h"

#include "others.h"
#include "ICPModule.h"
#include "ArticulatedObject.h"

#include <thread>
#include <chrono>
#include <stdexcept>
#include <ppl.h>

// Command-line user interface
//#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

using namespace Microsoft::WRL;



class SegmentationArticulation {
private:
	int b = 0;
	bool next_iteration = false;
	
	std::vector<Eigen::Matrix4d> tfMatrices;
	

	// Decide Folder Path
	std::string outputFolder = "/output";
	std::string inputFolder = "/input";

	// Kinect parameters
	int depthWidth = 512;
	int depthHeight = 424;
	int colorWidth = 1920;
	int colorHeight = 1080;

	int firstFrame = 0;
	int lastFrame = 0;

	int colorBytesPerPixel = 4;
	
//	ComPtr<IKinectSensor> kinectSensor;
//	ComPtr<ICoordinateMapper> coordinateMapper;
	
	 op::Wrapper* opWrapper;

	std::vector<Eigen::Matrix4d> ICPTransforms;
	
public:
	// Constructor
	SegmentationArticulation(op::Wrapper *opWrapper_);
	SegmentationArticulation(cv::Vec3f handPos);

	// Destructor
	~SegmentationArticulation();

	// Processing
	
	int a = 0;

	void SetInputFolder(std::string inputFolder_) {
		inputFolder = inputFolder_;
	};

	void SetOutputFolder(std::string outputFolder_) {
		outputFolder = outputFolder_;
	};

	void SetFrameIdx(int startFrame_, int endFrame_) {
		firstFrame = startFrame_;
		lastFrame = endFrame_;	
	}

	void StartCaptureMode();
	void PostProcessing();
	cv::Mat obtainDepthMap(std::string filename,int depthWidth,int depthHeight);
};