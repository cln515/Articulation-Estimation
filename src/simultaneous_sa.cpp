#include "simultaneous_sa.h"

namespace fs = boost::filesystem;
using namespace Microsoft::WRL;


void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
	try
	{
		// Example: How to use the pose keypoints
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
			op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
			op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
			op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}



SegmentationArticulation::SegmentationArticulation(op::Wrapper* opWrapper_) {
	opWrapper = opWrapper_;
}

SegmentationArticulation::SegmentationArticulation(cv::Vec3f handPos)
{

}

SegmentationArticulation::~SegmentationArticulation()
{

}



void SegmentationArticulation::StartCaptureMode()
{

	ComPtr<IKinectSensor> iKinect;

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

	int color_width, color_height, depth_width, depth_height;
	IFrameDescription* p_frame_desc;
	p_color_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&color_width);
	p_frame_desc->get_Height(&color_height);
	p_depth_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&depth_width);
	p_frame_desc->get_Height(&depth_height);

	cout << "color_width : " << color_width << endl;
	cout << "color_height : " << color_height << endl;
	cout << "depth_width : " << depth_width << endl;
	cout << "depth_height : " << depth_height << endl;

	cv::namedWindow("Color");
	cv::namedWindow("Depth");
	int frame_counter = 0;


	ICoordinateMapper* coordinateMapper;
	iKinect->get_CoordinateMapper(&coordinateMapper);


	int depth_buffer_size = depth_width * depth_height;
	bool backgroundSaved = false;
	unsigned short *bg_depth_buffer;
	bg_depth_buffer = new unsigned short[depth_buffer_size];

	bool capturing = false;

	while (1) {
		IColorFrame* p_color_frame = nullptr;

		cv::Mat color_image(color_height, color_width, CV_8UC4);
		cv::Mat color_image3ch(color_height, color_width, CV_8UC3);
		cv::Mat depth_image = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);

		int color_buffer_size = color_width * color_height * 4 * sizeof(unsigned char);

		HRESULT color_result = p_color_reader->AcquireLatestFrame(&p_color_frame);
		
		if (SUCCEEDED(color_result)) {
			color_result = p_color_frame->CopyConvertedFrameDataToArray(color_buffer_size, reinterpret_cast<BYTE*>(color_image.data), ColorImageFormat_Bgra);
			if (SUCCEEDED(color_result)) {

			}
		}

		IDepthFrame* p_depth_frame = nullptr;


		unsigned short *depth_buffer;
		depth_buffer = new unsigned short[depth_buffer_size];

		HRESULT depth_result = p_depth_reader->AcquireLatestFrame(&p_depth_frame);

		if (SUCCEEDED(depth_result)) {
			p_depth_frame->CopyFrameDataToArray(depth_buffer_size, depth_buffer);

			// Color mapping
			for (int y = 0; y < depth_buffer_size; y++) {

				DepthSpacePoint dsp; dsp.X = y % depth_width; dsp.Y = y / depth_width;
				ColorSpacePoint csp;
				HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, depth_buffer[y], &csp);
				if (hr != S_OK || depth_buffer[y] == 0) continue;
				int cidx = (int)csp.X + (int)csp.Y*color_width;
				if (csp.X >= 0 && csp.X < color_width && csp.Y >= 0 && csp.Y < color_height) {
					depth_image.data[y * 3] = color_image.data[cidx * 4];
					depth_image.data[y * 3 + 1] = color_image.data[cidx * 4 + 1];
					depth_image.data[y * 3 + 2] = color_image.data[cidx * 4 + 2];
				}
			}
			flip(depth_image, depth_image, 1);
		}

		char key = cv::waitKey(30);
		if (key == VK_ESCAPE) {
			break;
		}
		else if (key == 'c') {
			capturing = !capturing;
			if (capturing) {
				cout << "capture start" << endl;
			}
			else {
				cout << "capture stop" << endl;
			}
		}
		else if (key == 'b') {
			backgroundSaved = true;
			memcpy(bg_depth_buffer, depth_buffer, depth_buffer_size * sizeof(unsigned short));
			std::string filename2 = outputFolder + "/background.dat";
			ofstream ofs(filename2, std::ios::binary);
			ofs.write((char*)depth_buffer, depth_buffer_size * sizeof(unsigned short));
			ofs.close();

			cv::cvtColor(color_image, color_image3ch, cv::COLOR_BGRA2BGR);
			std::string filename = outputFolder + "/background.png";
			cv::flip(color_image3ch, color_image3ch, 1);
			cv::imwrite(filename, color_image3ch);
			cout << filename << " and "<< filename2 << " is saved" << endl;
		}

		if (SUCCEEDED(color_result)) {
			cv::flip(color_image, color_image, 1);
			if (capturing) {
				cv::cvtColor(color_image,color_image3ch,cv::COLOR_BGRA2BGR);
				std::string filename = outputFolder+ "/color_" + std::to_string(frame_counter) + ".png";
				cv::imwrite(filename, color_image3ch);
				std::string filename2 = outputFolder + "/depth_" + std::to_string(frame_counter) + ".dat";
				ofstream ofs(filename2, std::ios::binary);
				ofs.write((char*)depth_buffer, depth_buffer_size * sizeof(unsigned short));
				ofs.close();
				cout << filename << " and "<< filename2 << " is saved" << endl;
				frame_counter++;
			}
			resize(color_image, color_image, cv::Size(), 0.5, 0.5);
			cv::imshow("Color", color_image);
			if (SUCCEEDED(depth_result)) {
				cv::imshow("Depth", depth_image);
			}
		}

		if (p_depth_frame != nullptr) {
			p_depth_frame->Release();
		}

		if (p_color_frame != nullptr) {
			p_color_frame->Release();
		}

	}

}


cv::Mat SegmentationArticulation::obtainDepthMap(std::string filename, int depthWidth_, int depthHeight_) {
	cv::Mat depthMap = cv::Mat(depthHeight_,depthWidth_,CV_16UC1);
	std::ifstream ifs(filename,std::ios::binary);
	ifs.read((char*)depthMap.data,depthHeight_*depthWidth_*sizeof(unsigned short));
	return depthMap.clone();
}

void SegmentationArticulation::PostProcessing() {

	pcl::PLYWriter plywriter;
	pcl::PLYReader plyreader;
	
	//hand tracking
	std::map<int, std::vector<cv::Rect2f>> m_map;
	bool handTracking = false;
	{
		std::ifstream ifs(inputFolder + "\\handrect.txt", std::ios::binary);
		std::string str;
		int frame = 0;
		if (ifs.is_open()) {
			std::vector<cv::Rect2f > rois;
			while (true) {
				if(!getline(ifs, str))break;
				if (str.find("f") == 0) {
					frame = stoi(str.substr(1));
					rois = std::vector<cv::Rect2f>();
					m_map.insert(std::make_pair(frame, rois));
				}
				else {
					std::vector<std::string> roistr = split(str,',');
					cv::Rect2f roicand;
					roicand.x = stof(roistr.at(0));
					roicand.y = stof(roistr.at(2));
					roicand.width = stof(roistr.at(1))-roicand.x;
					roicand.height = stof(roistr.at(3))-roicand.y;
					if (roicand.width > roicand.height)roicand.height = roicand.width;
					else roicand.width = roicand.height;
					m_map.at(frame).push_back(roicand);
				}
			}
			handTracking = true;
		}
	}

	//visualization
	pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Result");

	//viewer->addCoordinateSystem();
	// Set camera position and orientation
	viewer->setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer->setSize(1280, 1024);  // Visualiser window size

	ComPtr<IKinectSensor> iKinect;

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

	int color_width, color_height, depth_width, depth_height;
	IFrameDescription* p_frame_desc;
	p_color_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&color_width);
	p_frame_desc->get_Height(&color_height);
	p_depth_source->get_FrameDescription(&p_frame_desc);
	p_frame_desc->get_Width(&depth_width);
	p_frame_desc->get_Height(&depth_height);

	cout << "color_width : " << color_width << endl;
	cout << "color_height : " << color_height << endl;
	cout << "depth_width : " << depth_width << endl;
	cout << "depth_height : " << depth_height << endl;

	int frame_counter = 0;

	ICoordinateMapper* coordinateMapper;
	iKinect->get_CoordinateMapper(&coordinateMapper);
	unsigned short* depth_buffer = (unsigned short*)malloc(sizeof(unsigned short)*depth_height*depth_width);
	unsigned short thresh = 20;
	//load depth data
	cv::Mat depthbg=obtainDepthMap(inputFolder+"/background.dat",depth_width,depth_height);
	cv::Mat colorbg = cv::imread(inputFolder + "/background.png");
	pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pc_bg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_pc_bgcolor(new pcl::PointCloud<pcl::PointXYZRGB>);
	unsigned short* depth_bgbuffer = (unsigned short*)depthbg.data;
	depth_pc_bg->width = depth_width;
	depth_pc_bg->height = depth_height;
	depth_pc_bg->resize(depth_width*depth_height);

	depth_pc_bgcolor->width = depth_width;
	depth_pc_bgcolor->height = depth_height;
	depth_pc_bgcolor->resize(depth_width*depth_height);	

	int pointCount=0;
	ColorSpacePoint colsp;
	for (int y = 0; y < depth_width*depth_height; y++) {
		DepthSpacePoint dsp; dsp.X = y % depth_width; dsp.Y = y / depth_width;
		CameraSpacePoint csp;
		unsigned short depth_value = depth_bgbuffer[y];
		if(depth_value == 0) depth_value = 1000;//dummy depth	
			
		HRESULT hr = coordinateMapper->MapDepthPointToCameraSpace(dsp, depth_value, &csp);

		(*depth_pc_bg)(dsp.X, dsp.Y).x = csp.X;
		(*depth_pc_bg)(dsp.X, dsp.Y).y = csp.Y;
		(*depth_pc_bg)(dsp.X, dsp.Y).z = csp.Z;
		std::cout<<"p\r";// For compiler error
		if (!(SUCCEEDED(hr)&& !isnan(csp.X) && !isnan(csp.Y) && !isnan(csp.Z))) {
			(*depth_pc_bg)(dsp.X, dsp.Y).x = 0;
			(*depth_pc_bg)(dsp.X, dsp.Y).y = 0;
			(*depth_pc_bg)(dsp.X, dsp.Y).z = 0;
		}else{
			HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, depth_value, &colsp);
			if (hr != S_OK || depth_bgbuffer[y] == 0) continue;
			int cidx = (int)(color_width - colsp.X - 1) + (int)colsp.Y*color_width;
			if (colsp.X >= 0 && colsp.X < color_width && colsp.Y >= 0 && colsp.Y < color_height) {
				
				pcl::PointXYZRGB pt = pcl::PointXYZRGB();
				pt.x = csp.X;
				pt.y = csp.Y;
				pt.z = csp.Z;
				pt.r = colorbg.data[cidx * 3 + 2];
				pt.g = colorbg.data[cidx * 3 + 1];
				pt.b = colorbg.data[cidx * 3];

				(*depth_pc_bgcolor)(dsp.X, dsp.Y)=pt;
				pointCount++;
			}
		}
	}
	std::cout <<  colsp.X << "," << colsp.Y << std::endl;
	std::cout<<"colored points: "<<pointCount<<std::endl;
	

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.1f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(depth_pc_bg);
	ne.compute(*normals);

	RemoveNormalPtr(depth_pc_bg,depth_pc_bgcolor, normals);


	plywriter.write<pcl::PointXYZRGB>(outputFolder + "/back_.ply", *depth_pc_bgcolor, true);
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_frames, pc_full_frames, pc_compare_frames;
	std::vector<pcl::PointCloud<pcl::Normal>::Ptr> pc_full_normals;
	std::vector<Eigen::Vector3d> hand_pos;
	std::vector<std::vector<Eigen::Vector3d>> hand_pos_detail;
	std::vector<std::vector<double>> hand_score_detail;

	std::vector<Eigen::Vector3d> arucoCorner_pos;


	int isRightHand = 1;


	for (int i = firstFrame; i < lastFrame; i++) {
	
		std::stringstream ss,ss2,ss3,ss4;
		ss << inputFolder << "\\mask_" << i << ".png";
		ss2 << inputFolder << "\\depth_" << i << ".dat";
		cv::Mat humanMask = cv::imread(ss.str());
		cv::flip(humanMask, humanMask, 1);
		cv::erode(humanMask, humanMask, cv::Mat(9, 9, CV_8U, cv::Scalar(1)));
		cv::Mat depthMask = cv::Mat::zeros(depth_height,depth_width,CV_8UC1);
		cv::Mat depthDiff = cv::Mat::zeros( depth_height, depth_width, CV_8UC1);
		std::ifstream ifs(ss2.str(), std::ios::binary);
		
		ifs.read((char*)depth_buffer , depth_height*depth_width * sizeof(unsigned short));
		ifs.close();

		for (int y = 0; y < depth_width*depth_height; y++) {
			DepthSpacePoint dsp; dsp.X = y % depth_width; dsp.Y = y / depth_width;
			ColorSpacePoint csp;
			HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, depth_buffer[y], &csp);

			if (hr != S_OK || depth_buffer[y] == 0) continue;
			int cidx = (int)csp.X + (int)csp.Y*color_width;
			if (csp.X >= 0 && csp.X < color_width && csp.Y >= 0 && csp.Y < color_height) {
				depthMask.data[y] = 128;
				if (humanMask.data[cidx*3] != 255)depthMask.data[y] = 255;		//mask data: 3ch
			}
		}
		// 0  : static
		// 64 : human
		// 128: out of range
		// 255: dynamic (non-human object)
		for (int y = 0; y < depth_width*depth_height; y++) {
			if (depthMask.data[y] == 128){
				if (fabs(((unsigned short*)depthbg.data)[y] - ((unsigned short*)depth_buffer)[y]) > thresh) { depthDiff.data[y] = 255; }				
			}
			else if (depthMask.data[y] == 255) {
				depthDiff.data[y] = 64;
			}else{
				depthDiff.data[y] = 128;
			}
		}
		ss3 << outputFolder << "\\differ_" << i << ".png";
		cv::imwrite(ss3.str(),depthDiff);

		std::stringstream ss_img;
		ss_img << inputFolder << "\\color_" << i << ".png";;
		cv::Mat cvtrgb = cv::imread(ss_img.str());

		std::vector<std::array<op::Rectangle<float>, 2>> handRectangles;

		auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
		const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvtrgb);
		datumsPtr->emplace_back();
		auto& datumPtr = datumsPtr->at(0);
		datumPtr = std::make_shared<op::Datum>();
		// Fill datum with image and handRectangles
		datumPtr->cvInputData = imageToProcess;

		if (handTracking) {
			for (int handroi_idx = 0; handroi_idx < m_map.at(i).size(); handroi_idx++) {
				if (isRightHand) {
					handRectangles.push_back(
						std::array<op::Rectangle<float>, 2>{
						op::Rectangle<float>{0.f, 0.f, 0.f, 0.f}, // Left hand
						op::Rectangle<float>{m_map.at(i).at(handroi_idx).x, m_map.at(i).at(handroi_idx).y,
							m_map.at(i).at(handroi_idx).width, m_map.at(i).at(handroi_idx).height}});
				}
				else{
					handRectangles.push_back(
					std::array<op::Rectangle<float>, 2>{
						op::Rectangle<float>{m_map.at(i).at(handroi_idx).x, m_map.at(i).at(handroi_idx).y, 
							m_map.at(i).at(handroi_idx).width, m_map.at(i).at(handroi_idx).height}, // Left hand
							op::Rectangle<float>{0.f, 0.f, 0.f, 0.f}});
				}

			}
			const std::vector<std::array<op::Rectangle<float>, 2>> handRectangles2(handRectangles);
			datumPtr->handRectangles = handRectangles2;
		}
		// Process and display image
		opWrapper->emplaceAndPop(datumsPtr);
		

		CameraSpacePoint* csps = new CameraSpacePoint[colorWidth*colorHeight];
		HRESULT hr = coordinateMapper->MapColorFrameToCameraSpace(depth_width*depthHeight, depth_buffer, colorWidth*colorHeight, csps);

		Eigen::Vector3d handP; handP << 0, 0, -1;
		std::vector<Eigen::Vector3d> handPoints = std::vector<Eigen::Vector3d>(21, Eigen::Vector3d::Zero()), handPoints_candidate = std::vector<Eigen::Vector3d>(21, Eigen::Vector3d::Zero());
		std::vector<double> handScores = std::vector<double>(21, 0), handScores_candidate = std::vector<double>(21, 0);

		if (datumsPtr->at(0)->handKeypoints[isRightHand].getCvMat().rows() >= 1) {
			int person = datumsPtr->at(0)->handKeypoints[isRightHand].getCvMat().rows();

			float* databuf = (float*)datumsPtr->at(0)->handKeypoints[isRightHand].getCvMat().data();
			
			
			// pixel to 3D coordinates
			//hand position computation
			double hgx = 0, hgy = 0, hgz = 0;

			double maxscore = 0;
			for (int pn = 0; pn < person; pn++) {
				double score=0;
				int validcnt = 0;
				int confidentialPoint=0;
				for (int y = 0; y < 21; y++) {
					if (databuf[pn*21 + y * 3 + 2] < 0.01 )continue;//low score is removed
					confidentialPoint++;
					if (databuf[pn * 21 + y * 3] < 0 || databuf[pn * 21 + y * 3] >= colorWidth)continue;
					if (databuf[pn * 21 + y * 3 + 1] < 0 || databuf[pn * 21 + y * 3 + 1] >= colorHeight)continue;
					//flip is needed
					CameraSpacePoint csp = csps[(color_width-(int)databuf[pn * 21 + y * 3])+ ((int)databuf[pn * 21 + y * 3 + 1])*colorWidth];
					if (csp.Z < 0)continue;
					hgx += csp.X;
					hgy += csp.Y;
					hgz += csp.Z;
					validcnt++;
					score += databuf[pn * 21 + y * 3 + 2];
					Eigen::Vector3d eigen_hand;
					eigen_hand << csp.X, csp.Y, csp.Z;
					handPoints_candidate[y] = eigen_hand;
					handScores_candidate[y] = databuf[pn * 21 + y * 3 + 2];

				}
				if (validcnt > 10) {
					hgx /= validcnt;
					hgy /= validcnt;
					hgz /= validcnt;
					CameraSpacePoint camsp; camsp.X = hgx; camsp.Y = hgy; camsp.Z = hgz;
					if (score > maxscore) {
						maxscore = score;
						handP << camsp.X , camsp.Y , camsp.Z;
						handPoints = std::vector<Eigen::Vector3d>(handPoints_candidate);
						handScores = std::vector<double>(handScores_candidate);

					}
				}
			}
			hand_pos.push_back(handP);
	
		}
		hand_pos_detail.push_back(handPoints);
		hand_score_detail.push_back(handScores);

		pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pc(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pc_compare(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pc_all(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);

		depth_pc_all->width = depth_width;
		depth_pc_all->height = depth_height;
		depth_pc_all->resize(depth_width*depth_height);

		cout << "cvt depth map 2 pc" << endl;
		for (int y = 0; y < depth_width*depth_height; y++) {
			if (depthDiff.data[y] == 255 || depthDiff.data[y]==64 || depthDiff.data[y] == 0) {
				DepthSpacePoint dsp; dsp.X = y % depth_width; dsp.Y = y / depth_width;
				CameraSpacePoint csp;
				HRESULT hr = coordinateMapper->MapDepthPointToCameraSpace(dsp, depth_buffer[y], &csp);
				if (SUCCEEDED(hr)) {
					if (depthDiff.data[y] != 64) (*depth_pc_all)(dsp.X,dsp.Y) = (pcl::PointXYZ(csp.X, csp.Y, csp.Z));
					double distFromHand = handP(2) > 0 ? (handP(0) - csp.X)*(handP(0) - csp.X)+ (handP(1) - csp.Y)*(handP(1) - csp.Y) + (handP(2) - csp.Z)*(handP(2) - csp.Z)
						: -1;

					if (depthDiff.data[y] == 255 || (distFromHand > 0 && distFromHand<0.2)) {
						depth_pc->push_back(pcl::PointXYZ(csp.X, csp.Y, csp.Z));
					}
					if (depthDiff.data[y] != 0){
						depth_pc_compare->push_back(pcl::PointXYZ(csp.X, csp.Y, csp.Z));
					}
				}
			}
		}
		ne.setInputCloud(depth_pc_all);   
		ne.compute(*normals_all);

		RemoveNormalPtr(depth_pc_all, normals_all);

		ss4 << outputFolder << "\\diff_" << i << ".ply";
		plywriter.write<pcl::PointXYZ>(ss4.str(), *depth_pc, true);

		delete csps;

		// Color mapping for paper
		cv::Mat depth_image = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);
		int depth_buffer_size = depth_width * depth_height;

		for (int y = 0; y < depth_buffer_size; y++) {
			DepthSpacePoint dsp; dsp.X = y % depth_width; dsp.Y = y / depth_width;
			ColorSpacePoint csp;
			HRESULT hr = coordinateMapper->MapDepthPointToColorSpace(dsp, depth_buffer[y], &csp);

			if (hr != S_OK || depth_buffer[y] == 0) continue;
			int cidx = (int)(color_width- csp.X-1) + (int)csp.Y*color_width;

			if (csp.X >= 0 && csp.X < color_width && csp.Y >= 0 && csp.Y < color_height) {
				depth_image.data[y * 3] = cvtrgb.data[cidx * 3];
				depth_image.data[y * 3 + 1] = cvtrgb.data[cidx * 3 + 1];
				depth_image.data[y * 3 + 2] = cvtrgb.data[cidx * 3 + 2];
			}
		}


		flip(depth_image, depth_image, 1);
		cv::imwrite(outputFolder + "\\depth_colormap" + std::to_string(i) + ".png", depth_image);
		pc_frames.push_back(depth_pc);
		pc_full_frames.push_back(depth_pc_all);
		pc_full_normals.push_back(normals_all);
		if (i == firstFrame) {
			pc_compare_frames.push_back(depth_pc_bg);
		}
		else {
			pc_compare_frames.push_back(depth_pc_compare);
		}
	}

	PrismaticObject po;
	RevoluteObject ro;

	std::ofstream paramOfs(outputFolder+"\\param.txt");
	std::vector<int> inlier1,inlier2;
	po.ParameterEstimationFromHandPoints(hand_pos, inlier1);
	double range = ro.ParameterEstimationFromHandPoints(hand_pos, inlier2);
	
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_frames_sub, pc_frames_full_sub;
	std::vector<pcl::PointCloud<pcl::Normal>::Ptr> pc_normals_full_sub;
	std::vector < Eigen::Vector3d> hand_pos_sub;
	std::vector<std::vector<double>> hand_score_det_sub;
	std::vector<std::vector<Eigen::Vector3d>> hand_point_det_sub;


	Eigen::Vector3d hp = hand_pos.at(0);
	for (int i = 1; i < hand_pos.size(); i++) {
		if (hp(2)<=0) {
			hp = hand_pos.at(i);
			continue;
		}
		if (hand_pos.at(i)(2) > 0) {
			pcl::PointXYZ p1, p2;

			p1.x = hp(0);
			p1.y = hp(1);
			p1.z = hp(2);

			hp = hand_pos.at(i);

			p2.x = hp(0);
			p2.y = hp(1);
			p2.z = hp(2);

			std::string handlineId = "hand" + std::to_string(i);
			viewer->addLine(p1, p2, 0.0,1.0,0.0,handlineId );
		}
	}

	double maxUsedFrame =std::min(15,lastFrame-firstFrame);

	JointTypes jt;

	//at least 30 degree 
	if (range > M_PI / 6) {
		jt = revolute;
	}
	else {
		jt = prismatic;
	}

	if (jt==prismatic) {
		double itv = inlier1.size()/maxUsedFrame;
		if (itv < 0)itv = 1.0;
		for (double i = 0; i < inlier1.size(); i+=itv) {
			pc_frames_sub.push_back(pc_frames.at(inlier1.at((int)i)));
			pc_frames_full_sub.push_back(pc_full_frames.at(inlier1.at((int)i)));
			pc_normals_full_sub.push_back(pc_full_normals.at(inlier1.at((int)i)));
			hand_pos_sub.push_back(hand_pos.at(inlier1.at((int)i)));
			hand_score_det_sub.push_back(hand_score_detail.at(inlier1.at((int)i)));
			hand_point_det_sub.push_back(hand_pos_detail.at(inlier1.at((int)i)));
		}
	} else{
		double itv = inlier2.size() / maxUsedFrame;
		if (itv < 0)itv = 1.0;
		for (double i = 0; i < inlier2.size(); i+=itv) {
			pc_frames_sub.push_back(pc_frames.at(inlier2.at((int)i)));
			pc_frames_full_sub.push_back(pc_full_frames.at(inlier2.at((int)i)));
			pc_normals_full_sub.push_back(pc_full_normals.at(inlier2.at((int)i)));
			hand_pos_sub.push_back(hand_pos.at(inlier2.at((int)i)));
			hand_score_det_sub.push_back(hand_score_detail.at(inlier2.at((int)i)));
			hand_point_det_sub.push_back(hand_pos_detail.at(inlier2.at((int)i)));
		}
	}

	SimultaneousICPModule simICPModule(depth_pc_bg, pc_frames_sub);

	simICPModule.setHandPoints(hand_pos_sub);

	double err[4];

	if (jt == prismatic) {
		cout << "Prismatic Joint" << endl;
		paramOfs << "Prismatic Joint" << endl;
		simICPModule.setJointTypeAndParams(JointTypes::prismatic, po.getJointParams());
		simICPModule.writeParam(&paramOfs, "hand");
	}
	else {
		cout << "Revolute Joint" << endl;
		paramOfs << "Revolute Joint" << endl;
		simICPModule.setJointTypeAndParams(JointTypes::revolute, ro.getJointParams());
		simICPModule.writeParam(&paramOfs, "hand");
	}


	std::vector<int> segidx;

	simICPModule.ICP_run();
	simICPModule.SegData_set(pc_frames_full_sub, pc_normals_full_sub, normals);
	segidx = simICPModule.Segmentation_run();
	
	simICPModule.setDetailedHandPoints(hand_point_det_sub, hand_score_det_sub);
	for (int i = 0; i < 2; i++) {
		simICPModule.ICP_refine_run();
		segidx = simICPModule.Segmentation_run(0.025, 0.03);
	}
	for (int i = 0; i < segidx.size(); i++) {
		depth_pc_bgcolor->at(segidx.at(i)).r = 255;
		depth_pc_bgcolor->at(segidx.at(i)).g = depth_pc_bgcolor->at(segidx.at(i)).g*0.5;
		depth_pc_bgcolor->at(segidx.at(i)).b = depth_pc_bgcolor->at(segidx.at(i)).b*0.5;
	}

	viewer->addPointCloud(depth_pc_bgcolor, "cloud_color");

	simICPModule.outPointCloud(outputFolder);
	simICPModule.writeParam(&paramOfs, "Result");

	simICPModule.articulationVisualize(viewer, 1.0, 0.0, 0.0,true);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return;
}
