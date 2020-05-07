#pragma once
#include <main.h>
#include <others.h>
#include <ArticulatedObject.h>
#define NOMINMAX
#undef ERROR
#include <ceres/ceres.h>


class SimultaneousICPModule {
public:
	SimultaneousICPModule(pcl::PointCloud<pcl::PointXYZ>::Ptr originFrame_, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_) {
		clouds = clouds_;
		originFrame = originFrame_;
		
//		tf_mats = std::vector<Eigen::Matrix4d>(clouds.size());
	}


	void ICP_run();
	void ICP_refine_run();

	void SegData_set(std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds_full_,
		std::vector < pcl::PointCloud<pcl::Normal>::Ptr > clouds_normal_full_,
		pcl::PointCloud<pcl::Normal>::Ptr background_normal_full_) {
		clouds_full = clouds_full_;
		clouds_normal_full = clouds_normal_full_;
		background_normal_full = background_normal_full_;	
		for (int i = 0; i < clouds_full.size(); i++) {
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(clouds_full.at(i));
			kdtrees.push_back(kdtree);
		}
	};

	std::vector<int> Segmentation_run(double dynamic_thresh = 0.025, double symmetric_thresh = 0.05, double cluster_param = 0.015);

	void setJointTypeAndParams(JointTypes jt_,double* params) {
		jt = jt_;
		jointParams = params;
	}
	void setJointType(JointTypes jt_) {
		jt = jt_;
	}
	double InitialPointComputation(Eigen::Vector3d handPosition);

	void setHandPoints(std::vector<Eigen::Vector3d> HandPoints_) {
		HandPoints = HandPoints_;
		BaseHandPoint = HandPoints_.at(0);
		handSet = true;
	}
	void outPointCloud(std::string fileFolder);

	void resultVisualize(pcl::visualization::PCLVisualizer* viewer);
	void articulationVisualize(pcl::visualization::PCLVisualizer* viewer, double r = 1.0, double g = 1.0, double b = 1.0, bool showRange = false) {
		articulationVisualize(viewer, "articulation" + std::to_string(visualizeCalledCnt), r, g, b, showRange);
	};
	void articulationVisualize(pcl::visualization::PCLVisualizer* viewer, std::string idtag, double r = 1.0, double g = 1.0, double b = 1.0, bool showRange = false);

	void writeParam(std::ofstream* ofs, std::string tag) {
		if (jt == prismatic) {
			(*ofs) << tag << std::endl << jointParams[0] << ","
				<< jointParams[1] << endl;
		}
		else if(jt == revolute){
			(*ofs) << tag << std::endl << jointParams[0] << ","
				<< jointParams[1] << ","
				<< jointParams[2] << ","
				<< jointParams[3] << endl;
		}
	};

	double* getJointParam() { return jointParams; };
	void setDetailedHandPoints(std::vector<std::vector<Eigen::Vector3d>> hand_point_det_sub_, std::vector<std::vector<double>>hand_score_det_sub_) {
		hand_score_det_sub = hand_score_det_sub_;
		hand_point_det_sub = hand_point_det_sub_;
	};
	double* getMovementRange() {
		double* minmax = (double*)malloc(sizeof(double) * 2);
		minmax[0] = 10;
		minmax[1] = -10;
		for (int i = 0; i < clouds.size(); i++) {
			if (minmax[0] > amountParams[i]) {
				minmax[0] = amountParams[i];
			}
			if (minmax[1] < amountParams[i]) {
				minmax[1] = amountParams[i];
			};
		}
		return minmax;
	};

private:
	std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
//	std::vector<Eigen::Matrix4d> tf_mats;
	std::vector<Eigen::Vector3d> HandPoints;
	Eigen::Vector3d BaseHandPoint;
	pcl::PointCloud<pcl::PointXYZ>::Ptr originFrame;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segm_pc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segm_cluster_pc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr symm_pc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr bg_pc;

	std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds_full;
	std::vector < pcl::PointCloud<pcl::Normal>::Ptr > clouds_normal_full;
	pcl::PointCloud<pcl::Normal>::Ptr background_normal_full;
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtrees;
	std::vector<std::vector<double>> hand_score_det_sub;
	std::vector<std::vector<Eigen::Vector3d>> hand_point_det_sub;

	JointTypes jt = prismatic;
	double* jointParams;
	double* amountParams;
	double dist_thresh = 0.5;
	int maxitr = 10;
	int visualizeCalledCnt = 0;
	bool handSet = false;
	
};

struct ClosestNNFunc{
public:
	ClosestNNFunc(Eigen::Vector3d source_,Eigen::Vector3d target_) {
		source = source_;
		target = target_;
	}
	bool operator()(const double* parameters2, const double* parameters3, double* residual) const {
		Eigen::Vector4d q; q << parameters2[0], parameters2[1], parameters2[2], 1;
		Eigen::Vector3d t; t << parameters3[0], parameters3[1], parameters3[2];
		q.normalize();
		Eigen::Matrix3d addMat = q2dcm(q);

		Eigen::Vector3d scanp_ = addMat * target +  t;
		residual[0] = w1*w2 *(scanp_ - source).norm();//eyeDirec.cross(plot).norm();

		return true;
	}
	void setWeight(double w1_, double w2_) { w1 = w1_, w2 = w2_; }

private:
	Eigen::Vector3d source;
	Eigen::Vector3d target;
	double w1=1.0, w2 = 1.0;
};

struct ArticulationClosestNNFunc {
public:
	ArticulationClosestNNFunc(Eigen::Vector3d source_, Eigen::Vector3d target_,JointTypes jt_) {
		source = source_;
		target = target_;
		jt=jt_;
		if(jt_==JointTypes::prismatic){
			paramDoF=2;
		}else if(jt_ == JointTypes::revolute){
			paramDoF=4;

		}

	}
	bool operator()(const double* parameter_joint, const double* parameter_source, const double* parameter_targ, double* residual) const {

		std::vector<int> coorespondsIdx_cloud, coorespondsIdx_target;
		Eigen::Matrix4d transformationMatrix, m1 = Eigen::Matrix4d::Identity(), m2 = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d R, R2;
		Eigen::Vector3d t, t2;
		double* jointParams=new double[paramDoF];
		memcpy(jointParams,parameter_joint,paramDoF*sizeof(double));
		double amountParam1=anc==1? 0 :parameter_source[0],amountParam2 = anc==2?0:parameter_targ[0];
		Param2TFMatrix(jt, jointParams, &amountParam1, R, t);
		Param2TFMatrix(jt, jointParams, &amountParam2, R2, t2);

		m1.block(0, 0, 3, 3) = R;
		m1.block(0, 3, 3, 1) = t;
		m2.block(0, 0, 3, 3) = R2;
		m2.block(0, 3, 3, 1) = t2;

		transformationMatrix = m2.inverse()*m1;

		Eigen::Vector3d scanp_ = R2.transpose()*((R*target+t)-t2);
		//Eigen::Vector3d err_ = w12 *(scanp_ - source);
		residual[0] = w12 *(scanp_ - source).norm();

		delete jointParams;
		return true;
	}
	void setWeight(double w1_, double w2_) { w12 =w1_*w2_; }
	void setAnchor(int anc_){
		anc=anc_;
	}

private:
	Eigen::Vector3d source;
	Eigen::Vector3d target;
	JointTypes jt;
	int paramDoF=2 ;
	double w12 = 1.0;
	int anc=0;
};

