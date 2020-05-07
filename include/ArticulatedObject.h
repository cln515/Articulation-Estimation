#pragma once
#include "main.h"
#include "others.h"
#include <algorithm>
#define NOMINMAX
#undef ERROR
#include <ceres/ceres.h>

enum JointTypes { unknown, prismatic, revolute };


class PrismaticObject{
public:
	void ParameterEstimationFromHandPoints(std::vector<Eigen::Vector3d> handpoint, std::vector<int>& inlier);
	double* getJointParams() { return jointParam; }
private:
	//parameters
	Eigen::Vector3d direction;
	double jointParam[2];//theta,phi
};

class RevoluteObject {
public:
	double ParameterEstimationFromHandPoints(std::vector<Eigen::Vector3d> handpoint, std::vector<int>& inlier);
	double* getJointParams() { return jointParam; }
private:
	//parameters
	Eigen::Vector3d direction;
	Eigen::Vector3d lineposition;
	double jointParam[4];//theta,phi,r,theta2
};


void Param2TFMatrix(JointTypes jt, double* params_joint, double* params_frame, Eigen::Matrix3d& R,Eigen::Vector3d &t);