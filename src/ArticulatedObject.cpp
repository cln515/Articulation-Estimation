#include "ArticulatedObject.h"


void PrismaticObject::ParameterEstimationFromHandPoints(std::vector<Eigen::Vector3d> handpoint, std::vector<int>& inlier_) {
	
	double lineangle[] = { 0,0 };
	//Line fitting
	//Ransac
	std::vector<int> idces;
	for (int i = 0; i < handpoint.size(); i++) {
		if (handpoint.at(i)(2)>0)idces.push_back(i);
	}

	int maxcnt = 0;
	int bestidces[2];
	double thresh = 0.05;
	std::vector<int> inlier;
	for(int i=0;i< 50; i++){
		std::random_shuffle(idces.begin(), idces.end());
		Eigen::Vector3d p1 = handpoint.at(idces[0]), p2 = handpoint.at(idces[1]), d;
		d = (p2 - p1).normalized();
		int cnt = 0;
		inlier.clear();
		for (int j = 0; j < handpoint.size(); j++) {
			Eigen::Vector3d p3= handpoint.at(j);
			double err = (p3 - p1).cross(d).norm();
			if (err < thresh) {
				cnt++;
				inlier.push_back(j);
			}
		}
		if (cnt>maxcnt) {
			bestidces[0] = idces[0];
			bestidces[1] = idces[1];
			maxcnt = cnt;
		}
		if (maxcnt >= handpoint.size()*0.9)break;
	}

	struct LineFitCostFunc{
	public: 
		LineFitCostFunc(Eigen::Vector3d p_){
			p = p_;
		}
		bool operator()(const double* parameters2, const double* parameters3, double* residual) const {
			double theta = parameters2[0];
			double phi = parameters2[1];
			double h = parameters3[0];
			double theta2 = parameters3[1];
			Eigen::Vector3d n, nt;
			LineParameterCvt(theta, phi, h, theta2,n,nt);
			residual[0]=(p + nt).cross(n).norm();
			return true;
		}
	private:
		Eigen::Vector3d p;
	};

	//compute init parameter
	Eigen::Vector3d bestc1 = handpoint.at(bestidces[0]);
	Eigen::Vector3d bestc2 = handpoint.at(bestidces[1]);
	Eigen::Vector3d initn = (bestc2 - bestc1).normalized(),initnt;
	initnt = bestc1 - (bestc1.dot(initn))*initn;

	double param1[2], param2[2];
	RevLineParameterCvt(initn,initnt,param1[0],param1[1], param2[0], param2[1]);
	LineParameterCvt(param1[0], param1[1], param2[0], param2[1], bestc1, bestc2);

	ceres::Problem problem;
	for(int i=0;i<inlier.size();i++){
		LineFitCostFunc* f = new LineFitCostFunc(handpoint.at(inlier.at(i)));
		ceres::CostFunction* c = new ceres::NumericDiffCostFunction<LineFitCostFunc, ceres::CENTRAL, 1, 2, 2>(f);
		problem.AddResidualBlock(c, NULL, param1, param2);
	}
	ceres::Solver::Options options;
	options.max_num_iterations = 1e3;
	options.function_tolerance = 1e-5;
	options.parameter_tolerance = 1e-5;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	LineParameterCvt(param1[0], param1[1], param2[0], param2[1],bestc1,bestc2);
	jointParam[0] = param1[0]; jointParam[1] = param1[1];
	inlier_ = std::vector<int>(inlier);
}


double RevoluteObject::ParameterEstimationFromHandPoints(std::vector<Eigen::Vector3d> handpoint,std::vector<int>& inlier_) {

	double lineangle[] = { 0,0 };
	//Circle fitting
	//Ransac
	std::vector<int> idces(handpoint.size());
	for (int i = 0; i < idces.size(); i++)if(handpoint.at(i)(2)>0)idces.at(i) = i;

	int maxcnt = 0;
	int bestidces[3];
	double thresh = 0.05;
	std::vector<int> inlier;
	double param1[6];

	for (int i = 0; i < 50; i++) {
		std::random_shuffle(idces.begin(), idces.end());
		Eigen::Vector3d p1 = handpoint.at(idces[0]), p2 = handpoint.at(idces[1]), p3 = handpoint.at(idces[2]), d;
		Eigen::Matrix3d A(3,3);
		A.row(0) = 2 * (p1 - p2).transpose();
		A.row(1) = 2 * (p1 - p3).transpose();
		A.row(2) = (p2 - p3).cross(p1-p3);
		Eigen::VectorXd B(3),C_;
		Eigen::Vector3d C;
		B(0) = p1.dot(p1)- p2.dot(p2);
		B(1) = p1.dot(p1) - p3.dot(p3);
		B(2) = A.row(2).dot(p1);
		C_=solveSimultaneousEquation(A, B );
		C << C_(0), C_(1), C_(2);
		double r = sqrt( p1.dot(p1) - 2 * C.dot(p1) + C.dot(C));
		Eigen::Vector3d n = (C - p1).cross(C - p3).normalized();

		int cnt = 0;
		inlier.clear();
		for (int j = 0; j < handpoint.size(); j++) {
			Eigen::Vector3d p = handpoint.at(j);
			double costheta = n.cross((p-C).normalized()).norm();
			double pcdot = (p - C).dot(p - C);
			double err = sqrt(r*r + pcdot - 2 * r* sqrt(pcdot)* costheta);

			if (err < thresh) {
				cnt++;
				inlier.push_back(j);
			}
		}
		if (cnt > maxcnt) {
			bestidces[0] = idces[0];
			bestidces[1] = idces[1];
			bestidces[2] = idces[2];
			param1[0] = C(0);			param1[1] = C(1);			param1[2] = C(2);
			param1[3] = atan2(n(1), n(0)); param1[4] = acos(n(2)); param1[5] = r;
			maxcnt = cnt;
		}	
		if (maxcnt >= handpoint.size()*0.9)break;
	}

	struct CircleFitCostFunc {
	public:
		CircleFitCostFunc(Eigen::Vector3d p_) {
			p = p_;
		}
		bool operator()(const double* parameters, double* residual) const {
			double cx = parameters[0];
			double cy = parameters[1];
			double cz = parameters[2];
			double theta = parameters[3];
			double phi = parameters[4];
			double r = fabs(parameters[5]);

			Eigen::Vector3d n, c;
			c << cx, cy, cz;
			n(0) = sin(phi)*cos(theta);
			n(1) = sin(phi)*sin(theta);
			n(2) = cos(phi);

			double costheta = n.cross((p - c).normalized()).norm();
			double pcdot = (p - c).dot(p - c);
			double err2 = r * r + pcdot - 2 * r* sqrt(pcdot)* costheta;
			if (err2 < 0)err2 = 0;
			double err = sqrt(err2);
			residual[0] = err;
			return true;
		}
	private:
		Eigen::Vector3d p;
	};

	ceres::Problem problem;	
	for (int i = 0; i < inlier.size(); i++) {
		CircleFitCostFunc* f = new CircleFitCostFunc(handpoint.at(inlier.at(i)));
		ceres::CostFunction* c = new ceres::NumericDiffCostFunction<CircleFitCostFunc, ceres::CENTRAL, 1, 6>(f);
		problem.AddResidualBlock(c, new ceres::CauchyLoss(0.01), param1);
	}
	ceres::Solver::Options options;
	options.max_num_iterations = 1e3;
	options.function_tolerance = 1e-8;
	options.parameter_tolerance = 1e-8;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	Eigen::Vector3d t_n, t_nt, pt;
	{
	
	t_n(0) = sin(param1[4])*cos(param1[3]);
	t_n(1) = sin(param1[4])*sin(param1[3]);
	t_n(2) = cos(param1[4]);
	pt << param1[0], param1[1], param1[2];
	t_nt = pt - (pt.dot(t_n))*t_n;
	RevLineParameterCvt(t_n, t_nt, jointParam[0], jointParam[1], jointParam[2], jointParam[3]);
	}

	Eigen::Vector3d h0 = handpoint.at(inlier.at(0));
	Eigen::Vector3d hd = (h0) + ( pt - h0).dot(t_n) * t_n;

	double minth = 2*M_PI, maxth = -2 * M_PI;
	for (int i = 0; i < inlier.size(); i++) {
		Eigen::Vector3d qd = (handpoint.at(inlier.at(i))) + (pt - handpoint.at(inlier.at(i))).dot(t_n) * t_n;
		double theta = acos((hd-pt).dot(qd-pt)/((hd - pt).norm()*(qd - pt).norm()));
		if (minth > theta) {
			minth = theta;
		}
		if (maxth < theta) {
			maxth = theta;
		}
	}

	inlier_=std::vector<int>(inlier);
	std::cout << "Movement range = " << maxth - minth << std::endl;
	return maxth - minth;
}


void Param2TFMatrix(JointTypes jt, double* params_joint, double* params_frame, Eigen::Matrix3d& R, Eigen::Vector3d &t) {
	R = Eigen::Matrix3d::Identity();
	t << 0, 0, 0;
	switch (jt) {
	case JointTypes::prismatic:
	{
		//prismatic parameter
		//joint params: 2DoF
		// direction vector:theta, phi
		//amount param:1 dof
		double theta = params_joint[0];
		double phi = params_joint[1];
		double amount = params_frame[0];
		Eigen::Vector3d n;
		n(0) = sin(phi)*cos(theta);
		n(1) = sin(phi)*sin(theta);
		n(2) = cos(phi);
		t = amount * n;
	}
		return;
	case JointTypes::revolute:
	{
		//articulated parameter
		//joint params: 4DoF
		// axis direction vector:theta, phi
		// axis position: theta2, h
		//amount param:1 dof
		double theta = params_joint[0];
		double phi = params_joint[1];
		double r = params_joint[2];
		double theta2 = params_joint[3];
		Eigen::Vector3d n,nt;//axis direction
		LineParameterCvt(theta, phi, r, theta2, n, nt);
		double amount = params_frame[0];
		//Rotation computation
		Eigen::Matrix3d  axist;
		axist << 0, -n(2), n(1),
			n(2), 0, -n(0),
			-n(1), n(0), 0;
		R = Eigen::Matrix3d::Identity() + sin(amount) * axist + (1 - cos(amount))*axist*axist;
		t = - R * nt + nt;
	}
		return;
	case unknown:
		return;
}
}