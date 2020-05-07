#include "others.h"

std::string getDate() {
	std::time_t t = std::time(nullptr);
	std::ostringstream oss;
	oss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H%M%S"); // Cannot use colon as a file name
	std::string timeDate = oss.str();
	return timeDate;
}

std::vector<std::string> split(std::string input, char delimiter) {
	std::stringstream ss;
	ss << input;
	std::string str;
	std::vector<std::string> ret;
	while (std::getline(ss, str, delimiter)) {
		ret.push_back(str);
	}

	return ret;
}
Eigen::Matrix3d q2dcm(Eigen::Vector4d& q) {
	Eigen::Matrix3d R;

	// Build quaternion element products
	double q1q1 = q(0)*q(0);
	double q1q2 = q(0)*q(1);
	double q1q3 = q(0)*q(2);
	double q1q4 = q(0)*q(3);

	double q2q2 = q(1)*q(1);
	double q2q3 = q(1)*q(2);
	double q2q4 = q(1)*q(3);

	double q3q3 = q(2)*q(2);
	double q3q4 = q(2)*q(3);

	double q4q4 = q(3)*q(3);

	// Build DCM
	R(0, 0) = q1q1 - q2q2 - q3q3 + q4q4;
	R(0, 1) = 2 * (q1q2 + q3q4);
	R(0, 2) = 2 * (q1q3 - q2q4);

	R(1, 0) = 2 * (q1q2 - q3q4);
	R(1, 1) = -q1q1 + q2q2 - q3q3 + q4q4;
	R(1, 2) = 2 * (q2q3 + q1q4);

	R(2, 0) = 2 * (q1q3 + q2q4);
	R(2, 1) = 2 * (q2q3 - q1q4);
	R(2, 2) = -q1q1 - q2q2 + q3q3 + q4q4;

	return R;

}

Eigen::Vector4d dcm2q(Eigen::Matrix3d& dcm) {
	Eigen::Vector4d q;
	if (dcm.trace() > 0) {
		double sr = sqrt(1 + dcm.trace());
		double sr2 = sr * 2;

		q(0) = (dcm(1, 2) - dcm(2, 1)) / sr2;
		q(1) = (dcm(2, 0) - dcm(0, 2)) / sr2;
		q(2) = (dcm(0, 1) - dcm(1, 0)) / sr2;
		q(3) = 0.5*sr;
	}
	else {
		if (dcm(0, 0) > dcm(1, 1) && dcm(0, 0) > dcm(2, 2)) {
			double sr = sqrt(1 + (dcm(0, 0) - (dcm(1, 1) + dcm(2, 2))));
			double sr2 = sr * 2;
			q(3) = (dcm(1, 2) - dcm(2, 1)) / sr2;
			q(2) = (dcm(2, 0) + dcm(0, 2)) / sr2;
			q(1) = (dcm(0, 1) + dcm(1, 0)) / sr2;
			q(0) = 0.5*sr;
		}
		else if (dcm(1, 1) > dcm(2, 2)) {
			double  sr = sqrt(1 + (dcm(1, 1) - (dcm(2, 2) + dcm(0, 0))));
			double  sr2 = 2 * sr;
			q(0) = (dcm(1, 0) + dcm(0, 1)) / sr2;
			q(1) = 0.5 * sr;
			q(2) = (dcm(1, 2) + dcm(2, 1)) / sr2;
			q(3) = (dcm(2, 0) - dcm(0, 2)) / sr2;

		}
		else {
			double  sr = sqrt(1 + (dcm(2, 2) - (dcm(0, 0) + dcm(1, 1))));
			double  sr2 = 2 * sr;
			q(0) = (dcm(2, 0) + dcm(0, 2)) / sr2;
			q(1) = (dcm(1, 2) + dcm(2, 1)) / sr2;
			q(2) = 0.5 * sr;
			q(3) = (dcm(0, 1) - dcm(1, 0)) / sr2;
		}
	}
	return q;

}


void LineParameterCvt(double theta, double phi, double r, double theta2, Eigen::Vector3d& n, Eigen::Vector3d& h){
	//theta,phi to n
	n(0) = sin(phi)*cos(theta);
	n(1) = sin(phi)*sin(theta);
	n(2) = cos(phi);
	//compute R where n=Rz
	Eigen::Vector3d xrevoluted, z, nd; z << 0, 0, 1; xrevoluted << cos(theta2), sin(theta2), 0;
	nd = z.cross(n);
	double sint = nd.norm();
	

	if (sint == 0) {
		h =r*xrevoluted;
		return;
	}
	nd = nd.normalized();
	Eigen::Matrix3d R,axist;
	axist << 0 ,-nd(2),nd(1),
		nd(2),0,-nd(0),
		-nd(1),nd(0),0;
	double cost = z.dot(n);
	R = Eigen::Matrix3d::Identity() + sint* axist + (1-cost)*axist*axist;
	h = r * R*xrevoluted;
	return;
}

void RevLineParameterCvt(Eigen::Vector3d n, Eigen::Vector3d h, double& theta, double& phi, double& r, double& theta2) {
	r = h.norm();//output 1
	if (r == 0) {
		theta2 = 0;
	}
	else {
		Eigen::Vector3d xrevoluted, z, nd; z << 0, 0, 1;
		nd = z.cross(n);
		double sint = nd.norm();
		Eigen::Matrix3d R, axist;

		if (sint == 0) {
			R = Eigen::Matrix3d::Identity();
		}
		else {
			nd = nd.normalized();
			axist << 0, -nd(2), nd(1),
				nd(2), 0, -nd(0),
				-nd(1), nd(0), 0;
			double cost = z.dot(n);
			R = Eigen::Matrix3d::Identity() + sint * axist + (1- cost) * axist*axist;
		}
		xrevoluted =R.transpose()*h.normalized();
		theta2 = atan2(xrevoluted(1), xrevoluted(0));
	}//output2

	theta = atan2(n(1), n(0));
	phi = acos( n(2));
	return;
}

void LineParameterCvt(double* params, Eigen::Vector3d& n, Eigen::Vector3d& h) {
	LineParameterCvt(params[0], params[1], params[2], params[3], n, h);
}

Eigen::VectorXd solveSimultaneousEquation(Eigen::MatrixXd A, Eigen::VectorXd B) {
	Eigen::VectorXd ans(A.rows());
	ans = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
	return ans;
};


void RemoveNormalPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr o_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr o_normals(new pcl::PointCloud<pcl::Normal>);

	for (int x = 0; x < cloud->width;x++) {
		for (int y = 0; y < cloud->height; y++) {

			if ((*cloud)(x, y).z <= 0 ||
				isnan((*normals)(x, y).normal_x)
				)continue;

			o_cloud->push_back((*cloud)(x, y));
			o_normals->push_back((*normals)(x, y));
		}
	}
	cloud->clear();
	cloud->insert(cloud->begin(), o_cloud->begin(), o_cloud->end());
	normals->clear();
	normals->insert(normals->begin(), o_normals->begin(), o_normals->end());
}

void RemoveNormalPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, pcl::PointCloud<pcl::Normal>::Ptr normals) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr o_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr o_normals(new pcl::PointCloud<pcl::Normal>);

	for (int x = 0; x < cloud->width; x++) {
		for (int y = 0; y < cloud->height; y++) {

			if ((*cloud)(x, y).z <= 0 ||
				isnan((*normals)(x, y).normal_x)
				)continue;


			o_cloud_color->push_back((*cloud_color)(x, y));
			o_cloud->push_back((*cloud)(x, y));
			o_normals->push_back((*normals)(x, y));
		}
	}
	cloud->clear();
	cloud->insert(cloud->begin(), o_cloud->begin(), o_cloud->end());
	cloud_color->clear();
	cloud_color->insert(cloud_color->begin(), o_cloud_color->begin(), o_cloud_color->end());
	normals->clear();
	normals->insert(normals->begin(), o_normals->begin(), o_normals->end());
}
