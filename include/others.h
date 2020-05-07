#include "main.h"

void print4x4Matrix(const Eigen::Matrix4d & matrix);
std::string getDate();
std::vector<std::string> split(std::string input,char delimiter);
Eigen::Matrix3d q2dcm(Eigen::Vector4d& q);
Eigen::Vector4d dcm2q(Eigen::Matrix3d& dcm);

void LineParameterCvt(double theta,double phi,double r,double theta2,Eigen::Vector3d& n, Eigen::Vector3d& h);
void LineParameterCvt(double* params, Eigen::Vector3d& n, Eigen::Vector3d& h);
void RevLineParameterCvt(Eigen::Vector3d n, Eigen::Vector3d h, double& theta, double& phi, double& r, double& theta2);


Eigen::VectorXd solveSimultaneousEquation(Eigen::MatrixXd A, Eigen::VectorXd B);

void RemoveNormalPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
void RemoveNormalPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, pcl::PointCloud<pcl::Normal>::Ptr normals);
