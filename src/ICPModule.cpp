#include <ICPModule.h>

double SimultaneousICPModule::InitialPointComputation(Eigen::Vector3d handPosition) {

		struct InitialParamFunc {
		public:
			InitialParamFunc(double* params_,JointTypes jt_, Eigen::Vector3d src_, Eigen::Vector3d dst_) {
				params=params_;
				jt = jt_;
				src = src_;
				dst = dst_;
			}
			bool operator()(const double* parameters, double* residual) const {
				double theta = parameters[0];
				Eigen::Matrix3d R;
				Eigen::Vector3d t;
				Param2TFMatrix(jt,params,&theta,R,t);
				residual[0] = (dst-(R*src+t)).norm();
				return true;
			}
		private:
			double* params;
			JointTypes jt;
			Eigen::Vector3d src, dst;
		};

		ceres::CostFunction* c = new ceres::NumericDiffCostFunction<InitialParamFunc, ceres::CENTRAL, 1, 1>
			(new InitialParamFunc(jointParams, jt, handPosition, BaseHandPoint));
		ceres::Problem problem;
		double ret = 0;
		problem.AddResidualBlock(c, NULL, &ret);
		
		ceres::Solver::Options options;
		options.max_num_iterations = 1e3;
		options.function_tolerance = 1e-8;
		options.parameter_tolerance = 1e-8;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		if (jt == revolute) {
			while (ret > M_PI) {
				ret -= 2 * M_PI;
			}
			while (ret < -M_PI) {
				ret += 2 * M_PI;
			}
		}
	return ret;
}

void SimultaneousICPModule::ICP_run() {
	
	//initial parameter setting and estimation
	if (handSet) {
		if (jt == prismatic) {
			amountParams = new double[clouds.size()];
			for (int i = 0; i < clouds.size(); i++) {
				if (HandPoints.at(i)(2) < 0) {
					if (i == 0)continue;
					amountParams[i] = amountParams[i - 1];
				}
				else {
					amountParams[i] = InitialPointComputation(HandPoints.at(i));
				}
			}
		}
		else if (jt == revolute) {
			amountParams = new double[clouds.size()];
			for (int i = 0; i < clouds.size(); i++) {
				if (HandPoints.at(i)(2) < 0) {
					if (i == 0)continue;
					amountParams[i] = amountParams[i - 1];
				}
				else {
					amountParams[i] = InitialPointComputation(HandPoints.at(i));
				}
			}
		}
	}
	else {
		amountParams = new double[clouds.size()];
		for (int i = 0; i < clouds.size(); i++) {
			amountParams[i] = 0;	
		}
		if (jt == prismatic) {
			jointParams = new double[2];
			jointParams[0] = jointParams[1] = 0;
		}
		else if (jt == revolute) {
			jointParams = new double[4];
			jointParams[0] = jointParams[1] = jointParams[2] = jointParams[3] = 0;
		}
	}

	//make KD-Tree & downsample
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtrees;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_o;
	kdtree_o.setInputCloud(originFrame);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ds_cloud;

	// Create the filtering object

	for (int i = 0; i < clouds.size(); i++) {
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(clouds.at(i));
		kdtrees.push_back(kdtree);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr_filt;
		cloudptr_filt.reset(new pcl::PointCloud<pcl::PointXYZ>);
		sor.setInputCloud(clouds.at(i));
		sor.setLeafSize(0.05f, 0.05f, 0.05f);
		sor.filter(*cloudptr_filt);
		ds_cloud.push_back(cloudptr_filt);
	}

	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointIdxNKNSquaredDistance(K);


	double dummy=0;

	for (int i = 0; i < maxitr; i++) {
		cout << "iteration "<< i << endl;
		double squaredThresh = dist_thresh * dist_thresh;
		ceres::Problem problem;
		for(int j=0;j<ds_cloud.size();j++){
			std::vector<Eigen::Matrix4d> transformationMatrices;
			for (int targ = 0; targ < kdtrees.size(); targ++) {
				Eigen::Matrix4d transformationMatrix, m1 = Eigen::Matrix4d::Identity(), m2 = Eigen::Matrix4d::Identity();
				Eigen::Matrix3d R, R2;
				Eigen::Vector3d t, t2;
				Param2TFMatrix(jt, jointParams, &amountParams[j], R, t);
				Param2TFMatrix(jt, jointParams, &amountParams[targ], R2, t2);

				m1.block(0, 0, 3, 3) = R;
				m1.block(0, 3, 3, 1) = t;
				m2.block(0, 0, 3, 3) = R2;
				m2.block(0, 3, 3, 1) = t2;

				transformationMatrix = m2.inverse()*m1;

				transformationMatrices.push_back(transformationMatrix);
			}
			for (int l = 0; l < ds_cloud.at(j)->size(); l++) {
				pcl::PointXYZ searchPoint = ds_cloud.at(j)->at(l),searchPoint_;
				Eigen::Vector4d sp;
				std::vector<int> corresponds;
				std::vector<double> dist;
				for (int targ = 0; targ < kdtrees.size(); targ++) {
					if (j == targ)continue;
					if (i<maxitr-3 && targ != 0 && abs(j-targ) >=2)continue;
					sp << searchPoint.x, searchPoint.y, searchPoint.z, 1;
					sp = transformationMatrices[targ] * sp;
					searchPoint_.x = sp(0);
					searchPoint_.y = sp(1);
					searchPoint_.z = sp(2);
					if (kdtrees.at(targ).nearestKSearch(searchPoint_, K, pointIdxNKNSearch, pointIdxNKNSquaredDistance) > 0) {
						if (pointIdxNKNSquaredDistance.at(0) < squaredThresh) {
							corresponds.push_back(targ);
							corresponds.push_back(pointIdxNKNSearch.at(0));
							corresponds.push_back(l);
							dist.push_back(pointIdxNKNSquaredDistance.at(0));

						}
					};
				}
				if (dist.size() == 0)continue;

				for (int m = 0; m < dist.size(); m++) {

					int targ = corresponds.at(3 * m);
					pcl::PointXYZ sourcePoint = clouds.at(targ)->at(corresponds.at(3 * m + 1));
					pcl::PointXYZ targetPoint = ds_cloud.at(j)->at(l);
					Eigen::Vector3d source_, target_;
					source_ << sourcePoint.x, sourcePoint.y, sourcePoint.z;
					target_ << targetPoint.x, targetPoint.y, targetPoint.z;
					ArticulationClosestNNFunc* f = new ArticulationClosestNNFunc(source_, target_, jt);

					double w1 = 1.0, w2 = 1.0;
					if (handSet) {
						if (HandPoints.at(targ)(2) > 0) {
							w1 = 1 / (0.2 + (source_ - HandPoints.at(targ)).norm());
						}
						if (HandPoints.at(j)(2) > 0) {
							w2 = 1 / (0.2 + (target_ - HandPoints.at(j)).norm());
						}
					}
					f->setWeight(w1, w2);
					double w12 = w1*w2;
					ceres::CostFunction* c = new ceres::NumericDiffCostFunction<ArticulationClosestNNFunc, ceres::CENTRAL, 1, 4, 1, 1>(f);
					if (j == 0) {
						f->setAnchor(1);
						problem.AddResidualBlock(c, new ceres::CauchyLoss(0.03*w12), jointParams, &dummy, &amountParams[targ]);
					}
					else if (targ == 0) {
						f->setAnchor(2);
						problem.AddResidualBlock(c, new ceres::CauchyLoss(0.03*w12), jointParams, &amountParams[j], &dummy);
					}
					else {
						problem.AddResidualBlock(c, new ceres::CauchyLoss(0.03*w12), jointParams, &amountParams[j], &amountParams[targ]);
					}
				}
			}
		}	
		ceres::Solver::Options options;
		options.max_num_iterations = 1e2;
		options.function_tolerance = 1e-5;
		options.parameter_tolerance = 1e-5;
		//options.linear_solver_type = ceres::DENSE_QR;
		ceres::Solver::Summary summary;
		options.num_threads=24;
		ceres::Solve(options, &problem, &summary);
		dist_thresh = 0.45 * (maxitr-(i+1))/maxitr + 0.05;
	}
	
	return;
}


std::vector<int> SimultaneousICPModule::Segmentation_run(
	double dynamic_thresh, double symmetric_thresh, double cluster_param) {
	//make KD-Tree & downsample

	//segmentation
	std::vector<Eigen::Matrix4d> transformationMatrices;
	for (int targ = 0; targ < kdtrees.size(); targ++) {
		Eigen::Matrix4d transformationMatrix, m1 = Eigen::Matrix4d::Identity(), m2 = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d R, R2;
		Eigen::Vector3d t, t2;
		Param2TFMatrix(jt, jointParams, &amountParams[targ], R2, t2);

		m2.block(0, 0, 3, 3) = R2;
		m2.block(0, 3, 3, 1) = t2;

		transformationMatrix = m2.inverse();
		transformationMatrices.push_back(transformationMatrix);
	}

	double segThresh = 0.03;
	double nrmThresh = 0.08;
	double hand_cluster_thresh = 0.01;
	std::vector<int> symmetIdx;

	for (int l = 0; l < originFrame->size(); l++) {
		pcl::PointXYZ searchPoint = originFrame->at(l), searchPoint_;
		pcl::Normal searchNormal = background_normal_full->at(l), searchNormal_;
		Eigen::Vector4d sp;
		Eigen::Vector3d sn, sn_, to_sp;
		std::vector<double> thresh_v, thresh_v2;
		int K = 1;
		for (int targ = 1; targ < kdtrees.size(); targ++) {
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointIdxNKNSquaredDistance(K);
			sp << searchPoint.x, searchPoint.y, searchPoint.z, 1;
			sn << searchNormal.normal_x, searchNormal.normal_y, searchNormal.normal_z;
			sp = transformationMatrices[targ] * sp;
			sn_ = transformationMatrices[targ].block(0, 0, 3, 3) * sn;

			//incident angle computation
			to_sp = sp.segment(0, 3).normalized();
			if (to_sp.dot(sn_) > -0.25) {
				continue;
			}

			searchPoint_.x = sp(0);
			searchPoint_.y = sp(1);
			searchPoint_.z = sp(2);
			if (kdtrees.at(targ).nearestKSearch(searchPoint_, K, pointIdxNKNSearch, pointIdxNKNSquaredDistance) > 0) {
				searchNormal_ = clouds_normal_full.at(targ)->at(pointIdxNKNSearch.at(0));
				double dotProd = sn_(0)*searchNormal_.normal_x + sn_(1)*searchNormal_.normal_y + sn_(2)*searchNormal_.normal_z;
				double offset = dotProd > 1 - nrmThresh ? 0.0 : 1;
				thresh_v.push_back(pointIdxNKNSquaredDistance.at(0) + offset);
			};
		}
		std::sort(thresh_v.begin(), thresh_v.end());
		if (thresh_v.size() >=2 && thresh_v.at(thresh_v.size() / 2) < symmetric_thresh*symmetric_thresh) {
			symmetIdx.push_back(l);
		}
	}

	symm_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<int> inliers;
	for (int i = 0; i < symmetIdx.size(); i++) {
		symm_pc->push_back(originFrame->at(symmetIdx.at(i)));
		inliers.push_back(symmetIdx.at(i));
	}
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(symm_pc);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_param); 
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(symm_pc);
	ec.extract(cluster_indices);

	pcl::PointXYZ handP_pcl;
	handP_pcl.x = BaseHandPoint(0);
	handP_pcl.y = BaseHandPoint(1);
	handP_pcl.z = BaseHandPoint(2);
	std::vector<int> idx(1);
	std::vector<float> dist(1);
	tree->nearestKSearch(handP_pcl,1,idx,dist);

	int j = 0;
	segm_cluster_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<int> cluster_idx;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		bool nearest_ = false;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> cloud_idx;
		double minDist2 = 1.0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(symm_pc->points[*pit]);
			cloud_idx.push_back(inliers.at(*pit));

			double dist2 = (handP_pcl.x- symm_pc->points[*pit].x) * (handP_pcl.x - symm_pc->points[*pit].x)
				+ (handP_pcl.y - symm_pc->points[*pit].y) * (handP_pcl.y - symm_pc->points[*pit].y)
				+ (handP_pcl.z - symm_pc->points[*pit].z) * (handP_pcl.z - symm_pc->points[*pit].z);
			if (dist2 < minDist2) {
				minDist2 = dist2;
			}
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		//std::cout << "Min dist: " << minDist2  << std::endl;
 
 		if (minDist2 < hand_cluster_thresh) {
			segm_cluster_pc->insert(segm_cluster_pc->end(), cloud_cluster->begin(), cloud_cluster->end());
			cluster_idx.insert(cluster_idx.end(), cloud_idx.begin(), cloud_idx.end());
		}
		j++;
	}
	return cluster_idx;
}

void SimultaneousICPModule::outPointCloud(std::string fileFolder){
	pcl::PLYWriter plywriter;
	for(int i=0;i<clouds.size();i++){
		pcl::PointCloud<pcl::PointXYZ> transformed;
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		Param2TFMatrix(jt,jointParams,&amountParams[i],R,t);
		Eigen::Matrix4d transformation_matrix;
		transformation_matrix.block(0,0,3,3)=R;
		transformation_matrix.block(0,3,3,1)=t;
		transformation_matrix.block(3, 0, 1, 4)<<0,0,0,1;

		pcl::transformPointCloud(*clouds.at(i), transformed, transformation_matrix);
		std::stringstream ss;ss<<fileFolder<<"/moved_"<<i<<".ply";
		plywriter.write<pcl::PointXYZ>(ss.str(), transformed, true);
	}	
	//plywriter.write<pcl::PointXYZ>(fileFolder + "/back.ply", *originFrame, true);
	plywriter.write<pcl::PointXYZ>(fileFolder + "/symm.ply", *symm_pc, true);
	plywriter.write<pcl::PointXYZ>(fileFolder + "/segm.ply", *segm_cluster_pc, true);
}

void SimultaneousICPModule::resultVisualize(pcl::visualization::PCLVisualizer* viewer) {
	for (int i = 0; i < clouds.size(); i++) {
		// Original point cloud is white
		std::stringstream ss;
		ss << "cloud_"<<i;
		viewer->addPointCloud(clouds.at(i), ss.str());
	}
}

void SimultaneousICPModule::articulationVisualize(pcl::visualization::PCLVisualizer* viewer, std::string idtag, double r, double g, double b, bool showRange) {
	if (jt == revolute) {
		Eigen::Vector3d n, a;
		LineParameterCvt(jointParams, n, a);
		Eigen::Vector3d p1, p2;
		p1 = a + 3.0*n;
		p2 = a - 3.0*n;
		pcl::PointXYZ p1_, p2_;

		p1_.x = p1(0);		p1_.y = p1(1);		p1_.z = p1(2);
		p2_.x = p2(0);		p2_.y = p2(1);		p2_.z = p2(2);

		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);    // We need 7 values
		cylinder_coeff.values[0] = p2_.x;
		cylinder_coeff.values[1] = p2_.y;
		cylinder_coeff.values[2] = p2_.z;
		cylinder_coeff.values[3] = 6 * n(0);
		cylinder_coeff.values[4] = 6 * n(1);
		cylinder_coeff.values[5] = 6 * n(2);
		cylinder_coeff.values[6] = 0.005;
		viewer->addCylinder(cylinder_coeff, idtag);

		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, idtag, 0);//articulation visualizetion

		if (!showRange) {
			visualizeCalledCnt++;
			return;
		}
		// range visualization
		double* minmax = getMovementRange();
		Eigen::Vector3d hb = HandPoints.at(0), prevp, curp;
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		bool isFirst = true;
		for (double i = minmax[0]; i <= minmax[1]; i += (minmax[1] - minmax[0]) / 20.0) {
			Param2TFMatrix(jt, jointParams, &i, R, t);
			curp = R.transpose() * (hb - t);
			if (isFirst) {
				isFirst = false;
			}
			else {
				pcl::ModelCoefficients cylinder_coeff_arc;
				cylinder_coeff_arc.values.resize(7);    // We need 7 values
				cylinder_coeff_arc.values[0] = curp(0);
				cylinder_coeff_arc.values[1] = curp(1);
				cylinder_coeff_arc.values[2] = curp(2);
				cylinder_coeff_arc.values[3] = prevp(0) - curp(0);
				cylinder_coeff_arc.values[4] = prevp(1) - curp(1);
				cylinder_coeff_arc.values[5] = prevp(2) - curp(2);
				cylinder_coeff_arc.values[6] = 0.005;
				viewer->addCylinder(cylinder_coeff_arc, idtag + "_range" + std::to_string(i));

				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, idtag + "_range" + std::to_string(i), 0);//articulation visualizetion
			}
			prevp = curp;
		}
		free(minmax);
	}
	else {
		pcl::PointXYZ p1, p2;

		p1.x = BaseHandPoint(0);
		p1.y = BaseHandPoint(1);
		p1.z = BaseHandPoint(2);
		double jointParams1 = jointParams[1];
		if (cos(jointParams1) < 0)jointParams1 = M_PI + jointParams1;

		p2.x = -sin(jointParams1) * cos(jointParams[0]) + p1.x;
		p2.y = -sin(jointParams1) * sin(jointParams[0]) + p1.y;
		p2.z = -cos(jointParams1) + p1.z;

		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);    // We need 7 values
		cylinder_coeff.values[0] = p1.x;
		cylinder_coeff.values[1] = p1.y;
		cylinder_coeff.values[2] = p1.z;
		cylinder_coeff.values[3] = -sin(jointParams1) * cos(jointParams[0]);
		cylinder_coeff.values[4] = -sin(jointParams1) * sin(jointParams[0]);
		cylinder_coeff.values[5] = -cos(jointParams1);
		cylinder_coeff.values[6] = 0.005;
		viewer->addCylinder(cylinder_coeff, idtag);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, idtag, 0);
		// range visualization
		if (!showRange) {
			visualizeCalledCnt++;
			return;
		}
		double* minmax = getMovementRange();
		Eigen::Vector3d hb = HandPoints.at(0), prevp, curp;
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		pcl::ModelCoefficients cylinder_coeff_arc;
		Param2TFMatrix(jt, jointParams, &minmax[0], R, t);
		curp = R.transpose() * (hb - t);
		Param2TFMatrix(jt, jointParams, &minmax[1], R, t);
		prevp = R.transpose() * (hb - t);

		cylinder_coeff_arc.values.resize(7);    // We need 7 values
		cylinder_coeff_arc.values[0] = curp(0);
		cylinder_coeff_arc.values[1] = curp(1);
		cylinder_coeff_arc.values[2] = curp(2);
		cylinder_coeff_arc.values[3] = prevp(0) - curp(0);
		cylinder_coeff_arc.values[4] = prevp(1) - curp(1);
		cylinder_coeff_arc.values[5] = prevp(2) - curp(2);
		cylinder_coeff_arc.values[6] = 0.007;
		viewer->addCylinder(cylinder_coeff_arc, idtag + "_range");

		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, idtag + "_range", 0);//articulation visualizetion

	}

	visualizeCalledCnt++;
}




void SimultaneousICPModule::ICP_refine_run() {
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtrees;
	for (int i = 0; i < clouds.size(); i++) {
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(clouds.at(i));
		kdtrees.push_back(kdtree);
	}
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointIdxNKNSquaredDistance(K);


	double dummy = 0;
	dist_thresh = 0.04;

	for (int i = 0; i < maxitr; i++) {
		cout << "iteration " << i << endl;
		double squaredThresh = dist_thresh * dist_thresh;
		ceres::Problem problem;
			std::vector<Eigen::Matrix4d> transformationMatrices;
			for (int targ = 0; targ < kdtrees.size(); targ++) {
				Eigen::Matrix4d transformationMatrix, m1 = Eigen::Matrix4d::Identity(), m2 = Eigen::Matrix4d::Identity();
				Eigen::Matrix3d R, R2;
				Eigen::Vector3d t, t2;
				Param2TFMatrix(jt, jointParams, &amountParams[0], R, t);
				Param2TFMatrix(jt, jointParams, &amountParams[targ], R2, t2);

				m1.block(0, 0, 3, 3) = R;
				m1.block(0, 3, 3, 1) = t;
				m2.block(0, 0, 3, 3) = R2;
				m2.block(0, 3, 3, 1) = t2;

				transformationMatrix = m2.inverse()*m1;
				transformationMatrices.push_back(transformationMatrix);
			}
			int ptcnt = 0;
			for (int l = 0; l < segm_cluster_pc->size(); l++) {
				ptcnt++;
				pcl::PointXYZ searchPoint = segm_cluster_pc->at(l), searchPoint_;
				Eigen::Vector4d sp;
				std::vector<int> corresponds;
				std::vector<double> dist;
				for (int targ = 1; targ < kdtrees.size(); targ++) {
					sp << searchPoint.x, searchPoint.y, searchPoint.z, 1;
					sp = transformationMatrices[targ] * sp;
					searchPoint_.x = sp(0);
					searchPoint_.y = sp(1);
					searchPoint_.z = sp(2);
					if (kdtrees.at(targ).nearestKSearch(searchPoint_, K, pointIdxNKNSearch, pointIdxNKNSquaredDistance) > 0) {
						if (pointIdxNKNSquaredDistance.at(0) < squaredThresh) {
							corresponds.push_back(targ);
							corresponds.push_back(pointIdxNKNSearch.at(0));
							corresponds.push_back(l);
							dist.push_back(pointIdxNKNSquaredDistance.at(0));
						}
					};
				}
				if (dist.size() == 0)continue;

				for (int m = 0; m < dist.size(); m++) {
					int targ = corresponds.at(3 * m);
					pcl::PointXYZ sourcePoint = clouds.at(targ)->at(corresponds.at(3 * m + 1));
					pcl::PointXYZ targetPoint = segm_cluster_pc->at(l);
					Eigen::Vector3d source_, target_;
					source_ << sourcePoint.x, sourcePoint.y, sourcePoint.z;
					target_ << targetPoint.x, targetPoint.y, targetPoint.z;
					ArticulationClosestNNFunc* f = new ArticulationClosestNNFunc(source_, target_, jt);

					ceres::CostFunction* c = new ceres::NumericDiffCostFunction<ArticulationClosestNNFunc, ceres::CENTRAL, 1, 4, 1, 1>(f);
					{
						f->setAnchor(1);
						problem.AddResidualBlock(c, new ceres::CauchyLoss(0.03), jointParams, &dummy, &amountParams[targ]);
					}
				}
			}
			for (int k = 0; k < 21; k++) {
				for (int j = 1; j < hand_point_det_sub.size(); j++) {
					Eigen::Vector3d handpt = transformationMatrices[j].block(0, 0, 3, 3) * hand_point_det_sub.at(0).at(k) + transformationMatrices[j].block(0, 3, 3, 1);

					if ((hand_point_det_sub.at(j).at(k) - handpt).norm() < dist_thresh) {
						ArticulationClosestNNFunc* f = new ArticulationClosestNNFunc(hand_point_det_sub.at(j).at(k), hand_point_det_sub.at(0).at(k), jt);
						ceres::CostFunction* c = new ceres::NumericDiffCostFunction<ArticulationClosestNNFunc, ceres::CENTRAL, 1, 4, 1, 1>(f);
						{
							f->setAnchor(1);
							f->setWeight(hand_score_det_sub.at(0).at(k) * sqrt(ptcnt) * 1e-1, hand_score_det_sub.at(j).at(k));//lambda: 1e-1*1e-1
							problem.AddResidualBlock(c, NULL, jointParams, &dummy, &amountParams[j]);
						}
					}
				}
			}

		ceres::Solver::Options options;
		options.max_num_iterations = 1e2;
		options.function_tolerance = 1e-7;
		options.parameter_tolerance = 1e-7;
		ceres::Solver::Summary summary;
		options.num_threads = 24;
		ceres::Solve(options, &problem, &summary);
		dist_thresh = 0.03 * (maxitr - (i + 1)) / maxitr + 0.01;
	}

	return;

}