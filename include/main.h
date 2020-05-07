#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/organized_pointcloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/features/integral_image_normal.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <kinect.h>

#if ENABLE_KINECT_V1
	#include <NuiApi.h>
#endif

#include <windows.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <wrl/client.h>
#include <ctime>

typedef pcl::PointXYZRGBA PointC;
typedef pcl::PointXYZ PointM;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointM> PointCloudM;
typedef pcl::PointCloud<PointN> PointCloudN;

