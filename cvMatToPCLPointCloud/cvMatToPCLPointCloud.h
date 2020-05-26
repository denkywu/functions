/**********************************************************************//**
		(1) convert data of cv::Mat format to PCL pointCloud
		(2)	convert data of PCL pointCloud to cv::Mat format

@file		cvMatToPCLPointCloud.h
@author		WD
@date		2020/03/02
@brief		Statement for "cvMatToPCLPointCloud.cpp"
**************************************************************************/
#ifndef _CVMAT_TO_PCL_POINTCLOUD_H_
#define _CVMAT_TO_PCL_POINTCLOUD_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// convert data of cv::Mat format to PCL pointCloud
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

// convert data of PCL pointCloud to cv::Mat format
bool convertPclToMat(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& xyzDepth);

#endif // _CVMAT_TO_PCL_POINTCLOUD_H_