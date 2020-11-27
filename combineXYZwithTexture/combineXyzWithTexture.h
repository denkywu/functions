/**********************************************************************//**
            combine XYZ pointCloud with Texture image

@file		combineXyzWithTexture.h
@author	WD
@date		2020/06/17
@brief	Statement for "combineXyzWithTexture.cpp"
**************************************************************************/
#ifndef _COMBINE_XYZ_WITH_TEXTURE_H_
#define _COMBINE_XYZ_WITH_TEXTURE_H_

// C++
#include <iostream>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>// PCL支持的点类型头文件
#include <pcl/io/io.h>


bool combineXyzWithTexture(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
      const cv::Mat& textureImage, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
);


#endif // _COMBINE_XYZ_WITH_TEXTURE_H_