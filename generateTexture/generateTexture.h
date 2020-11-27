/**********************************************************************//**
                  generate texture image

@file		generateTexture.h
@author     WD
@date		2020/06/19
@brief	    Statement for "generateTexture.cpp"
**************************************************************************/
#ifndef _GENERATE_TEXTURE_H_
#define _GENERATE_TEXTURE_H_

// C++
#include <iostream>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>// PCL支持的点类型头文件
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>


bool generateTexture(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, // 点云
      const cv::Mat& colorImage, // 2D彩色图像
      const cv::Mat& transMatFrame2dTo3d, // 外参 (齐次坐标), 详细说明见源文件
      const cv::Mat& intrMatColor, // 2D相机的内参
      const cv::Mat& discoefColor, // 2D相机的畸变系数
      cv::Mat& textureImage, // 点云纹理彩色图像
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud // XYZRGB 点云
);


// 基于 source 相机中的点云 xyz 提取 target 相机中的 uv 像素坐标
void extract2dImage(
    const std::vector<cv::Point3f>& xyzPoints, 
    const cv::Mat& transMat, 
    const cv::Mat& intrMatTarget, 
    const cv::Mat& discoefTarget,
    cv::Size& sizeTarget, 
    std::vector<cv::Point2i>& uvPoints
);


#endif // _GENERATE_TEXTURE_H_