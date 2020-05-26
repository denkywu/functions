/**********************************************************************//**
		[GPU Version] 
			calculate Normals of PointCloud based on PCL

@file		calculateNormalsOnGpu.h
@author		WD
@date		2020/03/02
@brief		Statement for "calculateNormalsOnGpu.cpp"
**************************************************************************/
#ifndef _CALCULATE_NORMALS_ON_GPU_H_
#define _CALCULATE_NORMALS_ON_GPU_H_

// C++
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV
#include <opencv2/opencv.hpp>

// macro
// 计时
#define WD_CALCULATE_NORMALS_ON_GPU_CALC_TIME 1
// 保存文件到本地
#define WD_CALCULATE_NORMALS_ON_GPU_SAVE_FILES 0
// Debug，主要显示过程信息
#define WD_CALCULATE_NORMALS_ON_GPU_DEBUG 1


// 版本1: 输入输出 pcl 格式
bool calNormalsOnGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float radius, const unsigned int max_elements, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);
// 版本2: 输入输出 Mat 矩阵
bool calNormalsOnGpu(const cv::Mat& matCloud, const float radius, const unsigned int max_elements, cv::Mat& matNormals);
void warpVec2Cloud(const std::vector<pcl::PointXYZ>& vecPoint, const uint32_t height, const uint32_t width, pcl::PointCloud<pcl::Normal>::Ptr& pclNormal);
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void warpVec2Mat(
    const std::vector<pcl::PointXYZ>& vecPoint, // 这是pcl基于gpu估计的法线（去除了NaN后的计算结果）
    const std::vector<int>& index, // 这是和去除NaN之前的点云cloud的对应索引关系
    cv::Mat& matNormal // 恢复NaN，存放在Mat矩阵中的法线估计结果(利用index)
);
bool convertMatToPclNormal(const cv::Mat& matNormal, pcl::PointCloud<pcl::Normal>::Ptr& pclNormal);


#endif// _CALCULATE_NORMALS_ON_GPU_H_