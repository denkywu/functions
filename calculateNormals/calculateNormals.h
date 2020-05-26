/**********************************************************************//**
		    calculate Normals of PointCloud based on PCL

@file		calculateNormals.h
@author		WD
@date		2020/03/02
@brief		Statement for "calculateNormals.cpp"
**************************************************************************/
#ifndef _CALCULATE_NORMALS_H_
#define _CALCULATE_NORMALS_H_

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// macro
// 计时
#define WD_CALCULATE_NORMALS_CALC_TIME 1
// 保存文件到本地
#define WD_CALCULATE_NORMALS_SAVE_FILES 0
// Debug，主要显示过程信息
#define WD_CALCULATE_NORMALS_DEBUG 1

// 法线估计: 基于 CPU 和 OpenMP加速
void calNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double scale, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);


#endif// _CALCULATE_NORMALS_H_