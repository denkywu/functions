/**********************************************************************//**
		    calculate Normals of PointCloud based on PCL

@file		calculateNormals.cpp
@author		WD
@date		2020/03/02
@brief		calculate Normals of PointCloud based on PCL 
            using CPU and OpenMP.
**************************************************************************/
#include "calculateNormals.h"

// C++
#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>



/**********************************************************************************************************************//**
@brief		法线估计: 基于 CPU 和 OpenMP加速
@param		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud		[IN]	输入点云xyz数据
@param		const double scale										[IN]	搜索半径
@param		pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals	[OUT] 	法线估计结果

@return		void

@author		WD
@date		2020/02/24
**************************************************************************************************************************/
void calNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double scale, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
#if WD_CALCULATE_NORMALS_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest0 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedNaN(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_removedNaN, index);
#if WD_CALCULATE_NORMALS_DEBUG == 1
	std::cout << "Before remove NaN, point size: " << cloud->points.size() << std::endl;
	std::cout << "After remove NaN, point size: " << cloud_removedNaN->points.size() << std::endl;
#endif

	// Create a search tree, use KDTreee for non-organized data.
	pcl::search::Search<pcl::PointXYZ>::Ptr tree;
	if (cloud_removedNaN->isOrganized())
	{
		tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
	}
	else
	{
#if WD_CALCULATE_NORMALS_DEBUG == 1
		std::cout << "PointCloud data is not organized!" << std::endl;
#endif
		tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(true));
		/*
			pcl::search::KdTree<pcl::PointXYZ>(in)
			[in]sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). 
				False otherwise. 
		*/
	}

	// Set the input pointcloud for the search tree
	tree->setInputCloud(cloud_removedNaN);

	//
	// Compute normals at each point
	//
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne(4);// 使用 OpenMP standard 加速法线估计
	// pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud(cloud_removedNaN);
	ne.setSearchMethod(tree);

	/**
	 * NOTE: setting viewpoint is very important, so that we can ensure
	 * normals are all pointed in the same direction!
	 */
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	// calculate normals with the given scale
#if WD_CALCULATE_NORMALS_DEBUG == 1
	std::cout << "Calculating normals for scale..." << scale << std::endl;
#endif
	pcl::copyPointCloud(*cloud_removedNaN, *cloud_normals);// 将cloud中点云xyz信息copy到后者
	ne.setRadiusSearch(scale);
	ne.compute(*cloud_normals);

#if WD_CALCULATE_NORMALS_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
	std::cout << "[calNormals] Cost: " << tTest1.time_since_epoch().count() - tTest0.time_since_epoch().count() << " ms" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_SAVE_FILES == 1
    pcl::io::savePCDFile("TestWD/pointNormals.pcd", *cloud_normals);
	std::cout << "Done: save pointNormals.pcd" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_DEBUG == 1
    std::cout << "Done: calNormals()." << std::endl;
#endif

	return;
}