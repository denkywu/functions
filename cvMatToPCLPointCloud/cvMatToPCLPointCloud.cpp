/**********************************************************************//**
		(1) convert data of cv::Mat format to PCL pointCloud
		(2)	convert data of PCL pointCloud to cv::Mat format

@file		cvMatToPCLPointCloud.cpp
@author		WD
@date		2020/03/02
@brief		convert cv::Mat format to/from pcl::PointCloud<pcl::PointXYZ>::Ptr
**************************************************************************/
#include "cvMatToPCLPointCloud.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/**********************************************************************************************************************//**
@brief		convert cv::Mat format to pcl::PointCloud<pcl::PointXYZ>::Ptr
			点云xyz数据, cv::Mat 转 pcl 格式
@param		const cv::Mat& xyzDepth					        [IN]    pointCloud xyz information in cv::Mat format
@param		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud	    [OUT]	pcl format

@return     bool

@author		WD
@date		2020/03/02
**************************************************************************************************************************/
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // cv::Mat矩阵的数据类型必须是 CV_32FC3
    if(21 != xyzDepth.type())
    {
        std::cerr << "Error in convertMatToPcl()." << std::endl;
		std::cerr << "The type of input data(cv::Mat) must be CV_32FC3." << std::endl;
        return false;
    }
	// cv::Mat数据大小
	auto depthWidth = xyzDepth.cols;
	auto depthHeight = xyzDepth.rows;

	cloud->width = static_cast<uint32_t>(depthWidth);
	cloud->height = static_cast<uint32_t>(depthHeight);
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int iRows = 0; iRows < depthHeight; iRows++)
	{
		for (int iCols = 0; iCols < depthWidth; iCols++, pt++)
		{
			pt->x = xyzDepth.at<cv::Vec3f>(iRows, iCols)[0];
			pt->y = xyzDepth.at<cv::Vec3f>(iRows, iCols)[1];
			pt->z = xyzDepth.at<cv::Vec3f>(iRows, iCols)[2];
		}
	}
	return true;
}



/**********************************************************************************************************************//**
@brief		convert pcl::PointCloud<pcl::PointXYZ>::Ptr to cv::Mat format
			点云xyz数据, pcl 格式 转 cv::Mat
@param		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud	[IN]	pcl format
@param		cv::Mat& xyzDepth									[OUT]	pointCloud xyz information in cv::Mat format

@return		bool

@author		WD
@date		2020/03/02
**************************************************************************************************************************/
bool convertPclToMat(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& xyzDepth)
{
	xyzDepth = cv::Mat(cloud->height, cloud->width, CV_32FC3, 
		cv::Scalar::all(0)).clone();// 初始值0
	// xyzDepth = cv::Mat(cloud->height, cloud->width, CV_32FC3, 
	// 	cv::Scalar::all(std::numeric_limits<float>::quiet_NaN())).clone();// 初始值NaN

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int iRows = 0; iRows < cloud->height; iRows++)
	{
		for (int iCols = 0; iCols < cloud->width; iCols++, pt++)
		{
			xyzDepth.at<cv::Vec3f>(iRows, iCols)[0] = pt->x;
			xyzDepth.at<cv::Vec3f>(iRows, iCols)[1] = pt->y;
			xyzDepth.at<cv::Vec3f>(iRows, iCols)[2] = pt->z;
		}
	}
	return true;
}