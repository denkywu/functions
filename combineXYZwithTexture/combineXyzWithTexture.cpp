/**********************************************************************//**
            combine XYZ pointCloud with Texture image

@file		combineXyzWithTexture.cpp
@author		WD
@date		2020/06/17
@brief		generate XYZRGB pointCloud by combining XYZ pointCloud with Texture image
**************************************************************************/
#include "combineXyzWithTexture.h"



/**********************************************************************************************************************//**
@brief		generate XYZRGB pointCloud by combining XYZ pointCloud with Texture image
@param		const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud     [IN] pointCloud xyz information in PCL format
@param      const cv::Mat& textureImage                             [IN] texture image
@param      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud     [OUT] pointCloud xyzRgb information in PCL format

@return     bool

@author		WD
@date		2020/06/17
**************************************************************************************************************************/
bool combineXyzWithTexture(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
    const cv::Mat& textureImage, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
)
{
    // textureImage 的数据类型必须是 CV_8UC3
    if(16 != textureImage.type())
    {
        std::cerr << "[Error] The type of textureImage(cv::Mat) must be CV_8UC3." << std::endl;
        return false;
    }

    xyzRgbCloud->width = xyzCloud->width;
    xyzRgbCloud->height = xyzCloud->height;
    xyzRgbCloud->is_dense = false;
    xyzRgbCloud->points.resize(xyzRgbCloud->height * xyzRgbCloud->width);

    pcl::PointXYZRGB* pt = &xyzRgbCloud->points[0];
    for (int i = 0; i < xyzCloud->width * xyzCloud->height; i++)
    {
        if (xyzCloud->points[i].z < 1e-5)// 无效点云用 0 (z=0) 表示的，新点云中改为 z=NaN 表示
        {
            pt->x = std::numeric_limits<float>::quiet_NaN();
            pt->y = std::numeric_limits<float>::quiet_NaN();
            pt->z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            pt->x = xyzCloud->points[i].x;
            pt->y = xyzCloud->points[i].y;
            pt->z = xyzCloud->points[i].z;
        }

        pt->b = textureImage.at<cv::Vec3b>(i)[0];// CV_8UC3, 三个通道依次表示 bgr
        pt->g = textureImage.at<cv::Vec3b>(i)[1];
        pt->r = textureImage.at<cv::Vec3b>(i)[2];

        pt++;
    }
    return true;
}