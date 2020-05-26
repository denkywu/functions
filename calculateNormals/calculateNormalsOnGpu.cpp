/**********************************************************************//**
		[GPU Version] 
			calculate Normals of PointCloud based on PCL

@file		calculateNormalsOnGpu.cpp
@author		WD
@date		2020/03/02
@brief		calculate Normals of PointCloud based on PCL using GPU.
**************************************************************************/
#include "calculateNormalsOnGpu.h"

// C++
#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>

// PCL
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/filters/filter.h>

// OpenCV
#include <opencv2/opencv.hpp>



/**********************************************************************************************************************//**
@brief		[GPU 版本] 法线估计
@param		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud		[IN]  输入点云xyz数据, pcl format
@param		const float radius										[IN]  搜索半径
@param		const unsigned int max_elements							[IN]  最大点数
@param		pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals	[OUT] 法线估计结果, pcl format

@return		bool

@author		WD
@date		2020/03/02
**************************************************************************************************************************/
bool calNormalsOnGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float radius, const unsigned int max_elements, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
#if WD_CALCULATE_NORMALS_ON_GPU_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest0 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedNaN(new pcl::PointCloud<pcl::PointXYZ>);
	//if cloud contained NaN points, GPU core will throw an error
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_removedNaN, index);
#if WD_CALCULATE_NORMALS_ON_GPU_DEBUG == 1
	std::cout << "Before remove NaN, point size: " << cloud->points.size() << std::endl;
	std::cout << "After remove NaN, point size: " << cloud_removedNaN->points.size() << std::endl;
#endif

    // normal estimation
	pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(cloud_removedNaN->points);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(radius, max_elements);
    ne_device.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    std::vector<pcl::PointXYZ> downloaded;
    normals_device.download(downloaded);// 至此，法线已计算完毕。接下来是将结果转换为需要的数据格式

    /*
        将法线结果存放为 pcl 格式
        注意: 这里没有恢复 NaN，是与点云 cloud_removedNaN 对应的。
    */
    pcl::PointCloud<pcl::Normal>::Ptr nc(new pcl::PointCloud<pcl::Normal>);
    warpVec2Cloud(downloaded, cloud_removedNaN->height, cloud_removedNaN->width, nc);
    // 合并点云和法线
    pcl::concatenateFields(*cloud_removedNaN, *nc, *cloud_normals);
    
#if WD_CALCULATE_NORMALS_ON_GPU_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
	std::cout << "[calNormalsOnGpu] Cost: " << tTest1.time_since_epoch().count() - tTest0.time_since_epoch().count() << " ms" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_ON_GPU_SAVE_FILES == 1
    pcl::io::savePCDFile("TestWD/pointNormals_gpu.pcd", *cloud_normals);
    std::cout << "Done: save pointNormals_gpu.pcd" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_ON_GPU_DEBUG == 1
    std::cout << "Done: calNormalsOnGpu()." << std::endl;
#endif

    return true;
}



/**********************************************************************************************************************//**
@brief		[GPU 版本] 法线估计 
@param		const cv::Mat& matCloud		            [IN]  输入点云xyz数据, cv::Mat format
@param		const float radius						[IN]  搜索半径
@param		const unsigned int max_elements			[IN]  最大点数
@param		cv::Mat& matNormals	                    [OUT] 法线估计结果, cv::Mat format

@return		bool

@author		WD
@date		2020/03/02
**************************************************************************************************************************/
bool calNormalsOnGpu(const cv::Mat& matCloud, const float radius, const unsigned int max_elements, cv::Mat& matNormals)
{
#if WD_CALCULATE_NORMALS_ON_GPU_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest0 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(false == convertMatToPcl(matCloud, cloud))
    {
        std::cerr << "Error in convertMatToPcl()." << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedNaN(new pcl::PointCloud<pcl::PointXYZ>);
	//if cloud contained NaN points, GPU core will throw an error
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_removedNaN, index);
#if WD_CALCULATE_NORMALS_ON_GPU_DEBUG == 1
	std::cout << "Before remove NaN, point size: " << cloud->points.size() << std::endl;
	std::cout << "After remove NaN, point size: " << cloud_removedNaN->points.size() << std::endl;
#endif

    // normal estimation
	pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(cloud_removedNaN->points);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(radius, max_elements);
    ne_device.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    std::vector<pcl::PointXYZ> downloaded;
    normals_device.download(downloaded);// 至此，法线已计算完毕。接下来是将结果转换为需要的数据格式

    /*
        将法线结果存放为 Mat 格式
        注意: 这里先恢复 NaN，是与点云 cloud 对应的。

        上面的法线数据是去除 NaN 后计算的，现在要利用 index 恢复索引，使其保持和输入 cloud 的size一致
        然后再将法线计算结果转换为 Mat 格式存储。
    */
    matNormals = cv::Mat(matCloud.rows, matCloud.cols, CV_32FC3, 
        cv::Scalar::all(std::numeric_limits<float>::quiet_NaN())).clone();// 初始值NaN
    warpVec2Mat(downloaded, index, matNormals);// 恢复NaN并将法线存放在Mat矩阵中

#if WD_CALCULATE_NORMALS_ON_GPU_CALC_TIME == 1
	std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> tTest1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
	std::cout << "[calNormalsOnGpu] Cost: " << tTest1.time_since_epoch().count() - tTest0.time_since_epoch().count() << " ms" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_ON_GPU_SAVE_FILES == 1   
    cv::String wd_Path_normal = "TestWD/normal_gpu.xml";
    cv::FileStorage fs_normal(wd_Path_normal, cv::FileStorage::WRITE);
    fs_normal << "normal" << matNormals;// 双引号内绝对不能有空格 !
    fs_normal.release();
    std::cout << "Done: save normal_gpu.xml" << std::endl;
    // 利用pcl保存为 pcd 文件
    pcl::PointCloud<pcl::Normal>::Ptr pclNormal(new pcl::PointCloud<pcl::Normal>);
    if(false == convertMatToPclNormal(matNormals, pclNormal))
    {
        std::cerr << "Error in convertMatToPclNormal()." << std::endl;
        return false;
    }
    std::cout << "Normal has: " << pclNormal->points.size() << " data points." << std::endl;
    pcl::io::savePCDFile("TestWD/normal_gpu.pcd", *pclNormal);
    std::cout << "Done: save normal_gpu.pcd" << std::endl; 
    // 合并点云和normal
    pcl::PointCloud<pcl::PointNormal>::Ptr pclCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *pclNormal, *pclCloudNormal);
    // std::cout << "cloudNormal has: " << pclCloudNormal->points.size() << " data points." << std::endl;
    pcl::io::savePCDFile("TestWD/cloudNormal_gpu.pcd", *pclCloudNormal);
    std::cout << "Done: save cloudNormal_gpu.pcd" << std::endl;
#endif
#if WD_CALCULATE_NORMALS_ON_GPU_DEBUG == 1
    std::cout << "Done: calNormalsOnGpu()." << std::endl;
#endif

    return true;
}



/**********************************************************************************************************************//**
@brief      将存放在 std::vector 中的法线数据转换为 pcl 格式 (没有恢复NaN)
@param		const std::vector<pcl::PointXYZ>& vecPoint		[IN] std::vector 格式的法线数据
@param		const uint32_t height							[IN] 高, 行数
@param		const uint32_t width							[IN] 宽, 列数	
@param		pcl::PointCloud<pcl::Normal>::Ptr& pclNormal    [INOUT] pcl 格式的法线数据	

@return		void

@author		WD
@date		2020/02/27
**************************************************************************************************************************/
void warpVec2Cloud(const std::vector<pcl::PointXYZ>& vecPoint, const uint32_t height, const uint32_t width, pcl::PointCloud<pcl::Normal>::Ptr& pclNormal)
{
    pclNormal->clear();
    for(std::size_t i = 0; i < vecPoint.size(); ++i)
    {
        pcl::Normal normal;
        pcl::PointXYZ xyz = vecPoint[i];

        normal.normal_x = xyz.x;
        normal.normal_y = xyz.y;
        normal.normal_z = xyz.z;
        normal.curvature = xyz.data[3];
        pclNormal->points.push_back(normal);
    }
    pclNormal->width = width;
    pclNormal->height = height;

    return;
}



/**********************************************************************************************************************//**
@brief		convert cv::Mat format to pcl::PointCloud<pcl::PointXYZ>::Ptr
			点云xyz数据, cv::Mat 转 pcl 格式
@param		const cv::Mat& xyzDepth					        [IN]    pointCloud xyz information in cv::Mat format
@param		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud	    [OUT]	pcl format

@return     bool

@author		WD
@date		2020/02/28
**************************************************************************************************************************/
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // cv::Mat矩阵的数据类型必须是 CV_32FC3
    if(21 != xyzDepth.type())
    {
        std::cerr << "Error: the type of cv::Mat data must be CV_32FC3." << std::endl;
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
@brief      将存放在 std::vector 中的法线数据转换为 Mat 格式 (先恢复NaN)
            另外，只保留法线信息，曲率就不保存了
@param		const std::vector<pcl::PointXYZ>& vecPoint  [IN]  std::vector 格式的法线数据
@param		const std::vector<int>& index			    [IN]  去除NaN时输出的索引关系	
@param		cv::Mat& matNormal                          [OUT] Mat 格式的法线数据	

@return		void

@author		WD
@date		2020/02/27
**************************************************************************************************************************/
void warpVec2Mat(
    const std::vector<pcl::PointXYZ>& vecPoint, // 这是pcl基于gpu估计的法线（去除了NaN后的计算结果）
    const std::vector<int>& index, // 这是和去除NaN之前的点云cloud的对应索引关系
    cv::Mat& matNormal // 恢复NaN，存放在Mat矩阵中的法线估计结果(利用index)
)
{
    for(std::size_t i = 0; i < vecPoint.size(); ++i)
    {
        pcl::PointXYZ xyz = vecPoint[i];

        matNormal.at<cv::Vec3f>(index[i])[0] = xyz.x;
        matNormal.at<cv::Vec3f>(index[i])[1] = xyz.y;
        matNormal.at<cv::Vec3f>(index[i])[2] = xyz.z;
        // float curvature = xyz.data[3];// 曲率信息在这里被丢弃了，没有往Mat矩阵中存放
    }
    return;
}



/**********************************************************************************************************************//**
@brief		convert cv::Mat format to pcl::PointCloud<pcl::Normal>::Ptr
			法线, cv::Mat 转 pcl 格式
@param		const cv::Mat& matNormal				        [IN]    normal of pointCloud in cv::Mat format
@param		pcl::PointCloud<pcl::Normal>::Ptr& pclNormal	[OUT]	pcl format

@return     bool

@author		WD
@date		2020/02/28
**************************************************************************************************************************/
bool convertMatToPclNormal(const cv::Mat& matNormal, pcl::PointCloud<pcl::Normal>::Ptr& pclNormal)
{
    // cv::Mat矩阵的数据类型必须是 CV_32FC3
    if(21 != matNormal.type())
    {
        std::cerr << "Error: the type of cv::Mat data must be CV_32FC3." << std::endl;
        return false;
    }
	// cv::Mat数据大小
	auto normalWidth = matNormal.cols;
	auto normalHeight = matNormal.rows;

	pclNormal->width = static_cast<uint32_t>(normalWidth);
	pclNormal->height = static_cast<uint32_t>(normalHeight);
	pclNormal->is_dense = false;
	pclNormal->points.resize(pclNormal->height * pclNormal->width);

	pcl::Normal* pt = &pclNormal->points[0];
	for (int iRows = 0; iRows < normalHeight; iRows++)
	{
		for (int iCols = 0; iCols < normalWidth; iCols++, pt++)
		{
			pt->normal_x = matNormal.at<cv::Vec3f>(iRows, iCols)[0];
			pt->normal_y = matNormal.at<cv::Vec3f>(iRows, iCols)[1];
			pt->normal_z = matNormal.at<cv::Vec3f>(iRows, iCols)[2];

            pt->curvature = std::numeric_limits<float>::quiet_NaN();// 赋值NaN
		}
	}
	return true;
}