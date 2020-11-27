/**********************************************************************//**
                  generate texture image

@file		generateTexture.cpp
@author	    WD
@date		2020/06/19
@brief		generate texture image based on pointCloud, 2D color image, 
            cameras intrinsic and extrinsic parameters.
**************************************************************************/
#include "generateTexture.h"



/**********************************************************************************************************************//**
@brief		generate texture image
@param		const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud     [IN] pointCloud xyz information in PCL format
@param      const cv::Mat& colorImage               [IN] 2D color image
@param      const cv::Mat& transMatFrame2dTo3d      [IN] 外参(齐次坐标): frame 从 target->source, 则数据从 source 变换到 target.
                                                            此处 frame 从 2D->3D. double类型. 
                                                            旋转矩阵放在前3*3的位置, 平移向量放在第4列, 最后一行是 (0,0,0,1).
@param      const cv::Mat& intrMatColor             [IN] 2D相机的内参
@param      const cv::Mat& discoefColor             [IN] 2D相机的畸变系数
@param      cv::Mat& textureImage                   [OUT] texture image, 点云纹理彩色图像
@param      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud     [OUT] pointCloud xyzRgb information in PCL format

@return     bool

@author		WD
@date		2020/06/19
**************************************************************************************************************************/
bool generateTexture(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud,
    const cv::Mat& colorImage, 
    const cv::Mat& transMatFrame2dTo3d, 
    const cv::Mat& intrMatColor, 
    const cv::Mat& discoefColor, 
    cv::Mat& textureImage, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
)
{
    // colorImage 的数据类型必须是 CV_8UC3
    if(16 != colorImage.type())
    {
        std::cerr << "[Error] The type of colorImage(cv::Mat) must be CV_8UC3." << std::endl;
        return false;
    }
    cv::Size sizeColor(colorImage.cols, colorImage.rows);
#if 0
    cv::imshow("color image", colorImage);
    cv::waitKey(0);
#endif


    // ---------------------------------------------------------------------------------------------------------
    /*
                生成 RGB-D 数据
    */
    // 将 点云数据 放 vector 中 (去除无效数据)
    std::vector<cv::Point3f> xyzPointsNoInvalid;
    std::vector<int> indexInXyzCloud;// 去掉无效数据后的点云在原始 xyzCloud 中的索引
	for (int iNum = 0; iNum < xyzCloud->points.size(); iNum++)
	{
        // if(std::isnan(xyzCloud->points[iNum].x))// 去掉 NaN (当确定无效数据是用 NaN 表示时, 用这个即可)
        // {
        //     continue;
        // }
        if(
            std::isnan(xyzCloud->points[iNum].x) 
            || (xyzCloud->points[iNum].z < 1e-6)
        )// 去掉 无效数据 (不清楚其表示时, 可能是 NaN, 可能是 z=0, 与点云无效数据的默认值有关)
        {
            continue;
        }

        cv::Point3f xyz;
        xyz.x = xyzCloud->points[iNum].x;
        xyz.y = xyzCloud->points[iNum].y;
        xyz.z = xyzCloud->points[iNum].z;
        xyzPointsNoInvalid.push_back(xyz);
        indexInXyzCloud.push_back(iNum);
	}

    // 将点云投影到 2D 彩色图像中, 取出结果(即为点云纹理彩色图像), 并再和相应的 xyz 合成 xyzrgb 点云
    std::vector<cv::Point2i> uvColor;
    extract2dImage(xyzPointsNoInvalid, transMatFrame2dTo3d, intrMatColor, discoefColor, sizeColor, uvColor);

    textureImage = cv::Mat::zeros(xyzCloud->height, xyzCloud->width, CV_8UC3);// 初始化
    pcl::copyPointCloud(*xyzCloud, *xyzRgbCloud);// 初始化: 先拷贝得到 xyz, 同时 rgb 初始化为 0
    for (size_t i = 0; i < uvColor.size(); i++)
    {
        int col = uvColor[i].x;
        int row = uvColor[i].y;

        int index = indexInXyzCloud[i];// 在 xyzCloud 中的索引
        
        // 更新 xyzRgbCloud 中的 rgb
        xyzRgbCloud->points[index].b = colorImage.at<cv::Vec3b>(row, col)[0];
        xyzRgbCloud->points[index].g = colorImage.at<cv::Vec3b>(row, col)[1];
        xyzRgbCloud->points[index].r = colorImage.at<cv::Vec3b>(row, col)[2];

        // 纹理彩色图像
        textureImage.at<cv::Vec3b>(index)[0] = colorImage.at<cv::Vec3b>(row, col)[0];
        textureImage.at<cv::Vec3b>(index)[1] = colorImage.at<cv::Vec3b>(row, col)[1];
        textureImage.at<cv::Vec3b>(index)[2] = colorImage.at<cv::Vec3b>(row, col)[2];
    }
    return true;
}



/**********************************************************************************************************************//**
@brief		基于 source 相机中的点云 xyz 提取 target 相机中的 uv 像素坐标
            source 和 target 可以相同或不同
@param		const std::vector<cv::Point3f>& xyzPoints	[IN] source 相机中的点云 xyz 数据
@param		const cv::Mat& transMat						[IN] 坐标系变换矩阵(齐次坐标): frame 从 target->source, 则数据从 source 变换到 target
@param		const cv::Mat& intrMatTarget				[IN] target 相机内参
@param		const cv::Mat& discoefTarget				[IN] target 相机畸变系数
@param		cv::Size& sizeTarget					    [IN] target 相机的图像 size
@param		std::vector<cv::Point2i>& uvPoints	        [OUT] 与 source 中 xyzPoints 对应的 target 中的 uv 数值

@return     void

@author		WD
@date		2020/04/20
**************************************************************************************************************************/
void extract2dImage(
    const std::vector<cv::Point3f>& xyzPoints, 
    const cv::Mat& transMat, 
    const cv::Mat& intrMatTarget, 
    const cv::Mat& discoefTarget,
    cv::Size& sizeTarget, 
    std::vector<cv::Point2i>& uvPoints
)
{
	cv::Mat rotMat = (cv::Mat_<double>(3, 3) <<
		transMat.at<double>(0, 0), transMat.at<double>(0, 1), transMat.at<double>(0, 2),
		transMat.at<double>(1, 0), transMat.at<double>(1, 1), transMat.at<double>(1, 2),
		transMat.at<double>(2, 0), transMat.at<double>(2, 1), transMat.at<double>(2, 2)
	);
	cv::Mat rVec;
	cv::Rodrigues(rotMat, rVec);
	cv::Mat tVec = (cv::Mat_<double>(3, 1) <<
		transMat.at<double>(0, 3),
		transMat.at<double>(1, 3),
		transMat.at<double>(2, 3)
	);
	std::vector<cv::Point2f> uvPoints2f;
    // 从 source 投影到 target
	cv::projectPoints(xyzPoints, rVec, tVec, intrMatTarget, discoefTarget, uvPoints2f);

    // 将格式从 Point2f 转为 Point2i
	for (int i = 0; i < uvPoints2f.size(); ++i) 
    {
		cv::Point2i uv;
		uv.x = int(uvPoints2f[i].x + 0.5);
		uv.y = int(uvPoints2f[i].y + 0.5);
		uv.x = (uv.x < 0 ? 0 : uv.x);
		uv.x = (uv.x >= sizeTarget.width ? sizeTarget.width - 1 : uv.x);
		uv.y = (uv.y < 0 ? 0 : uv.y);
		uv.y = (uv.y >= sizeTarget.height ? sizeTarget.height - 1 : uv.y);
		uvPoints.push_back(uv);
	}

	return;
}