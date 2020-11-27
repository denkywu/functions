/**********************************************************************//**
                extract targetUv based on sourceXyz

@file		extractTargetUvBasedOnSourceXyz.cpp
@author	    WD
@date		2020/06/19
@brief		extract uv points (target camera) based on xyz points (source camera)
**************************************************************************/
#include "extractTargetUvBasedOnSourceXyz.h"



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
@date		2020/06/19
**************************************************************************************************************************/
void extractTargetUvBasedOnSourceXyz(
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