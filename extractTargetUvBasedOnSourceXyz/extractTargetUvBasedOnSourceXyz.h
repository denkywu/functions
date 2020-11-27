/**********************************************************************//**
                extract targetUv based on sourceXyz

@file		extractTargetUvBasedOnSourceXyz.h
@author	    WD
@date		2020/06/19
@brief	    Statement for "extractTargetUvBasedOnSourceXyz.cpp"
**************************************************************************/
#ifndef _EXTRACT_TARGET_UV_BASED_ON_SOURCE_XYZ_H_
#define _EXTRACT_TARGET_UV_BASED_ON_SOURCE_XYZ_H_

// C++
#include <iostream>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>


void extract2dImage(
    const std::vector<cv::Point3f>& xyzPoints, 
    const cv::Mat& transMat, 
    const cv::Mat& intrMatTarget, 
    const cv::Mat& discoefTarget,
    cv::Size& sizeTarget, 
    std::vector<cv::Point2i>& uvPoints
);


#endif // _EXTRACT_TARGET_UV_BASED_ON_SOURCE_XYZ_H_