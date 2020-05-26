/**********************************************************************//**
		(1) write cv::Mat To Binary File
		(2)	read Binary File To cv::Mat

@file		cvMatToBinaryFile.h
@author		WD
@date		2019/11/14
@brief		Statement for "cvMatToBinaryFile.cpp"
**************************************************************************/
#ifndef _CVMAT_TO_BINARY_FILE_H_
#define _CVMAT_TO_BINARY_FILE_H_

#include <opencv2/opencv.hpp>

// 函数 matToBinaryFile() Debug用
#define DEBUG_matToBinaryFile 0

// 函数 binaryFileToMat() Debug用
#define DEBUG_binaryFileToMat 0


// 将 Mat 写到二进制文件
bool matToBinaryFile(const cv::Mat& matData, const std::string& filename);

// 从二进制文件读入到 Mat
bool binaryFileToMat(const std::string& filename, cv::Mat& matData);


#endif// _CVMAT_TO_BINARY_FILE_H_