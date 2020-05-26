/**********************************************************************//**
		(1) write cv::Mat To Binary File
		(2)	read Binary File To cv::Mat

@file		cvMatToBinaryFile.cpp
@author		WD
@date		2019/11/14
@brief		write/read cv::Mat format to/from Binary File
**************************************************************************/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>


/**********************************************************************************************************************//**
@brief		写文件：从 Mat 到二进制文件
@param		const cv::Mat&			[IN] matData，存放在cv::Mat中的数据，格式不限
@param		const std::string&		[INOUT] filename，要保存的文件名（不存在则创建，否则抹掉从头写）

@return		bool					[OUT]  1（true）成功，0（false）失败

@author		WD
@date		2019/11/14
**************************************************************************************************************************/
bool matToBinaryFile(const cv::Mat& matData, const std::string& filename)
{
	if (matData.empty())
	{
		std::cout << "Error: empty cv::Mat." << std::endl;
		return false;
	}
	const char *filenamechar = filename.c_str();
	FILE *fpw = fopen(filenamechar, "wb");// 如果没有则创建，如果存在则从头开始写
	if (fpw == NULL)
	{
		std::cout << "Error in opening file." << std::endl;
		fclose(fpw);
		return false;
	}

	int channl = matData.channels();// 通道
	int rows = matData.rows;// 行数
	int cols = matData.cols;// 列数
#if DEBUG_matToBinaryFile == 1
	std::cout << "channl:" << channl << ", rows:" << rows << ", cols:" << cols << std::endl;
	// 以 CV_32FC3 为例，则应该显示， channl:3, rows:424, cols:512
#endif// DEBUG

	int typeMat = matData.type();// Mat数据类型
	int elemSize_Mat = matData.elemSize();// Mat中单个元素所占的字节数（与通道数有关）
	int elemSize1_Mat = matData.elemSize1();// 所占的字节数（与通道数无关），我理解为单个元素单个通道上数据所占字节数。
#if DEBUG_matToBinaryFile == 1
	std::cout << "typeMat:" << typeMat << ", elemSize:" << elemSize_Mat << ", elemSize1:" << elemSize1_Mat << std::endl;
	// 以 CV_32FC3 为例，则应该显示， typeMat:21, elemSize:12, elemSize1:4
#endif// DEBUG

	fwrite(&channl, sizeof(char), 4, fpw);// 四个字节存
	fwrite(&rows, sizeof(char), 4, fpw);// 四个字节存
	fwrite(&cols, sizeof(char), 4, fpw);// 四个字节存
	fwrite(&typeMat, sizeof(char), 4, fpw);// 四个字节存
	fwrite(&elemSize_Mat, sizeof(char), 4, fpw);// 四个字节存
	fwrite(&elemSize1_Mat, sizeof(char), 4, fpw);// 四个字节存

	char* dp = (char*)matData.data;
	fwrite(dp, sizeof(char)*elemSize_Mat, cols*rows, fpw);// 每个元素所占字节为 elemSize_Mat，应该 = channl*elemSize1_Mat.

	fclose(fpw);
	return true;
}


/**********************************************************************************************************************//**
@brief		读文件：从二进制文件到 Mat
@param		const std::string&		[IN] filename，需要读入的文件名（要已存在）
@param		cv::Mat&				[INOUT] matData，将数据存放在cv::Mat中，格式不限

@return		bool					[OUT]  1（true）成功，0（false）失败

@author		WD
@date		2019/11/14
**************************************************************************************************************************/
bool binaryFileToMat(const std::string& filename, cv::Mat& matData)
{
	const char *filenamechar = filename.c_str();
	FILE *fpr = fopen(filenamechar, "rb");
	if (fpr == NULL)
	{
		std::cout << "Error in opening file." << std::endl;
		fclose(fpr);
		return false;
	}

	int channl = 0;// 通道
	int rows = 0;// 行数
	int cols = 0;// 列数
	int typeMat = 0;// Mat数据类型
	int elemSize_Mat = 0;// Mat中单个元素所占的字节数（与通道数有关）
	int elemSize1_Mat = 0;// 所占的字节数（与通道数无关），我理解为单个元素单个通道上数据所占字节数。
	int resultCount;// 存放fread()返回值
	resultCount = fread(&channl, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
	resultCount = fread(&rows, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
	resultCount = fread(&cols, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
	resultCount = fread(&typeMat, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
	resultCount = fread(&elemSize_Mat, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
	resultCount = fread(&elemSize1_Mat, sizeof(char), 4, fpr);
	if(4 != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
#if DEBUG_binaryFileToMat == 1
	std::cout << "channl:"<< channl << ", rows:" << rows << ", cols:" << cols << std::endl;
	std::cout << "typeMat:" << typeMat << ", elemSize:" << elemSize_Mat << ", elemSize1:" << elemSize1_Mat << std::endl;
#endif// DEBUG

	// 读入数据
	matData = cv::Mat::zeros(rows, cols, typeMat);
	char* pData = (char*)matData.data;
	resultCount = fread(pData, sizeof(char)*elemSize_Mat, cols*rows, fpr);
	if(cols*rows != resultCount)
	{
		std::cout << "Error on fread()!!!" << std::endl;
		fclose(fpr);
		return false;
	}
		
	fclose(fpr);
	return true;
}