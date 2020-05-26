/**********************************************************************//**
		[GPU Version] 
		    Demo about using calculateNormalsOnGpu.h(cpp)

@file		main_demo_normals_gpu.cpp
@author		WD
@date		2020/03/02
@brief		using function calNormalsOnGpu()
**************************************************************************/
// C++
#include <iostream>
#include <string>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// OpenCV
#include <opencv2/opencv.hpp>

#include "../calculateNormalsOnGpu.h"



// main
int main(int argc, char* argv[])
{
	// ---------------------------------------------------------------------------------------------------------
	/*
			calNormalsOnGpu() 测试
	*/
	if (argc < 4)
	{
		std::cout << "usage: " << argv[0] << " inputFile scale(5.0) max_elements(256)" << std::endl;
		return EXIT_FAILURE;
	}
	float scale;
    unsigned int max_elements;
    std::string infile = std::string(argv[1]);// the file to read from
	std::istringstream (argv[2]) >> scale;
    std::istringstream (argv[3]) >> max_elements;


    // test-1: 输入是pcl格式点云，输出是pcl格式法线的 法线估计函数
#if 0
	// Load cloud (.pcd file)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(infile.c_str(), *cloud);
	std::cout << "Done: Load pointCloud data of " << infile.c_str() << std::endl;
	// calculate normals on gpu
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    calNormalsOnGpu(cloud, scale, max_elements, normals);
	std::cout << "Done: calNormalsOnGpu(). " << std::endl;
#endif

    // test-2: 输入是Mat矩阵点云，输出是Mat矩阵法线的 法线估计函数
#if 1
    // Load cloud (.xml file)
	cv::Mat matCloud = {};
    cv::FileStorage fsRead(infile.c_str(), cv::FileStorage::READ);
    fsRead["imageDepth"] >> matCloud;// 这和我写出xml文件时使用的名字有关
    // fsRead["pointCloud"] >> matCloud;
    fsRead.release();
    std::cout << "Done: Load pointCloud data of " << infile.c_str() << std::endl;
    // calculate normals on gpu
    cv::Mat matNormals = {};
    calNormalsOnGpu(matCloud, scale, max_elements, matNormals);
    std::cout << "matNormals: \n"
        << " - type: " << matNormals.type() << "\n"
        << " - channel: " << matNormals.channels() << "\n"
        << " - size: " << matNormals.size
        << std::endl;
    std::cout << "Done: calNormalsOnGpu(). " << std::endl;
#endif



	// ---------------------------------------------------------------------------------------------------------
	/*
			End - 等待输入，方便显示上述运行结果
	*/
	std::cout << "--------------------------------------------" << std::endl;
	std::cout << "Waiting for inputting an integer: ";
	int wd_wait;
	std::cin >> wd_wait;

	std::cout << "----------------------------------" << std::endl;
	std::cout << "------------- closed -------------" << std::endl;
	std::cout << "----------------------------------" << std::endl;

	return EXIT_SUCCESS;
}