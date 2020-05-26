/**********************************************************************//**
		Demo about using calculateNormals.h(cpp)

@file		main_demo_normals_cpu.cpp
@author		WD
@date		2020/03/02
@brief		using function calNormals()
**************************************************************************/
// C++
#include <iostream>
#include <string>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../calculateNormals.h"



// main
int main(int argc, char* argv[])
{
	// ---------------------------------------------------------------------------------------------------------
	/*
			calNormals() 测试
	*/
	if (argc < 3)
	{
		std::cout << "usage: " << argv[0] << " inputFile scale(5.0)" << std::endl;
		return EXIT_FAILURE;
	}
	double scale;
    std::string infile = std::string(argv[1]);// the file to read from (need a .pcd file)
	std::istringstream (argv[2]) >> scale;

	// Load cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(infile.c_str(), *cloud);
	std::cout << "Done: Load pointCloud data of " << infile.c_str() << std::endl;

	// calculate normals
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	calNormals(cloud, scale, normals);
	std::cout << "Done: calNormals(). " << std::endl;



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