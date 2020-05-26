# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name
project(WD_test_normals)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")

# Find PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 包含头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# 可执行文件
add_executable(
	WD_test_normals 
	main_demo_normals_cpu.cpp
	../calculateNormals.cpp
)
# 链接
target_link_libraries (WD_test_normals ${PCL_LIBRARIES})



#　WD
# 2020.03.02
