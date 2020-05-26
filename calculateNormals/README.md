基于 PCL 计算点云的法线
1. 版本1：基于 cpu 和 OpenMP 加速，无需 gpu。文件“calculateNormals.h(cpp)”；
2. 版本2：基于 gpu 加速的法线计算。需要配置 GPU+CUDA+PCL(需安装gpu模块)。文件“calculateNormalsOnGpu.h(cpp)”。

WD
