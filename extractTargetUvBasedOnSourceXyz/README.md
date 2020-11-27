功能:
基于 source 相机中的点云 xyz 提取 target 相机中的 uv 像素坐标.
source 和 target 可以相同或不同.

输入:
1. source 相机中的点云 xyz 数据;
2. 坐标系变换矩阵(齐次坐标): frame 从 target->source, 则数据从 source 变换到 target;
3. target 相机内参;
4. target 相机畸变系数;
5. target 相机的图像 size.

输出:
1. 与 source 中 xyzPoints 对应的 target 中的 uv 数值.


WD

