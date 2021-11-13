<h1 align = "center">LiDAR 点云配准</h1>





# 1. 配准目的



- 对点云进行校正



# 2. 配准分类



- LiDAR点云和LiDAR点云配准
- LiDAR点云和密集匹配点云配准
- LiDAR点云和二维影像配准
- LiDAR点云和三维街景匹配
- 其它



# 3. 配准方法



粗配准方案



- LORAX
- 4点法（4-Points Congruent Sets，4PCS）
- Super 4PCS(Super 4-Points Congruent Sets)
- SK-4PCS（Semantic Keypoint 4-Points Congruent Sets）
- G-4PCS（Generalized 4-points congruent sets）



精配准方案



- DO（Discriminative Optimization）
- 结合法
- **ICP（Iterative Closest Point）**

Standard ICP

KD-tree Approximation

Soft Outlier Rejection

Generalized-icp（GICP）

Normal ICP（NICP）

Go-ICP

…

- 模型对应法
- NDT