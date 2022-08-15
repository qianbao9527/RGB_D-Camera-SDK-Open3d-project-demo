#pragma once
#include"depth_to_points.h"
#include<mutex>
#include "my_semaphore.h"
#include"OPEN3D_MVS.h"
extern  color_opt color_set;//可视化设置
extern  points_option  points_get;//点云数据读取设置
extern  depth_to_points* sol;//点云数据buffer
extern	point_pro points_pro;//静态点云处理
extern bool updata;
static mutex mtx_color_option;
static mutex mtx_read_points;
static mutex mtx_points_process;
static Semaphore sem_points_pro;
static mutex mtx_updata;
bool cmp_BoundingBox(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud = make_shared<open3d::geometry::PointCloud>());
bool ROI_points(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud = make_shared<open3d::geometry::PointCloud>());
bool normal_es(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud = make_shared<open3d::geometry::PointCloud>());
bool voxel_filter(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud = make_shared<open3d::geometry::PointCloud>());
static std::unordered_map<int, std::function<bool(const vector<double>&pra,shared_ptr<open3d::geometry::PointCloud>)>> points_process_func
{
	{1,cmp_BoundingBox},
	{2,ROI_points},
	{3,normal_es},
	{4,voxel_filter},
};
//class option
//{
//public:
//	static color_opt color_set;
//	static points_option points_get;
//};