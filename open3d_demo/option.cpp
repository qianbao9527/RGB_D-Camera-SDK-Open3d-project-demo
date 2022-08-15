#pragma once
#include "option.h"
color_opt	color_set;
points_option points_get;
point_pro points_pro;
bool cmp_BoundingBox( const vector<double>&pra,shared_ptr<open3d::geometry::PointCloud>pointcloud )
{
	mtx_points_process.lock();
	pointcloud = points_pro.points;
	open3d::geometry::AxisAlignedBoundingBox box1 = pointcloud->GetAxisAlignedBoundingBox();
	shared_ptr<open3d::geometry::AxisAlignedBoundingBox> box = make_shared<open3d::geometry::AxisAlignedBoundingBox>(pointcloud->GetAxisAlignedBoundingBox());
	box->color_ = { pra[0],pra[1],pra[2] };
	
	open3d::visualization::Visualizer visualizer;
	mtx_color_option.lock();//该函数调用静态对象，线程不安全
	visualizer.CreateVisualizerWindow("BoundingBox", 640, 480, 50, 50, true);
	mtx_color_option.unlock();//该函数调用静态对象，线程不安全
	visualizer.GetRenderOption().point_size_ = 1;
	visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(1, 1, 1);
	visualizer.GetRenderOption().show_coordinate_frame_ = true;
	visualizer.AddGeometry(box);
	visualizer.AddGeometry({ pointcloud });
	visualizer.UpdateGeometry();
	mtx_points_process.unlock();
	visualizer.PollEvents();
	visualizer.UpdateRender();
	while (true)
	{
		mtx_points_process.lock();
		if (points_pro.refush_p[1])//等待更新
		{
			points_pro.refush_p[1] = false;
			mtx_points_process.unlock();
			*box = pointcloud->GetAxisAlignedBoundingBox();
			box->color_ = { pra[0],pra[1],pra[2] };
			visualizer.UpdateGeometry();//更新点云\包围盒
			
		}
		mtx_points_process.unlock();
		visualizer.PollEvents();
		visualizer.UpdateRender();
	}
	visualizer.DestroyVisualizerWindow();
	return true;
}
bool ROI_points(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud )
{
	open3d::visualization::VisualizerWithEditing visualizer;//VisualizerWithEditing禁止添加除点云、个别线以外的geometry；
	mtx_color_option.lock();//该函数调用静态对象，线程不安全
	visualizer.CreateVisualizerWindow("open3d_ROI_points", 640, 480, 50, 50, true);
	mtx_color_option.unlock();
	visualizer.GetRenderOption().point_size_ = 1;
	visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(1, 1, 1);
	visualizer.GetRenderOption().show_coordinate_frame_ = true;
	mtx_points_process.lock();
	refush_process_points(pointcloud, points_pro.points);
	visualizer.AddGeometry({ pointcloud });
	visualizer.UpdateGeometry();
	mtx_points_process.unlock();
	visualizer.PollEvents();
	visualizer.UpdateRender();

	while (true)
	{	
		visualizer.Run();
	}
	visualizer.DestroyVisualizerWindow();
	return true;
}
bool normal_es(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud )
{

	pointcloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01, 30));
	//open3d::visualization::DrawGeometries(pointcloud, true);
	return true;
};
bool voxel_filter(const vector<double>&pra, shared_ptr<open3d::geometry::PointCloud>pointcloud)
{
	mtx_points_process.lock();
	pointcloud = points_pro.points->VoxelDownSample(pra[0]);
	//refush_process_points(pointcloud, points_pro.points);
	mtx_points_process.unlock();
	open3d::visualization::Visualizer visualizer;
	mtx_color_option.lock();//该函数调用静态对象，线程不安全
	visualizer.CreateVisualizerWindow("voxel_filter", 640, 480, 50, 50, true);
	mtx_color_option.unlock();//该函数调用静态对象，线程不安全
	visualizer.GetRenderOption().point_size_ = 1;
	visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(1, 1, 1);
	visualizer.GetRenderOption().show_coordinate_frame_ = true;
	visualizer.AddGeometry({ pointcloud });
	visualizer.UpdateGeometry();
	while (true)
	{
		mtx_points_process.lock();
		if (points_pro.refush_p[4])//等待更新
		{
			points_pro.refush_p[4] = false;
			*pointcloud=*points_pro.points->VoxelDownSample(points_pro.pra[0]);
			visualizer.UpdateGeometry();//更新点云(滤波)
			mtx_points_process.unlock();
		}
		mtx_points_process.unlock();
		visualizer.UpdateGeometry();
		visualizer.PollEvents();
		visualizer.UpdateRender();
	}
	visualizer.DestroyVisualizerWindow();
	return true;
}
bool updata = false;