#pragma once
#include<iostream>
#include<memory>
#include<Eigen/Core>
#include<Open3D/Open3D.h>//注意重定义，open3d放在sdk前面
#include"../common/common.hpp"
#include<vector>
#include<unordered_map>
using namespace std;
using namespace open3d;
struct point_pro {
	bool can_process=false;
	vector<bool>refush_p = vector<bool>(11,false);
	//操作参数保存
	int choose = 0; //点云处理选择
	vector<double>pra=vector<double>(4,1.f);//静态点云点云参数
	shared_ptr<open3d::geometry::PointCloud>points = make_shared<open3d::geometry::PointCloud>();//静态点云存储
};
struct color_opt {

	bool refush_veiw = false;//渲染设置更新信号
	open3d::visualization::PointColorOption color_option = open3d::visualization::PointColorOption(1);//设置渲染方式
	double point_size = 1;//渲染点大小
	Eigen::Vector3d background_color = { 1,1,1 };
	bool updata=false;//设置更新信号；

};

struct points_option
{
	void* handle = nullptr;//设备句柄
	bool canread = false;//设备读取完毕信号
	bool refush_color = true;//是否设置渲染颜色
};
class MV3D_OPEN3D_DATA
{
public:
	MV3D_OPEN3D_DATA() 
	{
		pointcloud->points_.resize(1000000);
	};
	MV3D_OPEN3D_DATA(float* p_data, int p_len)
	{
		int step = 3 * sizeof(float);
		int sum = p_len / step;
		pointcloud->points_.clear();
		for (int i = 0; i < sum; ++i)
		{
			float* add = p_data + i * 3;
			Eigen::Vector3d pp(*(add), *(add + 1), *(add + 2));
			pointcloud->points_.push_back(pp);
		}
	}
	shared_ptr<geometry::PointCloud> pointcloud = std::make_shared<geometry::PointCloud>();
};

class depth_to_points
{
public:
	void init(int m_len)
	{
		this->m_len = m_len;
		pSD_st = (float*)malloc(m_len * sizeof(float));
	}
	depth_to_points() :add_point_pos(0)
	{
		this->m_len = 680;//初始内存大小
		points_open3d = new MV3D_OPEN3D_DATA();
		output = new MV3D_OPEN3D_DATA();
		init(m_len);
	}
	depth_to_points(float* p_st, int m_len) :add_point_pos(0)
	{
		this->m_len = m_len; points_open3d = nullptr;
		points_open3d = new MV3D_OPEN3D_DATA(p_st, m_len);
		output = new MV3D_OPEN3D_DATA();
		init(m_len);
	}
	~depth_to_points()
	{
		free(pSD_st);
		delete points_open3d;
		delete output;
	}
	//重置点云
	bool refush_point(MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush = false, MV3D_RGBD_IMAGE_DATA* image_color = nullptr);//直接使用点云数据
	//bool refush_point(MV3D_RGBD_IMAGE_DATA* image_data);//自定义生成点云，状态码问题，后续补充
	//点云转open3d格式，并进行渲染。
	bool MV3D_TO_OPEN3D(MV3D_OPEN3D_DATA* points_open3d, MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush = false, MV3D_RGBD_IMAGE_DATA* image_color = nullptr);
	//体素滤波
	bool voxel_filter(double voxel_size = 5);
	//YUV422转RGB
	Eigen::Vector3d yuv422_to_rgb(int y1, int u, int v);
	//YUV转RGB格式，来渲染点云颜色
	bool YUV_to_RGB_colorpoints(MV3D_RGBD_IMAGE_DATA* image_data, shared_ptr<geometry::PointCloud> pointcloud);
	shared_ptr<geometry::PointCloud> get_pointcloud();
	shared_ptr<geometry::PointCloud> get_output();
private:
	//动态内存储存点云数据
	float* pSD_st;
	//获取的图像数据
	MV3D_RGBD_FRAME_DATA a;
	//数据总长度
	int m_len;
	//拍摄的图像数据,待后续
	MV3D_RGBD_IMAGE_DATA* image_data;
	//目前数据所在位置
	int add_point_pos;
	//动态内存储存opend3d数据类型
	struct MV3D_OPEN3D_DATA* points_open3d;
	//open3d预处理后的数据类型
	MV3D_OPEN3D_DATA* output;
	
};

bool refush_process_points(shared_ptr<open3d::geometry::PointCloud>process_points, shared_ptr<open3d::geometry::PointCloud>points_clould);//深拷贝
