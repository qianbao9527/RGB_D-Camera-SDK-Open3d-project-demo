#pragma once
#include<iostream>
#include<memory>
#include<Eigen/Core>
#include<Open3D/Open3D.h>//ע���ض��壬open3d����sdkǰ��
#include"../common/common.hpp"
#include<vector>
#include<unordered_map>
using namespace std;
using namespace open3d;
struct point_pro {
	bool can_process=false;
	vector<bool>refush_p = vector<bool>(11,false);
	//������������
	int choose = 0; //���ƴ���ѡ��
	vector<double>pra=vector<double>(4,1.f);//��̬���Ƶ��Ʋ���
	shared_ptr<open3d::geometry::PointCloud>points = make_shared<open3d::geometry::PointCloud>();//��̬���ƴ洢
};
struct color_opt {

	bool refush_veiw = false;//��Ⱦ���ø����ź�
	open3d::visualization::PointColorOption color_option = open3d::visualization::PointColorOption(1);//������Ⱦ��ʽ
	double point_size = 1;//��Ⱦ���С
	Eigen::Vector3d background_color = { 1,1,1 };
	bool updata=false;//���ø����źţ�

};

struct points_option
{
	void* handle = nullptr;//�豸���
	bool canread = false;//�豸��ȡ����ź�
	bool refush_color = true;//�Ƿ�������Ⱦ��ɫ
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
		this->m_len = 680;//��ʼ�ڴ��С
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
	//���õ���
	bool refush_point(MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush = false, MV3D_RGBD_IMAGE_DATA* image_color = nullptr);//ֱ��ʹ�õ�������
	//bool refush_point(MV3D_RGBD_IMAGE_DATA* image_data);//�Զ������ɵ��ƣ�״̬�����⣬��������
	//����תopen3d��ʽ����������Ⱦ��
	bool MV3D_TO_OPEN3D(MV3D_OPEN3D_DATA* points_open3d, MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush = false, MV3D_RGBD_IMAGE_DATA* image_color = nullptr);
	//�����˲�
	bool voxel_filter(double voxel_size = 5);
	//YUV422תRGB
	Eigen::Vector3d yuv422_to_rgb(int y1, int u, int v);
	//YUVתRGB��ʽ������Ⱦ������ɫ
	bool YUV_to_RGB_colorpoints(MV3D_RGBD_IMAGE_DATA* image_data, shared_ptr<geometry::PointCloud> pointcloud);
	shared_ptr<geometry::PointCloud> get_pointcloud();
	shared_ptr<geometry::PointCloud> get_output();
private:
	//��̬�ڴ洢���������
	float* pSD_st;
	//��ȡ��ͼ������
	MV3D_RGBD_FRAME_DATA a;
	//�����ܳ���
	int m_len;
	//�����ͼ������,������
	MV3D_RGBD_IMAGE_DATA* image_data;
	//Ŀǰ��������λ��
	int add_point_pos;
	//��̬�ڴ洢��opend3d��������
	struct MV3D_OPEN3D_DATA* points_open3d;
	//open3dԤ��������������
	MV3D_OPEN3D_DATA* output;
	
};

bool refush_process_points(shared_ptr<open3d::geometry::PointCloud>process_points, shared_ptr<open3d::geometry::PointCloud>points_clould);//���
