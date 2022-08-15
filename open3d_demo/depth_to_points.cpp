#pragma once
#include "depth_to_points.h"

bool depth_to_points::refush_point(MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush, MV3D_RGBD_IMAGE_DATA* image_color)
{
	this->image_data = image_data;
	return MV3D_TO_OPEN3D(points_open3d, image_data, color_refush, image_color);

}
bool refush_process_points(shared_ptr<open3d::geometry::PointCloud>process_points, shared_ptr<open3d::geometry::PointCloud>points_clould)//深拷贝
{
	size_t psize = points_clould->points_.size();
	if (psize == 0)
		return false;
	process_points->points_.resize(psize);
	process_points->colors_.resize(points_clould->colors_.size());
	process_points->points_ = points_clould->points_;
	process_points->colors_ = points_clould->colors_;
	return true;
}


bool depth_to_points::MV3D_TO_OPEN3D(MV3D_OPEN3D_DATA* points_open3d, MV3D_RGBD_IMAGE_DATA* image_data, bool color_refush, MV3D_RGBD_IMAGE_DATA* image_color)
{
	if (image_data->enImageType != ImageType_PointCloud)
		return false;
	if (color_refush)//需要颜色更新
	{
		if (image_color == nullptr)
			return false;
		//读取颜色；
		if (!YUV_to_RGB_colorpoints(image_color, points_open3d->pointcloud))//已经获取color
			return false;
		//动态内存读取点云
		int step = 3 * sizeof(float);
		int sum = image_data->nDataLen / step;
		float* p_data = (float*)image_data->pData;
		int j = 0;
		for (int i = 0; i < sum; ++i)
		{
			float* add = p_data + i * 3;
			if (*(add + 2) > 6000 || *(add + 2) < 20)
				continue;
			Eigen::Vector3d pp(*(add)*-1, *(add + 1)*-1, *(add + 2));
			points_open3d->pointcloud->points_[j] = pp;
			points_open3d->pointcloud->colors_[j] = points_open3d->pointcloud->colors_[i];
			++j;
		}
		points_open3d->pointcloud->points_.resize(j);
		points_open3d->pointcloud->colors_.resize(j);
		return true;
	}
	//动态内存
	int step = 3 * sizeof(float);
	int sum = image_data->nDataLen / step;
	if (sum == 0)
		return false;
	float* p_data = (float*)image_data->pData;	
	
	if (sum != points_open3d->pointcloud->points_.size())
		points_open3d->pointcloud->points_.resize(sum);
	int j = 0;
	for (int i = 0; i < sum; ++i)
	{
		float* add = p_data + i * 3;
		if (*(add + 2) > 6000 || *(add + 2) < 20)
			continue;
		Eigen::Vector3d pp(*(add)*-1, *(add + 1)*-1, *(add + 2));
		points_open3d->pointcloud->points_[j] = pp;
		++j;
	}
	points_open3d->pointcloud->points_.resize(j);
	return true;
}


bool depth_to_points::voxel_filter(double voxel_size)
{
	output->pointcloud = points_open3d->pointcloud->VoxelDownSample(voxel_size);
	//printf("size:%d \n", output->pointcloud->points_.size());
	return true;
}

Eigen::Vector3d depth_to_points::yuv422_to_rgb(int y, int u, int v)
{
	double r, g, b;
	r =y+ ((360 * (v - 128)) >> 8);
	g = y- (((88 * (u - 128) + 184 * (v - 128))) >> 8);
	b = y + ((455 * (u - 128)) >> 8);
	/*r = (y + (u - 128)*1.375);
	g = y - (u - 128)*0.34375 - (v - 128)*0.703125;
	b = y + (v - 128)*1.734375;*/
	if (r > 255)
		r = 255;
	else if (r < 0)
		r = 0;
	if (g > 255)
		g = 255;
	else if (g < 0)
		g = 0;
	if (b > 255)
		b = 255;
	else if (b < 0)
		b = 0;
	return { r,g,b };
}

bool depth_to_points::YUV_to_RGB_colorpoints(MV3D_RGBD_IMAGE_DATA * image_data, shared_ptr<geometry::PointCloud> pointcloud)//未剔除杂点
{
	if (image_data->enImageType == ImageType_YUV422)
	{
		//YUV转RGB
		unsigned char*pdata = (unsigned char*)image_data->pData;
		int step = 4 * sizeof(unsigned char);
		int sum = image_data->nDataLen / step;
		pointcloud->colors_.resize(sum * 2);
		int y1, y2, u, v;
		for (int i = 0; i < sum; ++i)
		{
			y1 = ((unsigned int)*(pdata + 4 * i));
			u = ((unsigned int)*(pdata + 4 * i + 1));
			y2 = ((unsigned int)*(pdata + 4 * i + 2));
			v = ((unsigned int)*(pdata + 4 * i + 3));
			pointcloud->colors_[2 * i] = yuv422_to_rgb(y1, u, v);
			pointcloud->colors_[2 * i + 1] = yuv422_to_rgb(y2, u, v);
		}
	}
	else if (image_data->enImageType == ImageType_RGB8_Planar)
	{
		unsigned char*pdata = (unsigned char*)image_data->pData;;
		int step = 3 * sizeof(unsigned char);
		int sum = image_data->nDataLen / step;
		pointcloud->colors_.resize(sum);
		for (int i = 0; i < sum; ++i)
		{
			//pointcloud->colors_[i] = { 1.f,0.f,0.f };
			pointcloud->colors_[i] = { double((unsigned int)*(pdata + i)) / 255.f,double((unsigned int)*(pdata + i + sum)) / 255.f,double((unsigned int)*(pdata + i + 2 * sum)) / 255.f };
		}
	}
	else
		return false;

	return true;
}

shared_ptr<geometry::PointCloud> depth_to_points::get_pointcloud()
{
	return points_open3d->pointcloud;
}

shared_ptr<geometry::PointCloud> depth_to_points::get_output()
{
	//体素滤波输出
	//体素下采样
	return output->pointcloud;
}
