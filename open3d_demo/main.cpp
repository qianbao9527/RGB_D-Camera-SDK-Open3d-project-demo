#include "open3d_demo.h"
#include <QtWidgets/QApplication>

#include"depth_to_points.h"
#include"mainwindow.h"

#include"OPEN3D_MVS.h"
#include<thread>
void refush_buffer(depth_to_points * sol );
void visual_open3d(shared_ptr<geometry::PointCloud> pointcloud);
void point_process(depth_to_points * sol);
void p_process(int choose, vector<double>pra);
mutex mtx_vis, mtx_refush, mtx_pdata;
Semaphore my_sm, my_vs;
int main(int argc, char *argv[])
{


    QApplication a(argc, argv);
     MainWindow w;
	 depth_to_points* sol = new depth_to_points();
	
	thread my_refush(refush_buffer, sol);
	thread my_vis(visual_open3d, sol->get_pointcloud());
	thread my_process(point_process,sol);
	//thread my_vis_process(point_process_veiw,sol->get_output());
	 my_refush.detach();
	 my_vis.detach();
	 my_process.detach();
	 
	w.show();
	//delete sol;
	
    return a.exec();
}
void point_process(depth_to_points * sol)
{
	
	visualization::Visualizer visualizer2;
	auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame(50);//坐标系框架
	mtx_color_option.lock();
	visualizer2.CreateVisualizerWindow("point_process", 640, 480, 200, 200, true);//注意该函数应该调用了某个静态对象方法，需要线程同步
	visualizer2.GetRenderOption().point_size_ = color_set.point_size;//渲染设置通用
	visualizer2.GetRenderOption().background_color_ = color_set.background_color;
	visualizer2.GetRenderOption().point_color_option_ = color_set.color_option;
	mtx_color_option.unlock();
	visualizer2.GetRenderOption().show_coordinate_frame_ = true;
	visualizer2.AddGeometry(mesh);
	visualizer2.UpdateGeometry();
	visualizer2.PollEvents();
	visualizer2.UpdateRender();
	//shared_ptr<open3d::geometry::PointCloud>points=make_shared<open3d::geometry::PointCloud>();
	//加一个条件变量来提示更新窗口
	bool flag = true;
	//限制窗口数
	unordered_set<int>choose_single;
	choose_single.insert(0);
	while (true)//程序结束后自动回收
	{
		mtx_points_process.lock();
		if (points_pro.can_process)
		{
		
			points_pro.can_process = false;
			mtx_pdata.lock();//锁住所有点云处理需要的接口、数据
			refush_process_points(points_pro.points, sol->get_pointcloud());//更新静态点云数据
			mtx_pdata.unlock();
			if (flag&&points_pro.points->points_.size()>0)
			{
				visualizer2.AddGeometry(points_pro.points); flag = false;
			}

	
			
		}	
		if (points_process_func.find(points_pro.choose) != points_process_func.end())
		{
			if (choose_single.count(points_pro.choose) < 1 && points_pro.points&&points_pro.points->points_.size())
			{
				if(points_pro.choose!=2)//Open3d提供的裁剪窗口仅适合静态点云，且好像不能更新
				choose_single.insert(points_pro.choose);
				points_pro.refush_p[points_pro.choose] = true;
				mtx_points_process.unlock();
				thread px(p_process,points_pro.choose, points_pro.pra);
				px.detach();
				mtx_points_process.lock();
			}
			else
			{
				points_pro.refush_p[points_pro.choose] = true;
			}
			points_pro.choose = 0;//设置为不显示
		}
		visualizer2.UpdateGeometry();
		mtx_points_process.unlock();
		visualizer2.PollEvents();
		visualizer2.UpdateRender();

		
	}
}
void p_process(int choose,vector<double>pra)
{
	//mtx_points_process.lock();
	shared_ptr<open3d::geometry::PointCloud>pointcloud = make_shared<open3d::geometry::PointCloud>();
	points_process_func[choose](pra,pointcloud);
}


//void point_process_veiw(shared_ptr<geometry::PointCloud> pointcloud);

void visual_open3d(shared_ptr<geometry::PointCloud> pointcloud)
{
	visualization::Visualizer visualizer;
	auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame(50);//坐标系框架
	mtx_color_option.lock();
	visualizer.CreateVisualizerWindow("open3d_demo", 640, 480, 50, 50, true);
	
	visualizer.GetRenderOption().point_size_ =color_set.point_size;
	visualizer.GetRenderOption().background_color_ = color_set.background_color;
	//open3d::visualization::RenderOption::point_color_option_ pp;
	visualizer.GetRenderOption().point_color_option_ = color_set.color_option;
	mtx_color_option.unlock();
	visualizer.GetRenderOption().show_coordinate_frame_ = true;
	visualizer.AddGeometry(mesh);
	visualizer.UpdateGeometry();
	visualizer.PollEvents();
	visualizer.UpdateRender();
	//加一个条件变量来提示更新窗口
	bool flag = true;
	//渲染这方准备就绪通知读数据
	my_sm.signal();
	while (true)
	{
		mtx_color_option.lock();
		if (color_set.refush_veiw)
		{
			color_set.refush_veiw = false;
			visualizer.GetRenderOption().point_size_ = color_set.point_size;
			visualizer.GetRenderOption().background_color_ = color_set.background_color;
			visualizer.GetRenderOption().point_color_option_ = color_set.color_option;
		}
		mtx_updata.lock();
		if (updata==false)//是否更新点云显示信号
		{
			mtx_color_option.unlock();
			mtx_updata.unlock();
			visualizer.PollEvents();
			visualizer.UpdateRender();
			continue;
		}
		else
		{
			updata = false;
		}
		mtx_updata.unlock();
		mtx_color_option.unlock();

		//my_vs.wait();//没有点时，线程睡眠；存在情况，未添加点云时，窗口卡死
		mtx_pdata.lock();//锁住数据
		if (flag)
		{
			visualizer.AddGeometry(pointcloud);
			int a = 0;
			flag = false;
		}

		visualizer.UpdateGeometry();
		mtx_pdata.unlock();
		my_sm.signal();//信号量+1
		visualizer.PollEvents();
		visualizer.UpdateRender();
		
		
	}
	visualizer.DestroyVisualizerWindow();
}

void refush_buffer(depth_to_points * sol)
{
	bool bExit_Main = false;
	
	MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
	//my_cond.notify_one();
	while (!bExit_Main)
	{
		mtx_read_points.lock(); 
		if (!points_get.canread||points_get.handle==nullptr)
		{
			mtx_read_points.unlock();
			int i = 100;//等一会
			while (i--) {};
			continue;
		}
		my_sm.wait();//等待渲染线程完成初始化或几何数据更新
		//w;//获取窗口的渲染参数设置
		// 获取图像数据
		int nRet = MV3D_RGBD_FetchFrame(points_get.handle, &stFrameData, 5000);
		if (MV3D_RGBD_OK == nRet)
		{
			MV3D_RGBD_IMAGE_DATA stPointCloudImage;
			int yuv_image = 0;
			for (int i = 0; i < stFrameData.nImageCount; i++)
			{
				if (ImageType_YUV422 == stFrameData.stImageData[i].enImageType|| ImageType_RGB8_Planar== stFrameData.stImageData[i].enImageType)//修改读取方式，ImageType_YUV422/ImageType_RGB8_Planar
				{
					//rgb图像处理
					yuv_image = i;
				}
				if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
				{

					nRet = MV3D_RGBD_MapDepthToPointCloud(points_get.handle, &stFrameData.stImageData[i], &stPointCloudImage);
					if (MV3D_RGBD_OK != nRet)
					{
					//	cout << "MV3D_RGBD_OK != nRet" << endl;
						break;
					}
				}
			}
			mtx_pdata.lock();
			if (!sol->refush_point(&stPointCloudImage, points_get.refush_color, &stFrameData.stImageData[yuv_image]))
			{
			////	cout << "read point error\n";
			mtx_pdata.unlock(); 
			mtx_read_points.unlock();
			my_sm.signal();// 跳过渲染更新点云，再去读
			continue;
			};
			mtx_pdata.unlock();
		}
		mtx_updata.lock();
		updata = true;
		mtx_updata.unlock();
		mtx_read_points.unlock();
	}
}
