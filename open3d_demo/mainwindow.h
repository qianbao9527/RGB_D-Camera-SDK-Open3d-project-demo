#pragma once

#include <QWidget>
#include "ui_MainWindow.h"
#include"OPEN3D_MVS.h"
#include"depth_to_points.h"
#include"my_semaphore.h"
#define	QT_ASSERT_OK(x)  (int(x)!=MV3D_RGBD_OK)

class  MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QMainWindow *parent = Q_NULLPTR);
	~MainWindow();
	Mv3dRgbdDeviceType device_type;//设备连接方式
	std::vector<MV3D_RGBD_DEVICE_INFO> devs;//设备列表
	//
	//void set_poins_st(depth_to_points* points_st);
	//void get_pra();//获取渲染参数
private:
	Ui::MainWindow ui;
	
	//depth_to_points* points_st;
	QValidator* Validator = new QRegExpValidator(QRegExp("[0-9\\.]+$"));

	//void refush_buffer();
	double isnumber(const string&& ss);


	void connection();
	

public slots:
	void find_device();
	void open_device();
	void close_device();
	void refush_viewoption();
	void point_process();
	void set_bouding_box();
	void set_ROI(); 
	void set_filter();
	//void refush_process();
};