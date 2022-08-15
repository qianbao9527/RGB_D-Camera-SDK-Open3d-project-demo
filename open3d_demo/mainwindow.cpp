#include "mainwindow.h"


MainWindow::MainWindow(QMainWindow *parent)
	: QMainWindow(parent)
{
	
	ui.setupUi(this);
	//初始化
	ui.textEdit->insertPlainText(QString::fromStdString("welcome !") + QString(getLocalTime())+QString("\n"));

	ui.lineEdit->setPlaceholderText("0-255"); ui.lineEdit->setValidator(Validator);
	ui.lineEdit_4->setPlaceholderText("0-255"); ui.lineEdit_4->setValidator(Validator);
	ui.lineEdit_5->setPlaceholderText("0-255"); ui.lineEdit_5->setValidator(Validator);
	ui.lineEdit_6->setPlaceholderText("填数字"); ui.lineEdit_6->setValidator(Validator);
	ui.pushButton_2->setEnabled(false);
	ui.pushButton_3->setEnabled(false);
	ui.pushButton_4->setEnabled(false);
	ui.radioButton->setChecked(1);
	connection();//连接信号
}

MainWindow::~MainWindow()
{

}
double MainWindow::isnumber(const string && ss)
{
	double cc = atof(ss.c_str());
	return cc;
}
void MainWindow::connection()
{
	connect(ui.pushButton,&QPushButton::clicked,this, &MainWindow::find_device);//查找设备
	connect(ui.pushButton_2, &QPushButton::clicked, this, &MainWindow::open_device);//开始采集数据
	connect(ui.pushButton_3,&QPushButton::clicked,this,&MainWindow::close_device);//关闭设备
	connect(ui.pushButton_4, &QPushButton::clicked, this, &MainWindow::refush_viewoption);//更新显示设置
	connect(ui.pushButton_7, &QPushButton::clicked, this, &MainWindow::point_process);//读取静态点云
	connect(ui.pushButton_6, &QPushButton::clicked, this, &MainWindow::set_bouding_box);//包围盒计算
	connect(ui.pushButton_9, &QPushButton::clicked, this, &MainWindow::set_ROI);//ROI裁剪
	connect(ui.pushButton_11, &QPushButton::clicked, this, &MainWindow::set_filter);//滤波
}
void MainWindow::find_device()
{
	ui.pushButton->setEnabled(false);
	ui.pushButton_2->setEnabled(false);
	ui.comboBox->clear();//清除所选
	ui.textEdit->insertPlainText(QString("finding device.....") + QString("\n"));

	MV3D_RGBD_VERSION_INFO stVersion;
	if (QT_ASSERT_OK(MV3D_RGBD_GetSDKVersion(&stVersion)))
	{
		ui.textEdit->insertPlainText(QString("Get SDK Version failed") + QString("\n"));
		ui.pushButton->setEnabled(true);
		return;
	}
	ui.textEdit->insertPlainText(QString::fromStdString("dll version : "+std::to_string(int(stVersion.nMajor)) + "." + std::to_string(int(stVersion.nMinor)) + "." +
		std::to_string(int(stVersion.nRevision))) + QString(getLocalTime() + QString("\n"))
	);
	if (QT_ASSERT_OK(MV3D_RGBD_Initialize()))
	{
		ui.textEdit->insertPlainText(QString("SDK run environment initialization failed") + QString("\n"));
		ui.pushButton->setEnabled(true);
		return;
	}
	unsigned int nDevNum = 0;
	uint32_t choose;
	if (ui.radioButton->isChecked())
		choose = DeviceType_USB;
	else  if (ui.radioButton_2->isChecked())
		choose = DeviceType_Ethernet;
	else
		choose =(DeviceType_Ethernet | DeviceType_USB);
	if (QT_ASSERT_OK((MV3D_RGBD_GetDeviceNumber(choose, &nDevNum))))//网络/USB接口
	{
		ui.textEdit->insertPlainText(QString("the number of devices is 0,can not find any device\n") + QString("\n"));
		ui.pushButton->setEnabled(true);
		return;
	}
	if (nDevNum == 0)
	{
		ui.textEdit->insertPlainText(QString("the number of devices is 0,can not find any device\n") + QString("\n"));
		ui.pushButton->setEnabled(true);
		return;
	}
		
	ui.textEdit->insertPlainText(QString("MV3D_RGBD_GetDeviceNumber success! nDevNum:")+QString::number(nDevNum) + QString("\n"));
	 devs.resize(nDevNum);
	if (QT_ASSERT_OK(MV3D_RGBD_GetDeviceList(choose, &devs[0], nDevNum, &nDevNum)))
	{
		ui.textEdit->insertPlainText(QString("Get 3D cameras list failed\n") + QString("\n"));
		ui.pushButton->setEnabled(true);
		return;
	}
	for (int i=0;i<nDevNum;++i)
	{
		ui.comboBox->addItem(QString::number(i)+": "+QString(devs[i].chModelName));
		ui.textEdit->insertPlainText("index:"+QString::number(i)+" chSerialNumber:"+QString(devs[i].chSerialNumber)+" device name:"+QString(devs[i].chModelName)+"\n");
	}
	ui.pushButton->setEnabled(true);
	ui.pushButton_2->setEnabled(true);
}
void MainWindow::close_device()
{
	mtx_read_points.lock();
	if (QT_ASSERT_OK(MV3D_RGBD_Stop(points_get.handle))||QT_ASSERT_OK(MV3D_RGBD_CloseDevice(&points_get.handle))|| QT_ASSERT_OK(MV3D_RGBD_Release()))
	{
		ui.textEdit->insertPlainText("device closed failed\n");
		points_get.handle=nullptr;
		mtx_read_points.unlock();
		return ;
	};
	ui.textEdit->insertPlainText( "device closed\n");
	points_get.handle = nullptr;
	mtx_read_points.unlock();
	ui.pushButton_3->setEnabled(false);
	ui.pushButton_2->setEnabled(true);
}
void MainWindow::open_device()
{

	QString tmp= ui.comboBox->currentText();
	ui.textEdit->insertPlainText(tmp+"  tmp\n");
	int index = 0;
	ui.pushButton->setEnabled(false);//保证线程安全
	ui.pushButton_2->setEnabled(false);//保证线程安全，保证只能弄一次
	auto fun= [&](const std::vector<MV3D_RGBD_DEVICE_INFO>&devs_tmp)->bool
	{
		std::string ss=tmp.toStdString();
		std::string tmp2 = "";
		for (char it:ss)
		{
			if (it >= '0'&&it <= '9')
			{
				tmp2 += it;
			}
			else if (it == ':')
				break;
			else
				return false;
		}
		index = atoi(tmp2.c_str());
		return true;
	};
	if (tmp == ""||!fun(devs))
	{
		ui.textEdit->insertPlainText("no divece ,open failed\n");
		ui.pushButton_2->setEnabled(true);
		ui.pushButton->setEnabled(true);//保证线程安全
		return;
	}
	mtx_read_points.lock();
	//读取设备
	if (QT_ASSERT_OK(MV3D_RGBD_OpenDevice(&points_get.handle, &devs[index])))
	{
		ui.textEdit->insertPlainText("can not read this divece ,open failed\n");
		ui.pushButton_2->setEnabled(true); 
		ui.pushButton->setEnabled(true);  points_get.handle = nullptr;  mtx_read_points.unlock();//保证线程安全
		return;
	};
	MV3D_RGBD_PARAM stParam;
	stParam.enParamType = ParamType_Enum;
	stParam.ParamInfo.stEnumParam.nCurValue = ResolutionType_1280_720;//设置分辨率
	MV3D_RGBD_SetParam(points_get.handle, MV3D_RGBD_ENUM_RESOLUTION, &stParam);
	// 开始工作流程
	if (QT_ASSERT_OK(MV3D_RGBD_Start(points_get.handle)))
	{
		ui.textEdit->insertPlainText("this divece work failed\n");
		ui.pushButton_2->setEnabled(true);
		ui.pushButton->setEnabled(true);  points_get.handle = nullptr; mtx_read_points.unlock();//保证线程安全
		return;
	};
	ui.textEdit->insertPlainText("this divece work successfully\n");
	ui.pushButton->setEnabled(true); 
	ui.pushButton_4->setEnabled(true);
	points_get.canread = true; mtx_read_points.unlock();//保证线程安全,注意不更新开始采集的按钮
	ui.pushButton_3->setEnabled(true);
}

void MainWindow::refush_viewoption()
{

	mtx_color_option.lock();
	double r =isnumber( ui.lineEdit->text().toStdString());
	double g = isnumber(ui.lineEdit_4->text().toStdString());
	double b = isnumber(ui.lineEdit_5->text().toStdString());
	r =( r > 255 ? 255 : r)/255; g =( g > 255 ? 255 : g)/255; b = (b > 255 ? 255 : b)/255;

	color_set.background_color = { r,g,b };
	int optionchoose = ui.comboBox_2->currentIndex();
	if (optionchoose == 1)
	{
		mtx_read_points.lock();
		points_get.refush_color = true;
		mtx_read_points.unlock();
	}
	else
	{
		mtx_read_points.lock();
		points_get.refush_color = false;
		mtx_read_points.unlock();
	}
	switch (optionchoose)
	{
	case 0:
		color_set.color_option = open3d::visualization::PointColorOption(0);
		break;
	case 1:
		color_set.color_option = open3d::visualization::PointColorOption(1);
		break;
	case 2:
		color_set.color_option = open3d::visualization::PointColorOption(2);
		break;
	case 3:
		color_set.color_option = open3d::visualization::PointColorOption(3);
		break;
	case 4:
		color_set.color_option = open3d::visualization::PointColorOption(4);
		break;
	case 5:
		color_set.color_option= open3d::visualization::PointColorOption(9);
		break;
	default:
		break;
	}
	color_set.point_size = isnumber(ui.lineEdit_6->text().toStdString());
	if (!color_set.point_size)
	{
		color_set.point_size = 1;}
	color_set.refush_veiw = true;
	mtx_color_option.unlock();
}

void MainWindow::point_process()
{
	ui.pushButton_7->setEnabled(false);
	mtx_points_process.lock();
	
	points_pro.can_process = true;//通知读取静态点云
	//sem_points_pro.signal();//唤醒该线程
	mtx_points_process.unlock();
	ui.pushButton_7->setEnabled(true);
}

void MainWindow::set_bouding_box()
{
	mtx_points_process.lock();
	points_pro.choose = 1;//包围盒;
	points_pro.refush_p[1] = true;
	//points_pro.pra[0] =ui/255;
	points_pro.pra[0]=ui.lineEdit_box_r->text().toFloat()/255;
	points_pro.pra[1]=ui.lineEdit_box_g->text().toFloat()/255;
	points_pro.pra[2] =ui.lineEdit_box_b->text().toFloat()/255;
	
	mtx_points_process.unlock();

}

void MainWindow::set_ROI()
{
	mtx_points_process.lock();
	points_pro.choose = 2;//ROI
	points_pro.refush_p[2] = true;
	mtx_points_process.unlock();
}

void MainWindow::set_filter()
{
	mtx_points_process.lock();
	points_pro.choose = 4;//
	points_pro.pra[0] = ui.lineEdit_2->text().toFloat();
	points_pro.refush_p[4] = true;
	mtx_points_process.unlock();
}



