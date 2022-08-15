#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_open3d_demo.h"

class open3d_demo : public QMainWindow
{
    Q_OBJECT

public:
    open3d_demo(QWidget *parent = Q_NULLPTR);

private:
    Ui::open3d_demoClass ui;
};
