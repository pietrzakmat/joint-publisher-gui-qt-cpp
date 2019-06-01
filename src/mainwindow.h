#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt
#include <QMainWindow>
#include <QApplication>
#include <QObject>
#include <QString>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>

// std
#include <iostream>
#include <string>

// ROS
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#endif

// KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "jntinfo.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
signals:

public slots:

    void slot_reset();
    void slot_home();
    void slot_set_joint(int val);

private:
    sensor_msgs::JointState m_joint_state;
    ros::Publisher m_joint_publisher;

    std::vector<QSlider*> m_jnt_SLD;
    std::vector<QLabel*> m_jnt_LBL;
    double m_sld_jnt_step;

    QWidget* m_centralWidget;

    JntInfo m_jnt_info;
    urdf::Model m_robot_model;
    KDL::Tree m_kdl_tree;
};

#endif // MAINWINDOW_H
