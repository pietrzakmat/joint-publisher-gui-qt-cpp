#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
     m_sld_jnt_step = 20.0f;
    if (ros::ok())
    {
//        std::cout << "ROS initialized" << std::endl;

        // gets the location of the robot description on the parameter server 
        if (!m_robot_model.initParam("robot_description"))
        {
            std::cout << "Error reading model" << std::endl;
        }

        if (!kdl_parser::treeFromUrdfModel(m_robot_model, m_kdl_tree))
        {
            std::cout << "Failed to extract kdl tree from xml robot description" << std::endl;
        }

        m_jnt_info.readJoints(m_robot_model, m_kdl_tree);

        ros::NodeHandle n;
        m_joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
        // message declarations
        m_joint_state.name.resize(m_jnt_info.m_nr_of_jnts);
        m_joint_state.position.resize(m_jnt_info.m_nr_of_jnts);
    }

    // Horizontal layout with 3 buttons
    QVBoxLayout *vLayout = new QVBoxLayout;
    QPushButton *bHome = new QPushButton("Home");
    QPushButton *bReset = new QPushButton("Reset");

    vLayout->addWidget(bHome);
    vLayout->addWidget(bReset);

    m_jnt_SLD.resize(m_jnt_info.m_nr_of_jnts);
    m_jnt_LBL.resize(m_jnt_info.m_nr_of_jnts);

    for (uint i = 0; i < m_jnt_info.m_nr_of_jnts; ++i) {
        m_joint_state.name[i] = m_jnt_info.m_joint_names.at(i);

        m_jnt_SLD[i] = new QSlider(Qt::Horizontal, this);
        m_jnt_SLD[i]->setRange(0, static_cast<int>(m_sld_jnt_step));
        m_jnt_LBL[i] = new QLabel();
        m_jnt_LBL[i]->setText(QString::fromStdString(m_joint_state.name[i]) + ": " /*+ QString::number(0.0f)*/);
        vLayout->addWidget(m_jnt_LBL[i]);
        vLayout->addWidget(m_jnt_SLD[i]);
        connect(m_jnt_SLD[i], SIGNAL(valueChanged(int)), this, SLOT(slot_set_joint(int)));
    }

    // Outer Layer
    QVBoxLayout *mainLayout = new QVBoxLayout;

    // Add the previous two inner layouts
    mainLayout->addLayout(vLayout);

    m_centralWidget = new QWidget;
    // Set the outer layout as a main layout
    // of the widget
    m_centralWidget->setLayout(mainLayout);

    setCentralWidget(m_centralWidget);

    connect(bHome, SIGNAL(released()), this, SLOT(slot_home()));
    connect(bReset, SIGNAL(released()), this, SLOT(slot_reset()));
}

MainWindow::~MainWindow()
{
    delete m_centralWidget;
}

void MainWindow::slot_home()
{
//    std::cout << __PRETTY_FUNCTION__ << std::endl;
    if (!ros::ok())
    {
        std::cout << "Error: ros is not ok" << std::endl;
        return;
    }

    //update joint_state
    m_joint_state.header.stamp = ros::Time::now();

    for (unsigned i(0); i<m_joint_state.position.size(); i++)
        m_joint_state.position[i] = 0.0;

    m_joint_state.position[1] = -1.26; // shoulder_lift_joint
    m_joint_state.position[2] = 1.88; // elbow_joint

    for (unsigned i=0; i<m_jnt_SLD.size(); i++)
    {
        m_jnt_LBL[i]->setText(QString::fromStdString(m_joint_state.name[i]) + ": " + QString::number(m_joint_state.position[i],'f', 2));
        double oldRange = m_jnt_info.m_joint_max(i) - m_jnt_info.m_joint_min(i);
        int sldVal = static_cast<int>(m_joint_state.position[i] * 20 / oldRange + 10); // convert to new range
//        std::cout << "m_joint_state.position[i]=" << m_joint_state.position[i] << ", sldVal= " << sldVal << std::endl;
        m_jnt_SLD[i]->setValue(sldVal);
    }

    //send the joint state
    m_joint_publisher.publish(m_joint_state);

}

void MainWindow::slot_reset()
{
    if (!ros::ok())
    {
        std::cout << "Error: ros is not ok" << std::endl;
        return;
    }
    //update joint_state
    m_joint_state.header.stamp = ros::Time::now();

    for (unsigned i(0); i<m_joint_state.position.size(); i++)
        m_joint_state.position[i] = 0.0;

    for (unsigned i=0; i<m_jnt_SLD.size(); i++)
    {
        m_jnt_LBL[i]->setText(QString::fromStdString(m_joint_state.name[i]) + ": " + QString::number(m_joint_state.position[i],'f', 2));
        double oldRange = m_jnt_info.m_joint_max(i) - m_jnt_info.m_joint_min(i);
        int sldVal = static_cast<int>(m_joint_state.position[i] * 20 / oldRange + 10); // convert to new range
        m_jnt_SLD[i]->setValue(sldVal);
    }


    //send the joint state
    m_joint_publisher.publish(m_joint_state);
}

void MainWindow::slot_set_joint(int val)
{
    if (!ros::ok())
    {
        std::cout << "Error: ros is not ok" << std::endl;
        return;
    }

    QObject* obj_sender = sender();

    for (uint i=0; i<m_jnt_SLD.size(); i++)
    {
        if (obj_sender == m_jnt_SLD[i])
        {
            double newRange = m_jnt_info.m_joint_max(i) - m_jnt_info.m_joint_min(i);
            m_joint_state.position[i] = (val * newRange / m_sld_jnt_step) + m_jnt_info.m_joint_min(i); // convert to new range
            m_jnt_LBL[i]->setText(QString::fromStdString(m_joint_state.name[i]) + ": " + QString::number(m_joint_state.position[i],'f', 2));
        }
    }

    //update joint_state
    m_joint_state.header.stamp = ros::Time::now();

    //send the joint state
    m_joint_publisher.publish(m_joint_state);
}

//bool MainWindow::readJoints(urdf::Model &robot_model, KDL::Tree &kdl_tree, JntInfo &info)
//{
//    KDL::SegmentMap segmentMap = kdl_tree.getSegments();
////    tree_root_name = kdl_tree.getRootSegment()->second.segment.getName();
//    info.m_nr_of_jnts = kdl_tree.getNrOfJoints();
////    std::cout << "the tree's number of joints: " << nr_of_jnts << std:::endl;
//    info.joint_min.resize(info.nr_of_jnts);
//    info.joint_max.resize(info.nr_of_jnts);
//    info.joint_vel_max.resize(info.nr_of_jnts);
//    info.joint_names.resize(info.nr_of_jnts);

////    std::cout << "Extracting all joints from the tree, which are not of type KDL::Joint::None." << std::endl;
//    boost::shared_ptr<const urdf::Joint> joint;
//    for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end(); ++seg_it)
//    {
//        if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
//        {
//            joint = robot_model.getJoint(seg_it->second.segment.getJoint().getName().c_str());
//            // check, if joint can be found in the URDF model of the robot
//            if (!joint)
//            {
//                std::cout << "Joint " << joint->name.c_str() << "has not been found in the URDF robot model! Aborting ..." <<  std::endl;
//                return false;
//            }
//            // extract joint information
//            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//            {
////                std::cout << "getting information about joint: " << joint->name.c_str() << std::endl;
//                double lower = 0.0, upper = 0.0, vel_limit = 0.0;
//                unsigned int has_pos_limits = 0, has_vel_limits = 0;

//                if ( joint->type != urdf::Joint::CONTINUOUS )
//                {
////                    std::cout << "joint is not continuous." << std::endl;
//                    lower = joint->limits->lower;
//                    upper = joint->limits->upper;
//                    has_pos_limits = 1;
//                    if (joint->limits->velocity)
//                    {
//                        has_vel_limits = 1;
//                        vel_limit = joint->limits->velocity;
////                        std::cout << "joint has following velocity limit: " << vel_limit << std::endl;
//                    }
//                    else
//                    {
//                        has_vel_limits = 0;
//                        vel_limit = 0.0;
////                        std::cout << "joint has no velocity limit" << std::endl;
//                    }
//                }
//                else
//                {
////                    std::cout << "joint is continuous." << std::endl;
//                    lower = -M_PI;
//                    upper = M_PI;
//                    has_pos_limits = 0;
//                    if (joint->limits && joint->limits->velocity)
//                    {
//                        has_vel_limits = 1;
//                        vel_limit = joint->limits->velocity;
////                        std::cout << "joint has following velocity limit: " << vel_limit << std::endl;
//                    }
//                    else
//                    {
//                        has_vel_limits = 0;
//                        vel_limit = 0.0;
////                        std::cout << "joint has no velocity limit" << std::endl;
//                    }
//                }

//                info.joint_names[seg_it->second.q_nr] = (joint->name);

//                info.joint_min(seg_it->second.q_nr) = lower;
//                info.joint_max(seg_it->second.q_nr) = upper;
//                info.joint_vel_max(seg_it->second.q_nr) = vel_limit;

////                std::cout << "joint->name = " << joint->name
////                          << "pos_min = " << lower
////                          << ", pos_max = " << upper
//////                          << ", vel_max = " << vel_limit
////                          << std::endl;
//            }
//        }
//    }
//    return true;
//}
