#ifndef JNTINFO_H
#define JNTINFO_H

#include <iostream>
#include <string>

// ROS
#ifndef Q_MOC_RUN
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#endif

// KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

class JntInfo
{
public:
    bool readJoints(urdf::Model &robot_model, KDL::Tree &kdl_tree)
    {
        KDL::SegmentMap segmentMap = kdl_tree.getSegments();
        m_nr_of_jnts = kdl_tree.getNrOfJoints();
        m_joint_min.resize(m_nr_of_jnts);
        m_joint_max.resize(m_nr_of_jnts);
        m_joint_vel_max.resize(m_nr_of_jnts);
        m_joint_names.resize(m_nr_of_jnts);

        for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end(); ++seg_it)
        {
            if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
            {
              const urdf::JointConstSharedPtr joint = robot_model.getJoint(seg_it->second.segment.getJoint().getName().c_str());
                // check, if joint can be found in the URDF model of the robot
                if (!joint)
                {
                    std::cout << "Joint " << joint->name.c_str() << "has not been found in the URDF robot model! Aborting ..." <<  std::endl;
                    return false;
                }
                // extract joint information
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                {
                    double lower = 0.0, upper = 0.0, vel_limit = 0.0;
                    unsigned int has_pos_limits = 0, has_vel_limits = 0;

                    if ( joint->type != urdf::Joint::CONTINUOUS )
                    {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                        has_pos_limits = 1;
                        if (joint->limits->velocity)
                        {
                            has_vel_limits = 1;
                            vel_limit = joint->limits->velocity;
                        }
                        else
                        {
                            has_vel_limits = 0;
                            vel_limit = 0.0;
                        }
                    }
                    else
                    {
                        lower = -M_PI;
                        upper = M_PI;
                        has_pos_limits = 0;
                        if (joint->limits && joint->limits->velocity)
                        {
                            has_vel_limits = 1;
                            vel_limit = joint->limits->velocity;
                        }
                        else
                        {
                            has_vel_limits = 0;
                            vel_limit = 0.0;
                        }
                    }

                    m_joint_names[seg_it->second.q_nr] = (joint->name);

                    m_joint_min(seg_it->second.q_nr) = lower;
                    m_joint_max(seg_it->second.q_nr) = upper;
                    m_joint_vel_max(seg_it->second.q_nr) = vel_limit;

    //                std::cout << "joint->name = " << joint->name
    //                          << "pos_min = " << lower
    //                          << ", pos_max = " << upper
    ////                          << ", vel_max = " << vel_limit
    //                          << std::endl;
                }
            }
        }
        return true;
    }

public:
    unsigned int m_nr_of_jnts;
    std::vector<std::string> m_joint_names;
    KDL::JntArray m_joint_min;
    KDL::JntArray m_joint_max;
    KDL::JntArray m_joint_vel_max;
};

#endif // JNTINFO_H
