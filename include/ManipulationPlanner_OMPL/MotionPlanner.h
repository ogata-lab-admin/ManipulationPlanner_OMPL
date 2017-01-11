#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include <ompl/geometric/SimpleSetup.h>

//Planning Methods
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/est/EST.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <ostream>

#include "TrajectoryPlannerSkel.h"
#include "ManipulationPlanner_OMPL.h"
#include <rtm/Manager.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;


class Manipulation_ManipulationPlannerServiceSVC_impl;
class ManipulationPlanner_OMPL;

struct JointLimit{
    double max;
    double min;
};

class JointStateSampler{
  public:
    JointStateSampler();
    ~JointStateSampler();

    void initSampler(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo* joints);

    void setPlanningMethod(int m){selector = m;}
    void setAngleLimits();

    bool planWithSimpleSetup(const Manipulation::JointAngleSeq& startJointAngleSeq, const Manipulation::JointAngleSeq& goalJointAngleSeq, Manipulation::ManipulationPlan_out manipPlan);

    void setComp(ManipulationPlanner_OMPL* rtc){m_rtcomp = rtc;}

  protected:

    bool isStateValid(const ob::State *state);

    ManipulationPlanner_OMPL* m_rtcomp;
    int m_jointNum = 6;
    int m_planningMethod = 1;
    std::vector<JointLimit> m_jointLimits;
    Manipulation::RobotIdentifier m_robotID;
    Manipulation::RobotJointInfo* m_robotJointInfo;
    Manipulation::CollisionPairSeq* m_collision;

    int selector;

};
#endif
