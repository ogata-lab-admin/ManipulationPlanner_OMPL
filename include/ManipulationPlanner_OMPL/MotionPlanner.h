#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include <ompl/geometric/SimpleSetup.h>

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
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <ostream>

#include "TrajectoryPlannerSkel.h"
#include "p4-arm-helper.h"
#include "ManipulationPlanner_OMPL.h"
#include <rtm/Manager.h>

namespace og = ompl::geometric;

struct JointLimit{
    double max;
    double min;
};

class JointStateSampler{
  public:
    JointStateSampler();
    ~JointStateSampler();

    void setPlanningMethod(int m){selector = m;}
    void setAngleLimits(Manipulation::RobotJointInfo robotJointInfo);

    bool planWithSimpleSetup(const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan);

    void setComp(RTC::RtcBase* rtc){m_rtcomp = rtc;}

  protected:

    bool isStateValid(const ob::State *state);
    void ForwardKinematics(const std::vector<TLink> &linkes,
                           const std::vector<double> &angles, const TVector &base,
                           std::vector<TVector> &result);

    //ArmMeshCollisionChecker* m_armMeshCC;

    RTC::RtcBase* m_rtcomp;
    int m_jointNum;
    int m_planningMethod = 1;
    std::vector<JointLimit> m_jointLimits;


    int selector;

};
#endif
