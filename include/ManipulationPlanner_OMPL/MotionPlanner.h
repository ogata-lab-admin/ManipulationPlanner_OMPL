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

#include "CollisionChecker.h"
#include "TrajectoryPlannerSkel.h"
#include "p4-arm-helper.h"
namespace og = ompl::geometric;

class JointStateSampler{
  public:
    JointStateSampler();
    ~JointStateSampler();

    void setPlanningMethod(int m){selector = m;}
    void setArm();
    void setMesh(Manipulation::MultiMesh robotsMesh, Manipulation::Node envMesh);

    bool planWithSimpleSetup(const Manipulation::JointPose& startPos, const Manipulation::JointPose& goalPos, Manipulation::JointTrajectory_out traj);

  protected:
    bool isStateValid(const ob::State *state);
    void ForwardKinematics(const std::vector<TLink> &linkes,
                           const std::vector<double> &angles, const TVector &base,
                           std::vector<TVector> &result);

    ArmMeshCollisionChecker* m_armMeshCC;

    int m_planningMethod = 1;
    std::vector<TLink> m_arm;  // Manipulator
    TVector m_armBase;  // The base position of Manipulator

    int selector;

};
#endif
