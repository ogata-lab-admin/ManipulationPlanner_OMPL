#include "MotionPlanner.h"

#include <boost/bind.hpp>
using namespace std;

JointStateSampler::JointStateSampler(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo* joints)
{
	m_robotID = robotID;
	m_robotJointInfo = joints;
	setAngleLimits();
	m_collision = new Manipulation::CollisionInfo();
}

JointStateSampler::~JointStateSampler(){
}

void JointStateSampler::setAngleLimits(){
	m_jointNum = m_robotJointInfo->jointInfoSeq.length();

	for (size_t  i = 0; i < m_jointNum; i++){
		JointLimit limit = {m_robotJointInfo->jointInfoSeq[i].minAngle, m_robotJointInfo->jointInfoSeq[i].maxAngle};
		m_jointLimits.push_back(limit);
	}
}

bool JointStateSampler::isStateValid(const ob::State *state)
{
    //casting: state=>state_vec=>angles
    const ob::RealVectorStateSpace::StateType *state_vec= state->as<ob::RealVectorStateSpace::StateType>();
	//std::vector<double> angles(m_jointNum);

	for (size_t  i = 0; i < m_jointNum; i++){
	//angles[i] = (*state_vec)[i];
	    m_robotJointInfo->jointInfoSeq[i].jointAngle= (*state_vec)[i];
	}

//	angles=>Manipulation::RobotJointInfo
	return !m_rtcomp->callIsCollide(m_robotID, *m_robotJointInfo, m_collision);

}

bool JointStateSampler::planWithSimpleSetup(const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan)
{
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(m_jointNum));

	ob::RealVectorBounds bounds(m_jointNum);
	for (int i = 0; i < m_jointNum; ++i){
		bounds.setLow(i, m_jointLimits[i].min);
		bounds.setHigh(i, m_jointLimits[i].max);
	}
	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	og::SimpleSetup ss(space);

	ss.setStateValidityChecker(boost::bind(&JointStateSampler::isStateValid, this, _1));

	assert(startRobotJointInfo.jointInfoSeq.length()==goalRobotJointInfo.jointInfoSeq.length());

	ob::ScopedState<ob::RealVectorStateSpace> start(space);
	for (int i = 0; i < m_jointNum; ++i){
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = startRobotJointInfo.jointInfoSeq[i].jointAngle;
	}
	ob::ScopedState<ob::RealVectorStateSpace> goal(space);
	for (int i = 0; i < m_jointNum; ++i){
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = goalRobotJointInfo.jointInfoSeq[i].jointAngle;
	}
	ss.setStartAndGoalStates(start, goal);

	ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

	if (selector == 1) {
		ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 2) {
		ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 3) {
		ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 4) {
		ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 5) {
		ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 6) {
		ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 7) {
		ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 8) {
		ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	} else if (selector == 9) {
		ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
		ss.setPlanner(planner);
	}

	if (ss.solve(10)) {
		cout << "Found solution:" << endl;
		ss.simplifySolution();
		
		/*
        og::PathGeometric path;
        path = ss.getSolutionPath()
        path.getSolutionPath().print(cout);
		*/
		ss.getSolutionPath().print(cout);
		//std::ofstream ofs("../plot/path.dat");
		//path.printAsMatrix(ofs);
		/*
                for(int i=0; i<path.length(); i++){
                        for(int j =0; j<m_jointNum; j++){
                                manipPlan->robotJointInfoSeq[i].jointInfoSeq[j].jointAngle = path.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[j]
                        }
                }
		*/

		return true;

	} else {
		cout << "No solution found" << endl;
		return false;
	}

}
