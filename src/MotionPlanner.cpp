#include "MotionPlanner.h"

#include <boost/bind.hpp>
using namespace std;

JointStateSampler::JointStateSampler()
{
	//m_armMeshCC = new ArmMeshCollisionChecker();
	//m_armMeshCC->debug_setRobotMesh();
}

JointStateSampler::~JointStateSampler(){
	//delete m_armMeshCC;
}

void JointStateSampler::setAngleLimits(Manipulation::RobotJointInfo joints){
	m_jointNum = joints.size();

	for (size_t  i = 0; i < m_jointNum; i++){
		JointLimit limit = {joints.minAngl, joints.maxAngle};
		m_jointLimits[i].push_back(limit);
	}
	/*
 	m_armBase = V3(-100.0, 445.0, 0.0);
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,250.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,130.0)));
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,100.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,105.0)));
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
	*/
}

bool JointStateSampler::isStateValid(const ob::State *state)
{
    //casting: state=>state_vec=>angles
    const ob::RealVectorStateSpace::StateType *state_vec= state->as<ob::RealVectorStateSpace::StateType>();
	std::vector<double> angles(m_jointNum);

	for (size_t  i = 0; i < m_jointNum; i++){
	angles[i] = (*state_vec)[i];
	}

//	angles=>Manipulation::RobotJointInfo
//	return !isCollide();
	return true;

}

bool JointStateSampler::planWithSimpleSetup(const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan)
{
	// Construct the state space where we are planning
	ob::StateSpacePtr space(new ob::RealVectorStateSpm_jointNumze()));

	ob::RealVectorBounds bounds(m_jointNum);
	for (int i = 0; i < m_jointNum; ++i){
		bounds.setLow(i, m_jointLimits[i].min);
		bounds.setHigh(i, m_jointLimits[i].max);
	}
	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// Instantiate SimpleSetup
	og::SimpleSetup ss(space);

	// Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&JointStateSampler::isStateValid, this, _1));

    //set start and goal
	assert(startPos.length()==goalPos.length());

	ob::ScopedState<ob::RealVectorStateSpace> start(space);
	for (int i = 0; i < m_jointNum; ++i){
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = startPos[i];
	}
	ob::ScopedState<ob::RealVectorStateSpace> goal(space);
	for (int i = 0; i < m_jointNum; ++i){
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = goalPos[i];
	}
	ss.setStartAndGoalStates(start, goal);


	// setting collision checking resolution to 1% of the space extent
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

	// If we have a solution,
	if (ss.solve(10)) {
		cout << "Found solution:" << endl;
		ss.simplifySolution();
		ss.getSolutionPath().print(cout);
		//std::ofstream ofs("../plot/path.dat");
		//path.printAsMatrix(ofs);

		//traj =PathGeo2JSTraje(ss.getSolutionPath());

		return true;

	} else {
		cout << "No solution found" << endl;
		return false;
	}

}
