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

//TODO: Generate these params automatically from robot mesh data
void JointStateSampler::setArm(){
	m_armBase = V3(-100.0, 445.0, 0.0);
	//                     r,p,y
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,250.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,130.0)));
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,100.0)));
	m_arm.push_back(TLink(V3(0,1,0), V3(0,0,105.0)));
	m_arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
}

void JointStateSampler::setMesh(Manipulation::MultiMesh* robotsMesh, Manipulation::Node* envMesh){
	//m_armMeshCC->setMeshData(robotsMesh,envMesh);
	//for(int i=0;i<21;i++){
	m_rtcomp->m_collisionDetector->addCollisionPair();
}


bool JointStateSampler::isStateValid(const ob::State *state)
{
    //casting: state=>state_vec=>angles
    const ob::RealVectorStateSpace::StateType *state_vec= state->as<ob::RealVectorStateSpace::StateType>();
	std::vector<double> angles(m_arm.size());

	for (size_t  i = 0; i < m_arm.size(); i++){
	angles[i] = (*state_vec)[i];
	}

	//From the joint coordinates to the SE3 coordinate of axes
	std::vector<TVector> axesPos;//joints coordinate in SE(3) space
	ForwardKinematics(m_arm, angles, m_armBase, axesPos);

	//call mesh collision check
	//return m_armMeshCC->isNotCollided(axesPos);
	return queryIntersectionForDefinedPairs(true,CharacterPositionSequence positions,LinkPairSequence collidedPairs)

}

/* Compute the forward kinematics of a manipulator ``linkes''
whose joint angles are specified by ``angles'',
and the base position is ``base''.
The result is stored into ``result'' that contains
the base position and every position of the end-points.

i.e. result.size()==linkes.size()+1 */
void JointStateSampler::ForwardKinematics(const std::vector<TLink> &linkes,
                                 const std::vector<double> &angles, const TVector &base,
                                 std::vector<TVector> &result)
{
	// assert(linkes.size() == angles.size());
	assert(base.size() == 3);
	result.resize(linkes.size() + 1);
	TMatrix R(3, 3), Rtmp(3, 3);
	R(0, 0) = R(1, 1) = R(2, 2) = 1.0;
	R(0, 1) = R(0, 2) = R(1, 0) = R(1, 2) = R(2, 0) = R(2, 1) = 0.0;
	result[0] = base;
	for (size_t i(1); i < result.size(); ++i) {
		result[i].resize(3);
		Rtmp = prod(R, RfromAxisAngle(linkes[i - 1].Axis, angles[i - 1]));
		R = Rtmp;
		result[i] = prod(R, linkes[i - 1].End) + result[i - 1];
	}
}

bool JointStateSampler::planWithSimpleSetup(const Manipulation::JointPose& startPos, const Manipulation::JointPose& goalPos, Manipulation::JointTrajectory_out  traj)
{
	// Construct the state space where we are planning
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(m_arm.size()));

	ob::RealVectorBounds bounds(m_arm.size());
	for (int i = 0; i < m_arm.size(); ++i){
		bounds.setLow(i, -M_PI);
		bounds.setHigh(i, M_PI);
	}
	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// Instantiate SimpleSetup
	og::SimpleSetup ss(space);

	// Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&JointStateSampler::isStateValid, this, _1));

    //set start and goal
	assert(startPos.length()==goalPos.length());

	ob::ScopedState<ob::RealVectorStateSpace> start(space);
	for (int i = 0; i < m_arm.size(); ++i){
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = startPos[i];
	}
	ob::ScopedState<ob::RealVectorStateSpace> goal(space);
	for (int i = 0; i < m_arm.size(); ++i){
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
