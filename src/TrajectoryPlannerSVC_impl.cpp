// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
	m_robotJointInfo = new Manipulation::RobotJointInfo();
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan)
{
	//m_rtcPtr->callGetModelInfo(robotID, m_robotJointInfo);
	m_robotJointInfo->jointInfoSeq.length(7);
	//m_m_m_robotJointInfo->jointInfoSeq[0].name =CORBA::string_dup('1');
	m_robotJointInfo->jointInfoSeq[0].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[0].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[0].linkLength=0;
	m_robotJointInfo->jointInfoSeq[0].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[0].minAngle = -2.792;
	m_robotJointInfo->jointInfoSeq[0].maxAngle = 2.792;

	//m_m_m_robotJointInfo->jointInfoSeq[1].name =CORBA::string_dup('2');
	m_robotJointInfo->jointInfoSeq[1].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[1].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[1].linkLength=0;
	m_robotJointInfo->jointInfoSeq[1].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[1].minAngle = -1.134;
	m_robotJointInfo->jointInfoSeq[1].maxAngle = 2.268;

	//m_m_robotJointInfo->jointInfoSeq[2].name =CORBA::string_dup('3');
	m_robotJointInfo->jointInfoSeq[2].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[2].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[2].linkLength=0;
	m_robotJointInfo->jointInfoSeq[2].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[2].minAngle = -0.1745;
	m_robotJointInfo->jointInfoSeq[2].maxAngle = 2.617;

	//m_m_robotJointInfo->jointInfoSeq[3].name =CORBA::string_dup('4');
	m_robotJointInfo->jointInfoSeq[3].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[3].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[3].linkLength=0;
	m_robotJointInfo->jointInfoSeq[3].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[3].minAngle = -2.792;
	m_robotJointInfo->jointInfoSeq[3].maxAngle = 2.792;

	//m_m_robotJointInfo->jointInfoSeq[4].name =CORBA::string_dup('5');
	m_robotJointInfo->jointInfoSeq[4].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[4].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[4].linkLength=0;
	m_robotJointInfo->jointInfoSeq[4].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[4].minAngle = -1.658;
	m_robotJointInfo->jointInfoSeq[4].maxAngle = 2.356;


	//m_m_robotJointInfo->jointInfoSeq[5].name =CORBA::string_dup('6');
	m_robotJointInfo->jointInfoSeq[5].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[5].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[5].linkLength=0;
	m_robotJointInfo->jointInfoSeq[5].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[5].minAngle = -2.967;
	m_robotJointInfo->jointInfoSeq[5].maxAngle = 2.967;

	//gripper
	//m_m_robotJointInfo->jointInfoSeq[5].name =CORBA::string_dup('7');
	m_robotJointInfo->jointInfoSeq[6].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[6].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[6].linkLength=0;
	m_robotJointInfo->jointInfoSeq[6].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[6].minAngle = 0.0;
	m_robotJointInfo->jointInfoSeq[6].maxAngle = 90.0;

	m_jointSampler = new JointStateSampler(robotID, m_robotJointInfo);

	m_jointSampler->setComp(m_rtcPtr);

	std::cout<<m_rtcPtr<<std::endl;
	m_jointSampler->setPlanningMethod(m_planningMethod);
	std::cout << m_jointSampler->planWithSimpleSetup(startRobotJointInfo, goalRobotJointInfo, manipPlan) << std::endl;
	delete m_jointSampler;
}


// End of example implementational code



