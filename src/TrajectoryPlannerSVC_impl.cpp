// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"







// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
	m_robotJointInfo = new Manipulation::RobotJointInfo();
	m_jointSampler = new JointStateSampler();
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)
{
	Manipulation::ReturnValue* result;

	m_rtcPtr->callGetModelInfo(robotID, m_robotJointInfo);

	m_jointSampler->initSampler(robotID, m_robotJointInfo);
	m_jointSampler->setComp(m_rtcPtr);
	m_jointSampler->setPlanningMethod(m_planningMethod);

	m_jointSampler->planWithSimpleSetup(startRobotJointInfo, goalRobotJointInfo, manipPlan);
	std::cout << "Finish Planning"<<std::endl;
  return result;
}



// End of example implementational code



