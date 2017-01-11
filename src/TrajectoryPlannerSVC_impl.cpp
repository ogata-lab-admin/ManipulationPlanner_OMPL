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
	m_rtcPtr->callGetModelInfo(robotID, m_robotJointInfo);

	m_jointSampler = new JointStateSampler(robotID, m_robotJointInfo);
	m_jointSampler->setComp(m_rtcPtr);
	m_jointSampler->setPlanningMethod(m_planningMethod);

	std::cout << m_jointSampler->planWithSimpleSetup(startRobotJointInfo, goalRobotJointInfo, manipPlan) << std::endl;

}


// End of example implementational code



