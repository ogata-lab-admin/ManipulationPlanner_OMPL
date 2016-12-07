// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"
#include <rtm/DataFlowComponentBase.h>

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}

Manipulation::RobotJointInfo Manipulation_ManipulationPlannerServiceSVC_impl::invKinematics(RTC::Pose3D pose)
{
	//;
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, RTC::Pose3D goalPose, Manipulation::ManipulationPlan_out manipPlan)
{
	  m_jointSampler = new JointStateSampler();

	  m_jointSampler->setPlanningMethod(m_planningMethod);
	  m_rtcPtr->m_modelServer->getModelInfo(robotID, m_robotJointInfo);
	  m_jointSampler->setArm(m_robotJointInfo);

	  if(m_jointSampler->planWithSimpleSetup(startRobotJointInfo, invKinematics(goalPose), manipPlan)){
		  delete m_jointSampler;
		  return Manipulation::RETVAL_OK;
	  }
	  delete m_jointSampler;
	  return Manipulation::RETVAL_NOT_FOUND;

}


// End of example implementational code



