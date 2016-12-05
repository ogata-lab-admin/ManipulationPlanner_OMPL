// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"

/*
 * Example implementational code for IDL interface Manipulation::TrajectoryPlanner
 */
RTC_TrajectoryPlannerSVC_impl::RTC_TrajectoryPlannerSVC_impl()
{
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{
}

void RTC_TrajectoryPlannerSVC_impl::setMesh(Manipulation::MultiMesh* robotsMesh, Manipulation::Node* envMesh){
	m_robotsMesh = robotsMesh;
	m_envMesh = envMesh;
}
/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::RETURN_VALUE RTC_TrajectoryPlannerSVC_impl::planTrajectory(const Manipulation::JointPose& start, const Manipulation::JointPose& goal, Manipulation::JointTrajectory_out trajectory)
{
  createSampler();

  jSampler->setPlanningMethod(method);
  jSampler->setArm();
  jSampler->setMesh(m_robotsMesh,m_envMesh);

  if(jSampler->planWithSimpleSetup(start, goal, trajectory)){
	  delete jSampler;
	  return Manipulation::RETVAL_OK;
  }
  delete jSampler;
  return Manipulation::RETVAL_NOT_FOUND;
}



// End of example implementational code



