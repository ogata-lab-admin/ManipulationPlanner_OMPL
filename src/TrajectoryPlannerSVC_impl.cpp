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
	  createSampler();
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{
	  delete jSampler;
}

void RTC_TrajectoryPlannerSVC_impl::setMesh(Manipulation::MultiMesh robotsMesh, Manipulation::Node envMesh){
	jSampler->setMesh(robotsMesh,envMesh);
}
/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::RETURN_VALUE RTC_TrajectoryPlannerSVC_impl::planTrajectory(const Manipulation::JointPose& start, const Manipulation::JointPose& goal, Manipulation::JointTrajectory_out trajectory)
{
  jSampler->setPlanningMethod(method);
  jSampler->setArm();

  if(jSampler->planWithSimpleSetup(start, goal, trajectory)){
	  return Manipulation::RETVAL_OK;
  }
  return Manipulation::RETVAL_NOT_FOUND;
}



// End of example implementational code



