// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"

/*
 * Example implementational code for IDL interface RTC::TrajectoryPlanner
 */
RTC_TrajectoryPlannerSVC_impl::RTC_TrajectoryPlannerSVC_impl()
{
	  createSampler();
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{
	  delete jSampler;
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_TrajectoryPlannerSVC_impl::planTrajectory(const RTC::JointPose& start, const RTC::JointPose& goal, RTC::JointTrajectory_out trajectory)
{
  jSampler->setPlanningMethod(method);
  jSampler->setArm();

  if(jSampler->planWithSimpleSetup(start, goal, trajectory)){
	  return RTC::RETVAL_OK;
  }
  return RTC::RETVAL_NOT_FOUND;
}



// End of example implementational code



