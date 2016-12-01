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
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{

}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE planTrajectory(const RTC::JointPose start, const RTC::JointPose goal, RTC::JointSpaceTrajectory trajectory);
{
  instantiatePlanner();

  planner->setPlanningMethod(method);
  //not passed arguments(start, goal, trajectory) yet
  planner->setStartAndGoal(start, goal);
  if(planner->planWithSimpleSetup(trajectory)){
	  delete planner;
	  return RTC::RETVAL_OK;
  }

  delete planner;
  return RTC::RETVAL_UNKNOWN_ERROR;
}



// End of example implementational code



