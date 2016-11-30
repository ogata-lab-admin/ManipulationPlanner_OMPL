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
  instantiatePlanner();
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{
  delete planner;
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_TrajectoryPlannerSVC_impl::planTrajectory(const RTC::jointPos start, const RTC::jointPos goal, RTC::jPosTraj trajectory)
{
  planner->setPlanningMethod(method);
  //not passed arguments(start, goal, trajectory) yet
  planner->planWithSimpleSetup();
  return RTC::RETVAL_OK;
}



// End of example implementational code



