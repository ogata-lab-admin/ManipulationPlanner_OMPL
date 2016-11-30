// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"
#include "iostream"

Planning planner("../plot/test_arm1.dat");

/*
 * Example implementational code for IDL interface RTC::TrajectoryPlanner
 */
RTC_TrajectoryPlannerSVC_impl::RTC_TrajectoryPlannerSVC_impl()
{
  // Please add extra destructor code here.
}


RTC_TrajectoryPlannerSVC_impl::~RTC_TrajectoryPlannerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_TrajectoryPlannerSVC_impl::planTrajectory(const RTC::jointPos start, const RTC::jointPos goal, RTC::jPosTraj trajectory)
{
  planner.setPlanningMethod(this.planningMethod);
  //not passed arguments(start, goal, trajectory) yet
  planner.planWithSimpleSetup();
  return RTC::RETVAL_OK;
}



// End of example implementational code



