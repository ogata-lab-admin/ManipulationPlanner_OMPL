// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"
#include "iostream"

/*
 * Example implementational code for IDL interface RTC::TrajectoryPlanner
 */
RTC_TrajectoryPlannerSVC_impl::RTC_TrajectoryPlannerSVC_impl()
{
  // Please add extra constructor code here.
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
  // Please insert your code here and remove the following warning pragma
  std::cout << "not impled yet" << std::endl;
  return RTC::RETVAL_OK;
}



// End of example implementational code



