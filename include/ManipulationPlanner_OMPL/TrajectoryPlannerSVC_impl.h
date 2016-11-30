// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.h
 * @brief Service implementation header of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSkel.h"
#include "MotionPlanner.h"

#ifndef TRAJECTORYPLANNERSVC_IMPL_H
#define TRAJECTORYPLANNERSVC_IMPL_H
 
/*!
 * @class TrajectoryPlannerSVC_impl
 * Example class implementing IDL interface RTC::TrajectoryPlanner
 */
class RTC_TrajectoryPlannerSVC_impl
 : public virtual POA_RTC::TrajectoryPlanner,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~TrajectoryPlannerSVC_impl();
   int planningMethod = 1;

 public:
  /*!
   * @brief standard constructor
   */
   RTC_TrajectoryPlannerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_TrajectoryPlannerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE planTrajectory(const RTC::jointPos start, const RTC::jointPos goal, RTC::jPosTraj trajectory);

   void setPlanningMethod(int m){planningMethod=m;}
};



#endif // TRAJECTORYPLANNERSVC_IMPL_H


