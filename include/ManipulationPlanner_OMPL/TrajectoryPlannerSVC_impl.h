// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.h
 * @brief Service implementation header of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSkel.h"

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
   void test();

};



#endif // TRAJECTORYPLANNERSVC_IMPL_H


