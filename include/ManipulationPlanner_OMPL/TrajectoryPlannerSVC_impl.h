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

  //std::string config = "../plot/test_arm1.dat";
  int method = 1;
  JointStateSampler* jSampler;

 public:
  /*!
   * @brief standard constructor
   */
   RTC_TrajectoryPlannerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_TrajectoryPlannerSVC_impl();

   void createSampler(){jSampler = new JointStateSampler();}

   // attributes and operations
   RTC::RETURN_VALUE planTrajectory(const RTC::JointPose& start, const RTC::JointPose& goal, RTC::JointTrajectory_out trajectory);

   void passPlanningMethod(int m){method=m;}
   //void setConfig(std::string c){config = c;}

};



#endif // TRAJECTORYPLANNERSVC_IMPL_H


