// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.h
 * @brief Service implementation header of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSkel.h"
#include "ManipulationPlanner_OMPL.h"
#include "MotionPlanner.h"

#ifndef TRAJECTORYPLANNERSVC_IMPL_H
#define TRAJECTORYPLANNERSVC_IMPL_H

/*!
 * @class ManipulationPlannerServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ManipulationPlannerService
 */
class Manipulation_ManipulationPlannerServiceSVC_impl
 : public virtual POA_Manipulation::ManipulationPlannerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ManipulationPlannerServiceSVC_impl();
	  int m_planningMethod = 1;
	  JointStateSampler* m_jointSampler;
	  ManipulationPlanner_OMPL* m_rtcPtr;
	  Manipulation::RobotJointInfo m_robotJointInfo;

 public:
  /*!
   * @brief standard constructor
   */
   ManipulationPlannerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ManipulationPlannerServiceSVC_impl();

   // attributes and operations
   void planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, RTC::Pose3D goalPose, Manipulation::ManipulationPlan_out manipPlan);

   void setPlanningMethod(int m){m_planningMethod=m;}
   void setComp(ManipulationPlanner_OMPL* rtc){m_rtcPtr=rtc;}

   Manipulation::RobotJointInfo invKinematics(RTC::Pose3D pose, Manipulation::RobotJointInfo joints);

};

#endif // TRAJECTORYPLANNERSVC_IMPL_H


