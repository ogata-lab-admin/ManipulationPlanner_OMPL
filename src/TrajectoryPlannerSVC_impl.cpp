// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"
#include <rtm/DataFlowComponentBase.h>

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, RTC::Pose3D goalPose, Manipulation::ManipulationPlan_out manipPlan)
{
	  createSampler();
	  jSampler->setPlanningMethod(method);
	  jSampler->setArm();

	  if(jSampler->planWithSimpleSetup(start, goal, trajectory)){
		  delete jSampler;
		  return Manipulation::RETVAL_OK;
	  }
	  delete jSampler;
	  return Manipulation::RETVAL_NOT_FOUND;

}


void Manipulation_ManipulationPlannerServiceSVC_impl::setMesh(Manipulation::MultiMesh* robotsMesh, Manipulation::Node* envMesh){
	jSampler->setMesh(robotsMesh,envMesh);
}



// End of example implementational code



