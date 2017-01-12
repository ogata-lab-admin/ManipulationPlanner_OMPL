// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "TrajectoryPlannerSVC_impl.h"







// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
	m_robotJointInfo = new Manipulation::RobotJointInfo();
	m_jointSampler = new JointStateSampler();
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)
{
	Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
	result->id = Manipulation::OK;
	result->message = CORBA::string_dup("OK");

	std::cout << "---Start Path Planning---" << std::endl;

    Manipulation::RobotIdentifier* robotID;
    robotID = new Manipulation::RobotIdentifier();
    robotID->name = CORBA::string_dup("orochi");

	std::cout << "---Get Robot Joints Data--" << std::endl;
	m_rtcPtr->callGetModelInfo((*robotID), m_robotJointInfo);
	//showJointsData();

	m_jointSampler->initSampler((*robotID), m_robotJointInfo);
	m_jointSampler->setComp(m_rtcPtr);
	m_jointSampler->setPlanningMethod(m_planningMethod);

	std::cout << "---Planning..---" << std::endl;
	m_jointSampler->planWithSimpleSetup(startJointAngles, goalJointAngles, manipPlan);

	std::cout << "---Finish!---"<<std::endl;

	return result._retn();
}

void Manipulation_ManipulationPlannerServiceSVC_impl::showJointsData(){
	for(int i=0; i<m_robotJointInfo->jointParameterSeq.length();i++){
		std::cout << "Joint"<<i<<" name:" <<m_robotJointInfo->jointParameterSeq[i].name << std::endl;
		//m_robotJointInfo->jointParameterSeq[i].jointType= Manipulation::JOINT_TYPE::JOINT_ROTATE;
		std::cout << "Joint Type:" <<m_robotJointInfo->jointParameterSeq[i].jointType << std::endl;
		std::cout << "Joint axis vector:";
		for(int j=0; j<3;j++){
			std::cout <<m_robotJointInfo->jointParameterSeq[i].axis[j] << " ";
		}
		std::cout << std::endl;
		std::cout << "Offset matrix:"<< std::endl;
		for(int j=0; j<3;j++){
			for(int k =0; k<4;k++){
				std::cout <<m_robotJointInfo->jointParameterSeq[i].offset[j][k]<< " ";
			}
			std::cout << std::endl;
		}
		std::cout << "Upper Limit:" <<m_robotJointInfo->jointParameterSeq[i].limit.upper << std::endl;
		std::cout << "Lower Limit" <<m_robotJointInfo->jointParameterSeq[i].limit.lower <<"\n"<< std::endl;

	}
}

// End of example implementational code



