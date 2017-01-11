// -*- C++ -*-
/*!
 * @file  ManipulationPlanner_OMPL.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "ManipulationPlanner_OMPL.h"
#include "MotionPlanner.h"

// Module specification
// <rtc-template block="module_spec">
static const char* manipulationplanner_ompl_spec[] =
  {
    "implementation_id", "ManipulationPlanner_OMPL",
    "type_name",         "ManipulationPlanner_OMPL",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "ogata-lab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.PlanningMethod", "(PRM,RRT,RRTConnect,RRTstar,LBTRRT,LaxyRRT,TRRT,pRRT,EST)",

    // Widget
    "conf.__widget__.PlanningMethod", "radio",
    // Constraints
    "conf.__constraints__.PlanningMethod", "(0,1,2,3,4,5,6,7,8)",

    "conf.__type__.PlanningMethod", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ManipulationPlanner_OMPL::ManipulationPlanner_OMPL(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_ManipulationPlannerServicePort("ManipulationPlannerService"),
    m_ModelServerServicePort("ModelServerService"),
    m_CollisionDetectionServicePort("CollisionDetectionService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ManipulationPlanner_OMPL::~ManipulationPlanner_OMPL()
{
}



RTC::ReturnCode_t ManipulationPlanner_OMPL::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ManipulationPlannerServicePort.registerProvider("ManipulationPlannerService", "Manipulation::ManipulationPlannerService", m_manipulationPlanner);
  
  // Set service consumers to Ports
  m_ModelServerServicePort.registerConsumer("Manipulation_ModelServerService", "Manipulation::ModelServerService", m_modelServer);
  m_CollisionDetectionServicePort.registerConsumer("Manipulation_CollisionDetectionService", "Manipulation::CollisionDetectionService", m_collisionDetection);
  
  // Set CORBA Service Ports
  addPort(m_ManipulationPlannerServicePort);
  addPort(m_ModelServerServicePort);
  addPort(m_CollisionDetectionServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("PlanningMethod", m_PlanningMethod, "(PRM,RRT,RRTConnect,RRTstar,LBTRRT,LaxyRRT,TRRT,pRRT,EST)");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onFinalize()
{
	  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ManipulationPlanner_OMPL::onActivated(RTC::UniqueId ec_id)
{
  //send config param to m_trajectoryPlanner
  m_manipulationPlanner.setPlanningMethod(m_PlanningMethod);
  m_manipulationPlanner.setComp(this);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ManipulationPlanner_OMPL::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void ManipulationPlanner_OMPL::callGetModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJointInfo){
	m_modelServer->getModelInfo(robotID, robotJointInfo);
/*
	m_robotJointInfo->jointInfoSeq.length(7);
	m_robotJointInfo->jointInfoSeq[0].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[0].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[0].linkLength=0;
	m_robotJointInfo->jointInfoSeq[0].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[0].minAngle = -2.792;
	m_robotJointInfo->jointInfoSeq[0].maxAngle = 2.792;

	m_robotJointInfo->jointInfoSeq[1].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[1].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[1].linkLength=0;
	m_robotJointInfo->jointInfoSeq[1].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[1].minAngle = -1.134;
	m_robotJointInfo->jointInfoSeq[1].maxAngle = 2.268;

	m_robotJointInfo->jointInfoSeq[2].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[2].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[2].linkLength=0;
	m_robotJointInfo->jointInfoSeq[2].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[2].minAngle = -0.1745;
	m_robotJointInfo->jointInfoSeq[2].maxAngle = 2.617;

	m_robotJointInfo->jointInfoSeq[3].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[3].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[3].linkLength=0;
	m_robotJointInfo->jointInfoSeq[3].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[3].minAngle = -2.792;
	m_robotJointInfo->jointInfoSeq[3].maxAngle = 2.792;

	m_robotJointInfo->jointInfoSeq[4].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[4].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[4].linkLength=0;
	m_robotJointInfo->jointInfoSeq[4].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[4].minAngle = -1.658;
	m_robotJointInfo->jointInfoSeq[4].maxAngle = 2.356;

	m_robotJointInfo->jointInfoSeq[5].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[5].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[5].linkLength=0;
	m_robotJointInfo->jointInfoSeq[5].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[5].minAngle = -2.967;
	m_robotJointInfo->jointInfoSeq[5].maxAngle = 2.967;

	//gripper
	m_robotJointInfo->jointInfoSeq[6].jointAngle=0;
	m_robotJointInfo->jointInfoSeq[6].jointDistance=0;
	m_robotJointInfo->jointInfoSeq[6].linkLength=0;
	m_robotJointInfo->jointInfoSeq[6].linkTwist=0;
	m_robotJointInfo->jointInfoSeq[6].minAngle = 0.0;
	m_robotJointInfo->jointInfoSeq[6].maxAngle = 90.0;
*/
}

bool ManipulationPlanner_OMPL::callIsCollide(const Manipulation::RobotIdentifier& manipInfo,const Manipulation::RobotJointInfo& jointSeq, Manipulation::CollisionInfo_out collision){
	for(int i=0;i<7;i++){
		//std::cout <<""<<std::endl;
		//std::cout <<"J"<<i<<" is: "<<jointSeq.jointInfoSeq[i].jointAngle<<std::endl;
	}
	return m_collisionDetection->isCollide(manipInfo, jointSeq, collision);
}

extern "C"
{
 
  void ManipulationPlanner_OMPLInit(RTC::Manager* manager)
  {
    coil::Properties profile(manipulationplanner_ompl_spec);
    manager->registerFactory(profile,
                             RTC::Create<ManipulationPlanner_OMPL>,
                             RTC::Delete<ManipulationPlanner_OMPL>);
  }
  
};


