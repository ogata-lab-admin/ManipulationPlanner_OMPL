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

Manipulation::ReturnValue*  ManipulationPlanner_OMPL::callGetModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJointInfo){
	return m_modelServer->getModelInfo(robotID, robotJointInfo);
}

Manipulation::ReturnValue*  ManipulationPlanner_OMPL::callIsCollide(const Manipulation::RobotIdentifier& robotID,const Manipulation::JointAngleSeq& jointSeq, Manipulation::CollisionPairSeq_out collision){
	return m_collisionDetection->isCollide(robotID, jointSeq, collision);
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


