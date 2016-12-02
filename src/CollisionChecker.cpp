#include "CollisionChecker.h"

ArmMeshCollisionChecker::ArmMeshCollisionChecker(){
}

ArmMeshCollisionChecker::~ArmMeshCollisionChecker(){
	delete m_S3MultiRBodyPlanner;
}

void ArmMeshCollisionChecker::setMeshData(std::vector<std::string> robotsMesh, std::string envMesh){
	m_jointNum = robotsMesh.size();
    m_S3MultiRBodyPlanner = new oa::SE3MultiRigidBodyPlanning(m_jointNum);

    m_S3MultiRBodyPlanner->setEnvironmentMesh(envMesh.c_str());
    m_S3MultiRBodyPlanner->setRobotMesh(robotsMesh[0].c_str());

    if(robotsMesh.size()>1){
    	for(unsigned int i=1; i<robotsMesh.size(); i++){
    		m_S3MultiRBodyPlanner->addRobotMesh(robotsMesh[i].c_str());
    		}
    }
}

void ArmMeshCollisionChecker::debug_setRobotMesh(){
	std::vector<std::string> robot_fname;
	std::string env_fname;
	robot_fname[0] = "/home/ogata/Models/orochi_dae/J1.dae";
	robot_fname[1] = "/home/ogata/Models/orochi_dae/J2.dae";
	robot_fname[2] = "/home/ogata/Models/orochi_dae/J3.dae";
	robot_fname[3] = "/home/ogata/Models/orochi_dae/J4.dae";
	robot_fname[4] = "/home/ogata/Models/orochi_dae/J5.dae";
	robot_fname[5] = "/home/ogata/Models/orochi_dae/J6.dae";
    env_fname = "/home/ogata/Models/bookshelf2.dae";

    m_S3MultiRBodyPlanner->setEnvironmentMesh(env_fname.c_str());
    m_S3MultiRBodyPlanner->setRobotMesh(robot_fname[0].c_str());
	for(unsigned int i=1; i<robot_fname.size(); i++){
		m_S3MultiRBodyPlanner->addRobotMesh(robot_fname[i].c_str());
	}

}


bool ArmMeshCollisionChecker::isNotCollided(std::vector<TVector> axesPos){
	//軸の三次元座標の値を各メッシュの位置に充てる
	ob::ScopedState<ob::CompoundStateSpace> SE3State(m_S3MultiRBodyPlanner->getStateSpace());

	for (int i = 0; i < m_jointNum; i++){
		  ob::SE3StateSpace::StateType* link = SE3State.get()->as<ob::SE3StateSpace::StateType>(i);
	    link->setXYZ(axesPos[i][0]-m_xOffset[i], axesPos[i][1]-m_yOffset[i], axesPos[i][2]-m_zOffset[i]);
	    link->rotation().setIdentity();
	    //SE3State->as<ob::SE3StateSpace::StateType>(i)->setXYZ(axesPos[i][0]-xOffset[i], axesPos[i][1]-yOffset[i], axesPos[i][2]-zOffset[i]);
	}

	m_S3MultiRBodyPlanner->inferEnvironmentBounds();

	//get collision checker
	ob::StateValidityCheckerPtr svc = m_S3MultiRBodyPlanner->getStateValidityChecker();//allocStateValidityChecker(m_S3MultiRBodyPlanner.getSpaceInformation(), m_S3MultiRBodyPlanner.getGeometricStateExtractor(), true);
	//oa::FCLStateValidityChecker<oa::Motion_3D> svc(m_S3MultiRBodyPlanner->getSpaceInformation(),m_S3MultiRBodyPlanner->getGeometrySpecification(),m_S3MultiRBodyPlanner->getGeometricStateExtractor(), true);

	assert(m_S3MultiRBodyPlanner->isSelfCollisionEnabled());
	for(int i =0; i<m_jointNum; i++){
		  if(!svc->isValid(SE3State->as<ob::State>(i))){
	        return false;
	    }
	}
    return true;
}
