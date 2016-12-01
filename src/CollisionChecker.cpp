#include "CollisionChecker.h"

MeshCollisionChecker::MeshCollisionChecker(int robotNum){
    smbp = new oa::SE3MultiRigidBodyPlanning(jNum);
}

MeshCollisionChecker::~MeshCollisionChecker(){
	delete smbp;
}
void MeshCollisionChecker::setMeshData(std::vector<std::string> robotsMeshPath, std::string envMeshPath){
    smbp->setEnvironmentMesh(envMeshPath.c_str());
    smbp->setRobotMesh(robotsMeshPath[0].c_str());

    if(robots.size()>1){
    	for(int i=1; i<robotsMeshPath.size(); i++){
    		smbp->addRobotMesh(robotsMeshPath[i].c_str());
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

    smbp->setEnvironmentMesh(env_fname.c_str());
    smbp->setRobotMesh(robot_fname[0].c_str());
	for(int i=1; i<robot_fname.size(); i++){
		smbp->addRobotMesh(robot_fname[i].c_str());
	}

}


bool ArmMeshCollisionChecker::isNotCollide(std::vector<TVector> axesPos){
	//軸の三次元座標の値を各メッシュの位置に充てる
	/*
	 ob::StateSpacePtr space(new ob::SE3StateSpace());
	 ob::RealVectorBounds bounds(3);
	 bounds.setLow(-300);
	 bounds.setHigh(300);
	 //smbp.setBoundsAddition(2.0);
	 space->as<ob::SE3StateSpace>()->setBounds(bounds);
	*/

	//ob::StateSpacePtr space(smbp.getStateSpace());
	ob::ScopedState<ob::CompoundStateSpace> SE3State(smbp.getStateSpace());//origin of mesh data
	//ob::ScopedState<ob::CompoundStateSpace> SE3State(space);//origin of mesh data
	for (int i = 0; i < jointNum; i++){
		  ob::SE3StateSpace::StateType* link = SE3State.get()->as<ob::SE3StateSpace::StateType>(i);
	    link->setXYZ(axesPos[i][0]-xOffset[i], axesPos[i][1]-yOffset[i], axesPos[i][2]-zOffset[i]);
	    link->rotation().setIdentity();
	    //SE3State->as<ob::SE3StateSpace::StateType>(i)->setXYZ(axesPos[i][0]-xOffset[i], axesPos[i][1]-yOffset[i], axesPos[i][2]-zOffset[i]);
	}
	smbp.inferEnvironmentBounds();
	//smbp.setup();
	//get collision checker
	//ob::StateValidityCheckerPtr svc = smbp.getStateValidityChecker();//allocStateValidityChecker(smbp.getSpaceInformation(), smbp.getGeometricStateExtractor(), true);
	oa::FCLStateValidityChecker<oa::Motion_3D> svc(smbp.getSpaceInformation(),smbp.getGeometrySpecification(),smbp.getGeometricStateExtractor(), true);

	assert(smbp.isSelfCollisionEnabled());
	for(int i =0; i<jointNum; i++){
		  //std::cout << SE3State->as<ob::State>(i) << std::endl;
		  if(!svc.isValid(SE3State->as<ob::State>(i))){
	        return false;
	    }
	}
    return true;
}
