#ifndef INCLUDED_COLLISION_CHECKER
#define INCLUDED_COLLISION_CHECKER

#include <omplapp/config.h>
#include <omplapp/apps/AppBase.h>
#include <omplapp/apps/SE3MultiRigidBodyPlanning.h>
//#include <omplapp/geometry/detail/FCLStateValidityChecker.h>

#include <iostream>
#include "p4-arm-helper.h"

namespace oa = ompl::app;
namespace ob = ompl::base;


class ArmMeshCollisionChecker{
public:
	ArmMeshCollisionChecker(int robotNum);
	~ArmMeshCollisionChecker();
    bool isNotCollided(std::vector<TVector> axesPos);
	void setMeshData(std::vector<std::string> robotsMeshPath, std::string envMeshPath);

	int jointNum = 6;
	//joint axis offset
	double xOffset[6] = {0,0,0,0,0,0};
	double yOffset[6] = {0,0,0,0,0,0};
	double zOffset[6] = {0.145,0.105,0.25,0.13,0.1,0105};

private:
	//oa::RigidBodyGeometry rg(oa::Motion_3D, oa::FCL);
	oa::SE3MultiRigidBodyPlanning* smbp;

public:
	void debug_setRobotMesh();
};


/*
class CollisionChecker{
public:
	CollisionChecker();
	~CollisionChecker();
	virtual bool isNotCollided()=0;
};

class MeshCollisionChecker:CollisionChecker{
public:
	MeshCollisionChecker(int robotNum);
	~MeshCollisionChecker();

	void setMeshData(std::vector<std::string> robotsMeshPath, std::string envMeshPath);

private:
	//oa::RigidBodyGeometry rg(oa::Motion_3D, oa::FCL);
	oa::SE3MultiRigidBodyPlanning* smbp;
};

class ArmMeshCollisionChecker:MeshCollisionChecker{
public:
	ArmMeshCollisionChecker(int robotNum):MeshCollisionChecker(robotNum){};
	~ArmMeshCollisionChecker();
    bool isNotCollided(std::vector<TVector> axesPos);

	int jointNum = 6;
	//joint axis offset
	double xOffset[6] = {0,0,0,0,0,0};
	double yOffset[6] = {0,0,0,0,0,0};
	double zOffset[6] = {0.145,0.105,0.25,0.13,0.1,0105};

public:
	void debug_setRobotMesh();
};



*/

#endif
