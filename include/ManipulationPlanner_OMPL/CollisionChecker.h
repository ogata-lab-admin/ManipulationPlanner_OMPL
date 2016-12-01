#include <omplapp/config.h>
#include <omplapp/apps/AppBase.h>
#include <omplapp/apps/SE3MultiRigidBodyPlanning.h>

#include <iostream>

namespace oa = ompl::app;

class CollisionChecker{
public:
	virtual bool isNotCollide();
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
    bool isNotCollide(std::vector<TVector> axesPos);

	int jointNum = 6;
	//joint axis offset
	double xOffset[6] = {0,0,0,0,0,0};
	double yOffset[6] = {0,0,0,0,0,0};
	double zOffset[6] = {0.145,0.105,0.25,0.13,0.1,0105};

public:
	void debug_setRobotMesh();
};


/*
class JointStateSampler{
public:
	JointStateSampler(){debug_setArm();}
    //void setArm(std::vector<TLink>)

private:
    TVector ArmBase = V3(-100.0, 445.0, 0.0);

    //Arm(TLink(Directional vector, End-point vector))
    std::vector<TLink> Arm;

    void debug_setArm(){
    	//                     r,p,y      x y z
    	Arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
    	Arm.push_back(TLink(V3(0,1,0), V3(0,0,250.0)));
    	Arm.push_back(TLink(V3(0,1,0), V3(0,0,130.0)));
    	Arm.push_back(TLink(V3(0,0,1), V3(0,0,100.0)));
    	Arm.push_back(TLink(V3(0,1,0), V3(0,0,105.0)));
    	Arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
    }

};
*/
