#include "MotionPlanner.h"

#include "p3-uav-helper.h"
#include "p4-arm-helper.h"
#include <boost/bind.hpp>
using namespace std;

// コンストラクタ
Planning::Planning(std::string fileName)
{
  initFromFile(fileName);
  SetArm();
  //PlannerSelector();
}

void Planning::SetArm(){
  // ArmBase: マニピュレータのベース位置．以下のように設定する：
  //ArmBase = V3(0.0,0.0,0.0);
  ArmBase = V3(-100.0, 445.0, 0.0);

  // Arm: マニピュレータオブジェクト．実質，ローカルフレームで定義された関節の方向ベクトルと，エンドポイント（端点）のベクトルから構成される構造体のベクトルである．以下のように，多リンク系を作る：
  total_len = 0.99 * (SizeZ - ArmBase(2));
  //                     r,p,y
  Arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,250.0)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,130.0)));
  Arm.push_back(TLink(V3(0,0,1), V3(0,0,100.0)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,105.0)));
  Arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
}


void Planning::initFromFile(std::string fileName)
{
  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> zBottom >> zTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];
  zMin = new double[numObstacles];
  zMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i] >> zMin[i] >> zMax[i];
  }

  Start = new double[num];
  Goal = new double[num];

  input >> Start[0] >> Start[1] >> Start[2] >> Start[3]>> Start[4]>> Start[5]
        >> Goal[0] >> Goal[1] >> Goal[2] >> Goal[3]>> Goal[4]>> Goal[5];

  input.close();

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop, zBottom, zTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i], zMin[i], zMax[i]);
  }

  printf("\nスタートとゴール    :Start[");
  for(int i=0;i<num;i++){
	  printf("%5.2lf, ", Start[i]);
  }
  printf("]\n                       End[");
  for(int i=0; i<num; i++){
	  printf("%5.2lf, ", Goal[i]);
  }
  printf("]\n");
}


bool Planning::isStateValid(const ob::State *state)
{
  const ob::RealVectorStateSpace::StateType *state_vec= state->as<ob::RealVectorStateSpace::StateType>();
  std::vector<double> angles(Arm.size());
  std::vector<TVector> result;

  for (size_t  i = 0; i < Arm.size(); ++i){
    // cout << (*state_vec)[i] << endl;
    angles[i] = (*state_vec)[i];
  }

  ForwardKinematics(Arm, angles, ArmBase, result);


  for (size_t i = 0; i < result.size()-1; ++i){
    if(/*mesh collision check func*/==false){
      // cout << "衝突してます" << endl;
      return false;
    }
  }
  return true;
}



/* Compute the forward kinematics of a manipulator ``linkes''
whose joint angles are specified by ``angles'',
and the base position is ``base''.
The result is stored into ``result'' that contains
the base position and every position of the end-points.

i.e. result.size()==linkes.size()+1 */
void Planning::ForwardKinematics(const std::vector<TLink> &linkes,
                                 const std::vector<double> &angles, const TVector &base,
                                 std::vector<TVector> &result)
{
  // assert(linkes.size() == angles.size());
  assert(base.size() == 3);
  result.resize(linkes.size() + 1);
  TMatrix R(3, 3), Rtmp(3, 3);
  R(0, 0) = R(1, 1) = R(2, 2) = 1.0;
  R(0, 1) = R(0, 2) = R(1, 0) = R(1, 2) = R(2, 0) = R(2, 1) = 0.0;
  result[0] = base;
  for (size_t i(1); i < result.size(); ++i) {
    result[i].resize(3);
    Rtmp = prod(R, RfromAxisAngle(linkes[i - 1].Axis, angles[i - 1]));
    R = Rtmp;
    result[i] = prod(R, linkes[i - 1].End) + result[i - 1];
  }
}



void Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(num));

  int count = 0;

  ob::RealVectorBounds bounds(num);
  for (int i = 0; i < num; ++i){
    bounds.setLow(i, -M_PI);
    bounds.setHigh(i, M_PI);
  }
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::RealVectorStateSpace> start(space);
  for (int i = 0; i < num; ++i){
    start->as<ob::RealVectorStateSpace::StateType>()->values[i] = Start[i];
  }

  // start->rotation().setIdentity();
  // start.random();

  cout << "start: ";
  start.print(cout);

  ob::ScopedState<ob::RealVectorStateSpace> goal(space);
  for (int i = 0; i < num; ++i){
    goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = Goal[i];
  }
  // goal->setXYZ(xGoal,yGoal,zGoal);
  // goal->rotation().setIdentity();
  // goal.random();
  cout << "goal: ";
  goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if (selector == 1) {
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 2) {
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 3) {
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 4) {
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 5) {
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 6) {
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 7) {
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 8) {
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 9) {
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  cout << "----------------" << endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(30);

  while (1) {
    // If we have a solution,
    if (solved) {
      // Print the solution path (that is not simplified yet) to a file
      std::ofstream ofs0("../plot/path0.dat");
      ss.getSolutionPath().printAsMatrix(ofs0);

      // Simplify the solution
      ss.simplifySolution();
      cout << "----------------" << endl;
      cout << "Found solution:" << endl;
      // Print the solution path to screen
      ss.getSolutionPath().print(cout);

      // Print the solution path to a file
      std::ofstream ofs("../plot/path.dat");
      ss.getSolutionPath().printAsMatrix(ofs);

        }
      }
      break;
    } else {
      cout << "No solution found" << endl;
        break;
      }
    }

  }
}
