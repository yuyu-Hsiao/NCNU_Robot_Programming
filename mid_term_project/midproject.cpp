#include "myProject.hpp"


using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  myRobot *robot = new myRobot();
  double pos[2];
  
  while (robot->step(TIME_STEP) != -1) {
	if(robot->State <= S8){
      robot->getPosition(pos);
      robot->SquareTransition(pos);
    }
    else{
      robot->readDistanceSensors();
      robot->transition();
    }   
  };

  delete robot;
  return 0;
}