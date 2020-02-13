#include "vex.h"
#include "robot-config.h"


//this function sets the rollers to a given speed
  //e.x. --> setRollers(50); would intake at a speed of 50
void setRollers(int speed){
  if(speed >= 0){
    lRoller.spin(vex::directionType::fwd,speed, vex::velocityUnits::pct);
    rRoller.spin(vex::directionType::fwd,speed, vex::velocityUnits::pct);
  }
  else{
    lRoller.spin(vex::directionType::rev,abs(speed), vex::velocityUnits::pct);
    rRoller.spin(vex::directionType::rev,abs(speed), vex::velocityUnits::pct);
  }
}

//this function sets the drive to a given speed
  //e.x. --> setDrive(50); would drive at a speed of 50
void setDrive(int speed){
  lDrive.spin(vex::directionType::fwd,speed, vex::velocityUnits::pct);
  rDrive.spin(vex::directionType::fwd,speed, vex::velocityUnits::pct);
}

void driveTime(int time_, int speed){

  setDrive(speed);
  vex::task::sleep(time_);
  setDrive(0);

}

//drives forward a given amount of ticks
  //inputs: distance -- how many ticks to drive forward
  //        max -- max speed to drive at
  //outputs: none
  //e.x. --> piDrive(400, 70); would drive forward 400 ticks, capping at a max speed of 70
void piDrive (float distance, float max){
  //sets the constants
  float kP = .3;
  float kI = 0.01;
  //averages values of two encoders
  float average = (lDrive.rotation(vex::rotationUnits::deg) + rDrive.rotation(vex::rotationUnits::deg))/2; 
  //calculates target by adding the distance to the current value of the encoder. this should account for the encoder not fully resetting
  float target = (distance) + average;
  //counts total
  float totalError = 0;
  int counter = 0;
  while(counter < 3){ 
    //resets the average
    average = (lDrive.rotation(vex::rotationUnits::deg) + rDrive.rotation(vex::rotationUnits::deg))/2;
    //get error by subtracting the average from target
    float error = target - average;
    //adds to total error to account for integral
    totalError += error;
    //multiplies p and i terms by constant
    float pTerm = error * kP;
    float iTerm = totalError * kI;
    //creates a counter to ensure that the drive is in the given range for a set amount of time
    //the loop will only end if the average is within 40 ticks of the target for 3 cycles
    if(fabs(error) < 40){
      counter ++;
    }
    else counter = 0;
    float speed = pTerm + iTerm;
    if(fabs(speed) > fabs(max)){
      float signedMax = max * (speed/fabs(speed));
      setDrive(signedMax);
    }
    else{
      setDrive(speed);
    }     
  }
  setDrive(0);
}

//turns clockwise (positive) or counter clockwise (negative) a given amount of degrees
  //inputs: target -- how many degrees to turn
  //        max -- max speed to drive at
  //outputs: none
  //e.x. --> pTurn(90, 70); would turn forward 90 degrees, capping at a max speed of 70
  
  //NOTE THAT 90 =/= 90 ACTUAL DEGREES, YOU NEED TO TUNE FOR A BETTER VALUE
void pTurn (float target, int max){
  //this is to ensure that even if the gyro isn't at 0, the target will be the correct distance away
  gyroscope.resetRotation();
  //sets the kp and the counter
  float kP = 1.2;  
  int counter = 0;
  while (counter < 3){
    //calculates error by subtracting the gyro value from the target
    float error = target - gyroscope.rotation();
    //checks to see if the error is within a specific threshold to increment the counter
    if(fabs(error) < 40){
      counter ++;
    }
    else counter = 0;
    //sets motor value equal to error * kp
    float speed = kP * error;
    if (fabs(speed) > abs(max)){
      //assigns the sign from the speed to the max value to ensure it turns the same way
      float signedMax = max * (speed/fabs(speed));
      lDrive.spin(vex::directionType::fwd,signedMax,vex::velocityUnits::pct);
      rDrive.spin(vex::directionType::rev,signedMax,vex::velocityUnits::pct);
    }
    else{
      lDrive.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
      rDrive.spin(vex::directionType::rev,speed,vex::velocityUnits::pct);
    }
    //helps with debugging.. prints the counter to screen. uncomment if you need to test.
    /*Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);          
    Controller1.Screen.print("%f", error);*/  
    vex::task::sleep(2);
  }
  setDrive(0);
}


//deploys out the robot
void deploy ( void ){
  //starts rollers
  setRollers(-100);

  //moves forward to help flip out rollers
  lDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct, false);
  rDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct);
    
  //moves back
  setDrive(-60);
 
  vex::task::sleep(400);

  lDrive.stop(coast);
  rDrive.stop(coast);

  //raises and lowers the lift to flip out rollers
  rLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  vex::task::sleep(400);
   
  rLift.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
  vex::task::sleep(400);
  rLift.stop(coast);
  lLift.stop(coast);
}
