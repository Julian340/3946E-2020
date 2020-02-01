#include "vex.h"
#include "robot-config.h"

float rotatorVal = 0;

void setVal (float val){
  rotatorVal = val;
}

float getVal(void){
  return rotatorVal;
}

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
void piDrive (float distance, float max){
  //sets the constants
  float kP = .3;
  float kI = 0.01;
  //calculates target by adding the distance to the current value of the encoder. this should account for the encoder not fully resetting
  float target = (distance) + lDrive.rotation(vex::rotationUnits::deg);
  //averages values of two encoders
  float average = (lDrive.rotation(vex::rotationUnits::deg) + rDrive.rotation(vex::rotationUnits::deg))/2; 
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
    if(pTerm + iTerm > max){
      setDrive(max);
    }
    else{
      setDrive(pTerm + iTerm);
    }     
  }
  setDrive(0);
}

//turns clockwise (positive) or counter clockwise (negative) a given amount of degrees
  //inputs: target -- how many degrees to turn
  //        max -- max speed to drive at
  //outputs: none
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

void deploy ( void ){
  //starts rollers
  setRollers(-100);

  //moves forward to help flip out rollers
  lDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct, false);
  rDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct);
    
  //moves back
  setDrive(-70);
 
  vex::task::sleep(300);

  lDrive.stop(coast);
  rDrive.stop(coast);

  //raises and lowers the lift to flip out rollers
  rLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  vex::task::sleep(200);
  rLift.stop(coast);
  lLift.stop(coast);
}

//scores 5 cubes into the nonprotected zone
void stack (int side){
  //deploys the robot to flip out
  deploy();
  //sleeps to give the robot time to fold
  vex::task::sleep(800);
  //start the rollers
  setRollers(80);
  //drive forward to pick up cubes
  piDrive(1850, 40);
  vex::task::sleep(10);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  //drive back
  lDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);          
  vex::task::sleep(100);
  //turns to face the goal
  if(side == 1){
    pTurn(150, 100);
  }
  else{
    pTurn(-145, 100);
  }
  vex::task::sleep(200);
  //drive to goal -- uses time instead in case we hit the bump
  setDrive(70);
  vex::task::sleep(2000);
  lDrive.stop(coast);
  rDrive.stop(coast);
  //place down stack
  rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
  setRollers(-10);
  vex::task::sleep(2800);
  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
  setRollers(-20);
  vex::task::sleep(200);
  //back up
  lDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
  //move tray down
  rotator.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  vex::task::sleep(800);
  rotator.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);      
  //stop rollers
  setRollers(0);
}


//auton that picks up a stack and then crosses the field to score
  //scores 5-7 cubes
void bigZone(int side){
  //deploys the robot to let it flip out
  deploy();
  //allows time for the tray to flip out
  vex::task::sleep(800);
  //start the rollers
  setRollers(80);
  //drive forward to pick up cubes
  piDrive(1900, 50);
  vex::task::sleep(10);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  //drive forward a little more
  piDrive(600, 100);
  vex::task::sleep(10);
  //turns to the bigzone
  if(side == 1){
    pTurn(-120, 100);
  }
  else{
    pTurn(125, 100);
  }
  //starts rollers for the field cross
  setRollers(35);
  //drives based on time to deal with the bump
  setDrive(100);
  vex::task::sleep(2500);
  //starts rotating out to stack
  rotator.spin(vex::directionType::fwd, 30, vex::velocityUnits::pct);
  vex::task::sleep(2050);
  setRollers(-20);
  setDrive(0);
  //turns to adjust after crossing
  if(side == 1){
    pTurn(-45, 40);
  }
  else{
    pTurn(55, 40);
  }
  vex::task::sleep(600);
  //stops the rotator and backs out 
  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  vex::task::sleep(500);
  setDrive(-40);
  vex::task::sleep(500);
  setDrive(0);
  setRollers(0);
}

//scores 3 cubes in the protected zone
  //a safer bet than bigZone, but less points
void smallZone(int side){
  //deploys the robot to let it flip out
  deploy();
  //allows time for the tray to flip out
  vex::task::sleep(800);
  //starts the rollers
  setRollers(80);
  //drive forward to pick up one cube
  piDrive(1000, 60);
  vex::task::sleep(10);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  //drives back
  piDrive(-400, 100);
  //turns to next cube
  if(side == 1){
    pTurn(-45, 100);
  }
  else{
    pTurn(45, 100);
  }
  //starts rollers to intake
  setRollers(80);
  //drives forward (based on time) in case cube gets in the way
  driveTime(1200, 70);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  driveTime(1100, 70);
  //drives forward (based on time) incase cube gets in the way
  if(side == 1){
    pTurn(-150, 100);
  }
  else{
    pTurn(150, 100);
  }
  //drives forward (based on time) incase cube gets in the way
  driveTime(1200, 50);
  //starts rollers halfway through the drive
  setRollers(80);
  driveTime(1250, 50);
  lRoller.stop(hold);
  rRoller.stop(hold);
  //turns to the goal
  if(side == 1){
    pTurn(-25, 100);
  }
  else{
    pTurn(25, 100);
  }
  //drive to goal based on time because of the bump
  setDrive(80);
  vex::task::sleep(900);
  lDrive.stop(coast);
  rDrive.stop(coast);
  //place down stack
  rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
  setRollers(-10);
  vex::task::sleep(2800);
  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
  setRollers(-20);
  vex::task::sleep(200);
  //back up
  lDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
  //move tray down
  rotator.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  vex::task::sleep(800);
  rotator.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);      
  //stop rollers
  setRollers(0);
  vex::task::sleep(10);
}
