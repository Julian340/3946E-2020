#include "vex.h"
#include "robot-config.h"

/*void setRollers(int speed){

}*/

void piDrive (float distance, float max){
  //sets the constants
  float kP = .3;
  float kI = 0.01;
  
  //calculates target by adding the distance to the current value of the encoder. this should account for the encoder not fully resetting
  float target = (distance) + leftDrive.rotation(vex::rotationUnits::deg);
  //averages values of two encoders
  float average = (leftDrive.rotation(vex::rotationUnits::deg) + rightDrive.rotation(vex::rotationUnits::deg))/2; 
  //counts total
  float totalError = 0;
  int counter = 0;

  while(counter < 3){ 
    //resets the average
    average = (leftDrive.rotation(vex::rotationUnits::deg) + rightDrive.rotation(vex::rotationUnits::deg))/2;
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
    leftDrive.spin(vex::directionType::fwd,max,vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,max,vex::velocityUnits::pct);
    }
    else{
      leftDrive.spin(vex::directionType::fwd,(pTerm + iTerm),vex::velocityUnits::pct);
      rightDrive.spin(vex::directionType::fwd,(pTerm + iTerm),vex::velocityUnits::pct);
    }  
    
    //helps with debugging.. prints the counter to screen. uncomment if you need to test.
    /*Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);          
    Controller1.Screen.print("%f", error);   
    vex::task::sleep(2);  */ 
    
        
    
  }
  leftDrive.spin(vex::directionType::fwd,0,vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::fwd,0,vex::velocityUnits::pct);

}

void pTurn (float target){

  //this is to ensure that even if the gyro isn't at 0, the target will be the correct distance away
  gyroscope.resetRotation();
  
  //sets the kp and the counter
  float kP = 1.2;  
  int counter = 0;

  while (counter < 3){

    //calculates error by subtracting the 
    float error = target - gyroscope.rotation();
    
    //checks to see if the error is within a specific threshold to increment the counter
    if(fabs(error) < 40){
      counter ++;
    }
    else counter = 0;


    //sets motor value equal to error * kp
    float speed = kP * error;
    leftDrive.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::rev,speed,vex::velocityUnits::pct);

 
    vex::task::sleep(2);

  }
  leftDrive.spin(vex::directionType::fwd,0,vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::rev,0,vex::velocityUnits::pct);


}

void deploy ( void ){
  //starts rollers
  rollerLeft.spin(vex::directionType::rev,100, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);

  //moves forward to help flip out rollers
  leftDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct, false);
  rightDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct);
    
  //moves back
  leftDrive.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);

  vex::task::sleep(400);

  leftDrive.stop(coast);
  rightDrive.stop(coast);


  //raises and lowers the lift to flip out rollers
  rightLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  leftLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  vex::task::sleep(200);
  rightLift.stop(coast);
  leftLift.stop(coast);
}

//turns counterclockwise a given amount of degrees
void turnLeft (double deg){
 rightDrive.rotateFor(deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct, false);
 leftDrive.rotateFor(-deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct);  
}

//turns clockwise a given amount of degrees
void turnRight(double deg){
  leftDrive.rotateFor(deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct, false);
  rightDrive.rotateFor(-deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct);

}

void stack (int side){
  deploy();
  vex::task::sleep(800);
  
  //start the rollers
  rollerLeft.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);

  //drive forward
  piDrive(1850, 40);
  vex::task::sleep(10);

  //stop rollers
  rollerLeft.stop(hold);
  rollerRight.stop(hold);

  //drive back
  leftDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rightDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);          
  vex::task::sleep(100);

  //turns to face the goal
  if(side == 1){
    pTurn(153);
  }
  else{
    pTurn(-150);
  }
  vex::task::sleep(200);

  //drive to goal -- uses time instead because we might not always reach same value
  leftDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);

  vex::task::sleep(2000);
  
  leftDrive.stop(coast);
  rightDrive.stop(coast);

  //place dwon stack
  rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
    
  rollerLeft.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);

  vex::task::sleep(2800);

  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
  
  rollerLeft.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);

  vex::task::sleep(200);

  //back up
  leftDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rightDrive.rotateFor(-400,vex::rotationUnits::deg,40,vex::velocityUnits::pct);

  //move tray down
  rotator.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  vex::task::sleep(800);
  rotator.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);      

  //stop rollers
  rollerLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);

}


//test auton for the non stack side
void bigZone(int side){
 
  deploy();
  vex::task::sleep(800);
  
  //start the rollers
  rollerLeft.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);

  //drive forward
  piDrive(1800, 50);
  vex::task::sleep(10);

  //stop rollers
  rollerLeft.stop(hold);
  rollerRight.stop(hold);

  piDrive(600, 100);
  vex::task::sleep(10);

  pTurn(-120);

  rollerLeft.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);

  leftDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

  

  vex::task::sleep(2500);

  rotator.spin(vex::directionType::fwd, 32, vex::velocityUnits::pct);

  vex::task::sleep(2100);
  
  leftDrive.stop(coast);
  rightDrive.stop(coast);

  vex::task::sleep(200);

  rollerLeft.spin(vex::directionType::rev, 15, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::rev, 15, vex::velocityUnits::pct);

  //piDrive(-400,40);


  rollerLeft.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
  rollerRight.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
}

