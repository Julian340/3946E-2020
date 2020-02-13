#include "vex.h"

void deploy();
void setRollers(int speed);
void setDrive(int speed);
void piDrive(float driveDistance, float maxspeed);
void pTurn(float degrees, float maxspeed);
void driveTime(int milliseconds, float maxspeed);


//scores 5 cubes into the nonprotected zone
void stack (int side){

  //deploys the robot to flip out
  deploy();
  //sleeps to give the robot time to fold
  vex::task::sleep(800);
  //start the rollers
  setRollers(90);
  //drive forward to pick up cubes
  piDrive(1850, 55);
  vex::task::sleep(10);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  //drive back
  lDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);          
  vex::task::sleep(100);

  //turns to face the goal
  //red used to be 156
  if(side == 1){
    //THIS IS FOR RED SIDE
    pTurn(170, 100);
  }
  else{
    //THIS IF FOR BLUE SIDE
    pTurn(-170, 100);
  }
  vex::task::sleep(200);
  //drive to goal -- uses time instead in case we hit the bump
  setDrive(75);
  vex::task::sleep(1900);
  lDrive.stop(coast);
  rDrive.stop(coast);
  //place down stack
  rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
  setRollers(-6);
  vex::task::sleep(2800);
  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
  setRollers(-30);
  vex::task::sleep(200);
  driveTime(500, 15);
  //back up
  lDrive.rotateFor(-500,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-500,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
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
    //THIS IS FOR RED
    pTurn(-120, 100);
  }
  else{
    //THIS IS FOR BLUE
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
  driveTime(500, 10);
  vex::task::sleep(500);
  driveTime(400, -50);
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
    //THIS IS FOR RED SIDE
    pTurn(-25, 100);
  }
  else{
    //THIS IS FOR BLUE SIDE
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

void small(int side ){
  //deploys and pushes cube
  deploy();
  setRollers(0);
  driveTime(800, 100);
  driveTime(800, -100);
}

void prog(){

  //~~~~~~~~~~~~~FIRST ROW OF CUBES~~~~~~~~~~~~~~~~~~//

  //deploys the robot to flip out
  deploy();
  //sleeps to give the robot time to fold
  vex::task::sleep(800);
  //start the rollers
  setRollers(75);
  //drive forward to pick up cubes
  piDrive(2100, 55);
  vex::task::sleep(10);
  //stop rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  //drive back to hit the wall
  driveTime(1800, -100);
  driveTime(200, 50);
  //strafe along the wall to the left
  mDrive.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  vex::task::sleep(1400);
  mDrive.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
  //drive back again to realign
  driveTime(400, -70);
  vex::task::sleep(300);

  //~~~~~~~~~~~~~SECOND ROW OF CUBES~~~~~~~~~~~~~~~~~~//

  piDrive(500, 100);
  //start the rollers
  setRollers(90);
  //drive forward to pick up cubes
  piDrive(1900, 55);
  vex::task::sleep(10);
  //stop rollers
  setRollers(5);
  //drive back
  lDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);          
  vex::task::sleep(100);
  
  //~~~~~~~~~~~~~STACKING THE CUBES~~~~~~~~~~~~~~~~~~//

  //turns to face the goal
  pTurn(-180, 100);
  vex::task::sleep(200);
  //drive to goal -- uses time instead in case we hit the bump
  setDrive(85);
  vex::task::sleep(2300);
  lDrive.stop(coast);
  rDrive.stop(coast);
  //place down stack
  rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
  setRollers(-6);
  vex::task::sleep(2800);
  rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
  //setRollers(-30);
  vex::task::sleep(200);
  driveTime(500, 15);
  //back up
  setRollers(-40);
  lDrive.rotateFor(-500,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
  rDrive.rotateFor(-500,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
  //move tray down
  rotator.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  vex::task::sleep(800);
  rotator.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);      
  //stop rollers
  setRollers(0);

  //~~~~~~~~~~~~~FIRST TOWER~~~~~~~~~~~~~~~~~~//
  
  //turning to go to new tower
  pTurn(-150, 100);
  //allign back with wall
  driveTime(1400, -70);
  //starts driving to the tower
  piDrive(1800, 100);
  //then starts rollers while driving
  setRollers(70);
  piDrive(750, 100);
  vex::task::sleep(400);
  //drives back after picking up cube
  piDrive(-400, 100);
  //extakes the cube a little bit to align
  setRollers(-60);
  vex::task::sleep(500);
  //stops rollers
  lRoller.stop(hold);
  rRoller.stop(hold);
  vex::task::sleep(500);
  //lifts up
  rLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  vex::task::sleep(1600);
  rLift.stop(hold);
  lLift.stop(hold);
  //drives forward
  piDrive(300, 100);
  //extakes cube into towers
  setRollers(-50);
  vex::task::sleep(700);

  //~~~~~~~~~~~~~SECOND TOWER~~~~~~~~~~~~~~~~~~//

  //backs up
  driveTime(650, -100);
  //stops rollers
  setRollers(0);
  //lifts down
  rLift.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
  vex::task::sleep(1200);
  rLift.stop(hold);
  lLift.stop(hold);
  //turns to the wall
  pTurn(-110,100);
  //drivees back against the wall
  driveTime(2000, -100);
  //drives forward towards cubes
  piDrive(1800, 100);
  //then starts rollers while continuing to drive
  setRollers(50);
  piDrive(800, 100);
  vex::task::sleep(400);
  //drives back after intaking cube
  piDrive(-400, 100);
  //extakes a little bit for better grip
  setRollers(-40);
  vex::task::sleep(550);
  lRoller.stop(hold);
  rRoller.stop(hold);
  vex::task::sleep(500);
  //lifts up to the tower
  rLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  lLift.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  vex::task::sleep(1400);
  rLift.stop(hold);
  lLift.stop(hold);
  //drives forward to stack the tower
  piDrive(250, 100);
  //extakes the rollers to score tower
  setRollers(-50);
  vex::task::sleep(700);
  //drives backwards
  driveTime(700, -100);
}