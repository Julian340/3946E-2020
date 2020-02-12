#include "vex.h"

//Creates a competition object that allows access to Competition methods.
vex::competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void piDrive(float, float);
void deploy();
void stack(int);
void turnRight(float);
void turnLeft(float);
void bigZone(int);
void pTurn(float, int);
void pStrafe(float, int);
void smallZone(int);
void small(int);
void prog();

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  vexcodeInit();

  //calibrate the inertial sensor
  gyroscope.calibrate();
  while(gyroscope.isCalibrating()){
   vex::task::sleep(10);
  }
  rotator.resetRotation();


}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/


void autonomous( void ) {
  
  //hello, please put 
  //1 == red 
  //0 == blue
  prog();

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/


void usercontrol( void ) {
 
  //rotator.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
  //vex::ta,,sk::sleep(1000);
  //rotator.resetRotation();

  float left;
  float right;
  float strafe;


  float liftHeight;
  int rotatorTarget = 1300;
  int rotatorMax = 60;
  int rotatorMin = 10;
  
  lLift.resetRotation();
  rLift.resetRotation();
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);          
  Controller1.Screen.print("Main Controller");

  Controller2.Screen.clearScreen();
  Controller2.Screen.setCursor(1,1);          
  Controller2.Screen.print("Partner");
    

  // User control code here, inside the loop
  while (1) {
    //drive control
    left =  Controller1.Axis3.value() + .7 * Controller1.Axis1.value();
    right =  Controller1.Axis3.value() - .7 * Controller1.Axis1.value();
    strafe = Controller1.Axis4.value();

    lDrive.spin(vex::directionType::fwd, left, vex::velocityUnits::pct); //(Axis3+Axis4)/2
    rDrive.spin(vex::directionType::fwd, right  , vex::velocityUnits::pct);//(Axis3-Axis4)/2         
    mDrive.spin(vex::directionType::fwd, strafe, vex::velocityUnits::pct);    
      
    //rotator
    if(Controller2.ButtonA.pressing()){
     //drives the rotator proportional to the distance from stacking
     float error = rotatorTarget - rotator.rotation(vex::rotationUnits::deg);
     float speed = (rotatorMax + rotatorMin) * (error/rotatorTarget) + rotatorMin;
     rotator.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    }
    else if(Controller2.ButtonB.pressing() && rotator.rotation(vex::rotationUnits::deg) > -10){
     rotator.spin(vex::directionType::rev, 50, vex::velocityUnits::pct); 
    }
    else if(Controller2.ButtonX.pressing()){
     rotator.spin(vex::directionType::rev, 50, vex::velocityUnits::pct); 
    }
    else if(Controller2.ButtonY.pressing()){
     rotator.resetRotation(); 
    }
    else{
     rotator.stop(hold); 
    }
        
    //rollers
    if(Controller1.ButtonR1.pressing()){
      lRoller.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
      rRoller.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
    }
    else if (Controller2.ButtonR1.pressing()){
      lRoller.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
      rRoller.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing()){
     lRoller.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
     rRoller.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
    }
    else if (Controller2.ButtonR2.pressing()){
     lRoller.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
     rRoller.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }
    else{
     lRoller.stop(hold);
     rRoller.stop(hold);
    }
        

    //lift
    liftHeight = (rLift.rotation(vex::rotationUnits::deg) + lLift.rotation(vex::rotationUnits::deg))/2.0;
    
    if(Controller2.ButtonL2.pressing()){    
      rLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
      lLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
    }
    else if(Controller2.ButtonL1.pressing()){
     rLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);  
     lLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }
    
    else if (liftHeight > -50) {  
      rLift.stop(vex::brakeType::coast);
      lLift.stop(vex::brakeType::coast);    
    } 
    else {
      rLift.stop(vex::brakeType::brake);
      lLift.stop(vex::brakeType::brake);    
    }

    if(Controller1.ButtonUp.pressing()){
      //deploys for drivver
      deploy();
         
      
    }    
  
    vex::task::sleep(5); //Sleep the task for a short amount of time to prevent wasted resources. 
  
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}