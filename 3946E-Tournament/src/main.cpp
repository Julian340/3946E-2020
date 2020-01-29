#include "vex.h"

//Creates a competition object that allows access to Competition methods.
vex::competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  vexcodeInit();

  //calibrate the inertial sensor
  gyroscope.calibrate();
  while(gyroscope.isCalibrating()){
    vex::task::sleep(10);
  }

}

void piDrive(float, float);
void deploy();
void stack(int);
void turnRight(float);
void turnLeft(float);
void bigZone(int);
void pTurn(float);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  //hello, please put 
  //1 == red 
  //-1 == blue
  bigZone(1);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/


void usercontrol( void ) {
 
  float left;
  float right;
  float liftHeight;
  
  rotator.resetRotation();
  leftLift.resetRotation();
  rightLift.resetRotation();
    

  // User control code here, inside the loop
  while (1) {
    //drive control
    left =  Controller1.Axis3.value() + Controller1.Axis1.value();
    right =  Controller1.Axis3.value() - Controller1.Axis1.value();

    leftDrive.spin(vex::directionType::fwd, left, vex::velocityUnits::pct); //(Axis3+Axis4)/2
    rightDrive.spin(vex::directionType::fwd, right  , vex::velocityUnits::pct);//(Axis3-Axis4)/2        
        
      
    //rotator
    if(Controller2.ButtonA.pressing()){
     rotator.spin(vex::directionType::fwd, 20, vex::velocityUnits::pct);
    }
    else if(Controller2.ButtonB.pressing() && rotator.rotation(vex::rotationUnits::deg) > -10){
     rotator.spin(vex::directionType::rev, 50, vex::velocityUnits::pct); 
    }
    else{
     rotator.stop(hold); 
    }
        
    //rollers
    if(Controller1.ButtonR1.pressing()){
      rollerLeft.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
      rollerRight.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
    }
    else if (Controller2.ButtonR1.pressing()){
      rollerLeft.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
      rollerRight.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing()){
     rollerLeft.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
     rollerRight.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
    }
    else if (Controller2.ButtonR2.pressing()){
     rollerLeft.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
     rollerRight.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }
    else{
     rollerLeft.stop(hold);
     rollerRight.stop(hold);
    }
        

    //lift
    liftHeight = (rightLift.rotation(vex::rotationUnits::deg) + leftLift.rotation(vex::rotationUnits::deg))/2.0;
    
    if(Controller2.ButtonL2.pressing()){    
      rightLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
      leftLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
      if(rotator.rotation(vex::rotationUnits::deg) > -10){
        rotator.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);    
      }
    }
    else if(Controller2.ButtonL1.pressing()){
     rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
     rightLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);  
     leftLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }
    
    else if (liftHeight > -50) {  
      rightLift.stop(vex::brakeType::coast);
      leftLift.stop(vex::brakeType::coast);    
    } 
    else {
      rightLift.stop(vex::brakeType::brake);
      leftLift.stop(vex::brakeType::brake);    
    }

    if(Controller1.ButtonUp.pressing()){
      //deploys for drivver
      deploy();
         
      
    }

      
    
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);          
    Controller1.Screen.print("Main Controller");

    Controller2.Screen.clearScreen();
    Controller2.Screen.setCursor(1,1);          
    Controller2.Screen.print("Partner");
    
  
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