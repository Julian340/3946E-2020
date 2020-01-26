// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftDrive            motor         1               
// rightDrive           motor         17              
// rotator              motor         12              
// rollerLeft           motor         6               
// rollerRight          motor         2               
// rightLift            motor         16              
// leftLift             motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

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
}

void piDrive(float, float);
void deploy();
void stack(int);
void turnRight(float);
void turnLeft(float);
void bigZone(int);
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  //hello, please put 
  //1 == red 
  //-1 == blue
  stack(1);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
  // User control code here, inside the loop
  while (1) {
    //drive control
    leftDrive.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct); //(Axis3+Axis4)/2
    rightDrive.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);//(Axis3-Axis4)/2        
        
      
    //rotator
    if(Controller1.ButtonA.pressing()){
     rotator.spin(vex::directionType::fwd, 20, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonB.pressing()){
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
    else if (Controller1.ButtonR2.pressing()){
     rollerLeft.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
     rollerRight.spin(vex::directionType::rev, 90, vex::velocityUnits::pct);
    }
    else{
     rollerLeft.stop(hold);
     rollerRight.stop(hold);
    }
        
    //lift
    if(Controller1.ButtonL2.pressing()){    
      rightLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
      leftLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
      rotator.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);    
    }
    else if(Controller1.ButtonL1.pressing()){
     rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
     rightLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);  
     leftLift.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }
    else {  
      rightLift.stop(vex::brakeType::hold);
      leftLift.stop(vex::brakeType::hold);    
    } 
    
    if(Controller1.ButtonUp.pressing()){
      //deploys for drivver
      deploy();          
      
    }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  
  }
}

//
// Main will set up the competition functions and callbacks.
//
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