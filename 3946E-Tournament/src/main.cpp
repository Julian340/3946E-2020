
// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

#include "robot-config.h"
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
vex::competition Competition;

vex::motor leftDrive  (vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive (vex::PORT17, vex::gearSetting::ratio18_1, true);
//vex::motor frontRightMotor (vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor rotator(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor rollerLeft(vex::PORT6, vex::gearSetting::ratio18_1, true);
vex::motor rollerRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor rightLift(vex::PORT16, vex::gearSetting::ratio18_1, false);
vex::motor leftLift(vex::PORT7, vex::gearSetting::ratio18_1, true);

vex::controller Controller1 = vex::controller();
//vex::controller Controller2 = vex::controller();

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
}

void deploy ( void ){

  leftDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(200,vex::rotationUnits::deg,30,vex::velocityUnits::pct);
    
    leftDrive.rotateFor(-200,vex::rotationUnits::deg,30,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(-200,vex::rotationUnits::deg,30,vex::velocityUnits::pct);

    leftLift.rotateTo(300, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);
    rightLift.rotateTo(300, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);

    //deploy rollers
    rollerLeft.rotateFor(-700,vex::rotationUnits::deg,100,vex::velocityUnits::pct, false);
    rollerRight.rotateFor(-700,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    //rightLift.stop(vex::brakeType::hold);
           //leftLift.stop(vex::brakeType::hold);

   leftLift.rotateTo(0, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);
    rightLift.rotateTo(0, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);

    

}

void turnLeft (double deg){
 rightDrive.rotateFor(deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct, false);
 leftDrive.rotateFor(-deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct);
  
}
void turnRight(double deg){
  leftDrive.rotateFor(deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct, false);
  rightDrive.rotateFor(-deg * 280/90, vex::rotationUnits::deg, 80, vex::velocityUnits:: pct);

}


void redStack (){
          deploy();
          vex::task::sleep(1200);
          
          //start the rollers
          rollerLeft.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);

          //drive forward
          leftDrive.rotateFor(1150,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(1150,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          vex::task::sleep(50);

          //stop rollers
          rollerLeft.stop(hold);
          rollerRight.stop(hold);

          //drive back
          leftDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          //turns to face the goal
          turnRight(171);
          vex::task::sleep(200);

          //drive to goal -- uses time instead because we might not always reach same value
          leftDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
          rightDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);

          vex::task::sleep(1100);

          leftDrive.stop(hold);
          rightDrive.stop(hold);

          //place dwon stack
          rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
           
          rollerLeft.spin(vex::directionType::rev, 5, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::rev, 5, vex::velocityUnits::pct);

          vex::task::sleep(2800);

         rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 

          rollerLeft.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);

          vex::task::sleep(200);

          //back up
          leftDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct);

          //stop rollers
          rollerLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
          
          
}

void bigZone(){
          deploy();
          vex::task::sleep(1200);

          leftDrive.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
          rightDrive.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);

          vex::task::sleep(1000);

          leftDrive.spin(vex::directionType:: fwd, 50, vex::velocityUnits::pct);
          rightDrive.spin(vex::directionType:: fwd, 50, vex::velocityUnits::pct);
          
          vex::task::sleep(1000);

          leftDrive.stop(hold);
          rightDrive.stop(hold);

}


void blueStack (){
          deploy();
          vex::task::sleep(1200);
          
          //start the rollers
          rollerLeft.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);

          //drive forward
          leftDrive.rotateFor(1150,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(1150,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          vex::task::sleep(50);

          //stop rollers
          rollerLeft.stop(hold);
          rollerRight.stop(hold);

          //drive back
          leftDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          //turns to face the goal
          turnLeft(171);
          vex::task::sleep(200);

          //drive to goal -- uses time instead because we might not always reach same value
          leftDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
          rightDrive.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);

          vex::task::sleep(1100);

          leftDrive.stop(hold);
          rightDrive.stop(hold);

          //place dwon stack
          rotator.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct); 
           
          rollerLeft.spin(vex::directionType::rev, 5, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::rev, 5, vex::velocityUnits::pct);

          vex::task::sleep(2800);

         rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 

          rollerLeft.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::rev, 35, vex::velocityUnits::pct);

          vex::task::sleep(200);

          //back up
          leftDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          rightDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct);

          //stop rollers
          rollerLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
          rollerRight.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
          
          
}


//void blueStack(){
 //deploy();
          //vex::task::sleep(1200);
          
          //start the rollers
          //rollerLeft.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
          //rollerRight.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);

          //drive forward
          //leftDrive.rotateFor(1250,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          //rightDrive.rotateFor(1250,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          //vex::task::sleep(500);

          //stop rollers
          //rollerLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
          //rollerRight.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);

          //drive back
          //leftDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          //rightDrive.rotateFor(-70,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
          //turns to face the goal
          //turnLeft(177);
          //vex::task::sleep(200);

          //drive to goal -- uses time instead because we might not always reach same value
          //leftDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
          //rightDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

          //vex::task::sleep(1000);

           //leftDrive.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
          //rightDrive.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);

          //place dwon stack
          //rotator.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); 

          //vex::task::sleep(2500);

         //rotator.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 

          //rollerLeft.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
          //rollerRight.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);

          //vex::task::sleep(400);

          //back up
          //leftDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct, false);
          //rightDrive.rotateFor(-200,vex::rotationUnits::deg,40,vex::velocityUnits::pct);
          
//}


/*void turn2 (double time){
   leftDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); //(Axis3+Axis4)/2
    rightDrive.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);//(Axis3-Axis4)/2        
        
}*/

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
 
   deploy();

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
        
        else {
            rotator.stop(hold);
        }
        
        //rollers
        if(Controller1.ButtonR1.pressing()){
            rollerLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            rollerRight.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if (Controller1.ButtonR2.pressing()){
            rollerLeft.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
            rollerRight.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        else {
            rollerLeft.stop(vex::brakeType::hold);
            rollerRight.stop(vex::brakeType::hold);
        }
       
        
        //lift
        if(Controller1.ButtonL1.pressing()){
            rightLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
            leftLift.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
            
            
        }
        else if(Controller1.ButtonL2.pressing()){
            rightLift.spin(vex::directionType::rev, 30, vex::velocityUnits::pct);  
            leftLift.spin(vex::directionType::rev, 30, vex::velocityUnits::pct);
           
        
        }
        else {
         
           rightLift.stop(vex::brakeType::hold);
           leftLift.stop(vex::brakeType::hold);
            
        } 

        if(Controller1.ButtonUp.pressing()){
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


          

