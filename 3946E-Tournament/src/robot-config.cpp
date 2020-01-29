#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDrive = motor(PORT1, ratio18_1, false);
motor rightDrive = motor(PORT17, ratio18_1, true);
motor rotator = motor(PORT12, ratio18_1, true);
motor rollerLeft = motor(PORT6, ratio18_1, true);
motor rollerRight = motor(PORT2, ratio18_1, false);
motor rightLift = motor(PORT16, ratio18_1, false);
motor leftLift = motor(PORT7, ratio18_1, true);
controller Controller2 = controller(partner);
inertial gyroscope = inertial(PORT20);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}