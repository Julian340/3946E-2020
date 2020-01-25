using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftDrive;
extern motor rightDrive;
extern motor rotator;
extern motor rollerLeft;
extern motor rollerRight;
extern motor rightLift;
extern motor leftLift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );