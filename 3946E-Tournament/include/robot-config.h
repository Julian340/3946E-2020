using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor lDrive;
extern motor rDrive;
extern motor rotator;
extern motor lRoller;
extern motor rRoller;
extern motor rLift;
extern motor lLift;
extern controller Controller2;
extern inertial gyroscope;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );