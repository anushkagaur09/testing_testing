using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor rightFront;
extern motor leftFront;
extern motor leftBack;
extern motor rightBack;
extern motor Intake;
extern digital_out Expansion;
extern motor flywheel;
extern motor rightMiddle;
extern motor leftMiddle;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );