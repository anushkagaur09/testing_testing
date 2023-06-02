#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor rightFront = motor(PORT19, ratio18_1, false);
motor leftFront = motor(PORT3, ratio18_1, false);
motor leftBack = motor(PORT8, ratio18_1, true);
motor rightBack = motor(PORT14, ratio18_1, true);
motor Intake = motor(PORT1, ratio18_1, true);
digital_out Expansion = digital_out(Brain.ThreeWirePort.H);
motor flywheel = motor(PORT11, ratio18_1, true);
motor rightMiddle = motor(PORT17, ratio18_1, true);
motor leftMiddle = motor(PORT18, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}