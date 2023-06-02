/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                           d                                                 */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// rightFront           motor         19           
// leftFront            motor         3               
// leftBack             motor         8               
// rightBack            motor         14              
// Intake               motor         1               
// Expansion            digital_out   H               
// flywheel             motor         11              
// rightMiddle          motor         17              
// leftMiddle           motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//auton selector 
int autonselect = 1;
int numOfAutons = 9;

int getSign (double inputValue) {
  if (inputValue > 0){
    return 1;
  }
  else if (inputValue < 0){
    return -1;
  }
  else return 0;
}

void driveFunction(){
  
}
//PID settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

int error; //SensorValue - DesiredValue --- positional value
int prevError = 0; //position 20 milliseconds ago
int derivative; //difference between error and previous error --- calculates speed
int totalError = 0;// totalError = totalError + error --- integral converts position to absement

//turn variables
int turnError; //SensorValue - DesiredValue --- positional value
int turnPrevError = 0; //position 20 milliseconds ago
int turnDerivative; //difference between error and previous error --- calculates speed
int turnTotalError = 0;

int desiredValue = 0;
int desiredTurnValue = 0;

bool enabledrivePID = true;
//switch to reset the Drive
bool resetDriveSensors = false;


int drivePID(){
  
  while(enabledrivePID){

    if(resetDriveSensors){
      resetDriveSensors = false;

      rightFront.setPosition(0, degrees);
      leftFront.setPosition(0, degrees);
      leftBack.setPosition(0, degrees);
      rightBack.setPosition(0, degrees);
    }
    //get the position of the motors
    int rightFrontPosition = rightFront.position(degrees);
    int leftFrontPosition = leftFront.position(degrees);
    int leftBackPosition = leftBack.position(degrees);
    int rightBackPosition = rightBack.position(degrees);
    //////////////////////////////////////////////////////////////////////////////////
    //lateral movement PID
    ////////////////////////////////////////////////////////////////////////
    //get the average of the four motors
    int averagePosition = (rightFrontPosition + leftFrontPosition + leftBackPosition + rightBackPosition)/4;

    error = averagePosition - desiredValue;

    derivative = error - prevError;

    //absement = position * time -- this is the integral
    totalError += error;

    //add everything up to a mootor power
    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI)/12;

    //////////////////////////////////////////////////////////////////////////////////
    //turning PID
    int turnDifference = (rightFrontPosition - leftFrontPosition);

    turnError = turnDifference - desiredTurnValue;

    turnDerivative = turnError - turnPrevError;



    //absement = position * time -- this is the integral
    turnTotalError += turnError;

    //add everything up to a motor power
    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI)/12.0;
    ////////////////////////////////////////////////////////////////////////
    //putting in the motor power into the motor statements
    rightFront.spin(forward, lateralMotorPower + turnMotorPower, percent);
    leftFront.spin(forward, lateralMotorPower - turnMotorPower, percent);
    leftBack.spin(forward, lateralMotorPower - turnMotorPower, percent);
    rightBack.spin(forward, lateralMotorPower + turnMotorPower, percent);

    prevError = error;
    turnPrevError = turnError;

    vex::task::sleep(20);
  }

  return 1;

}

void driveDrive(){
  int forwardAmount = Controller1.Axis3.position();
  int turnAmount = (Controller1.Axis1.position())/2;
  rightFront.setVelocity(100, percent);
  leftFront.setVelocity(100, percent);
  leftBack.setVelocity(100, percent);
  rightBack.setVelocity(100, percent);

  rightFront.spin(reverse,forwardAmount + turnAmount, percent);
  rightMiddle.spin(forward, forwardAmount + turnAmount, percent);
  leftFront.spin(forward, forwardAmount - turnAmount, percent);
  leftBack.spin(reverse, forwardAmount - turnAmount, percent);
  leftMiddle.spin(reverse, forwardAmount - turnAmount, percent);
  rightBack.spin(forward, forwardAmount + turnAmount, percent);
}

void rollerCode(){
  Intake.setVelocity(100,percent);
  if(Controller1.ButtonR1.pressing() == true){
    Intake.spin(forward);
  }
  else if(Controller1.ButtonR2.pressing() == true){
    Intake.spin(reverse);
  }
  else{
    Intake.stop();
  }
}
void intakeCode(){
  Intake.setVelocity(100, percent);
  if(Controller1.ButtonL1.pressing() == true){
    Intake.spin(vex::directionType::fwd);
    
  }
  else if(Controller1.ButtonL2.pressing() == true){
    Intake.spin(vex::directionType::rev);
  }
  else{
    Intake.stop();
  }
}

///////////////////////////////////////////////////////////////////////
//////////////////////////flywheel pid/////////////////////////////////

bool Controller1XY = true;

double fly_kp = 0.05; // how fast it increases
double fly_ki = 0.2; // how much offshoot/range of fluctuation
double fly_kd = 0.00005; // how many fluctuations are there
double speed_margin = 0;
double speed_marg_pct = 2;
bool flyescvar = false;
int speed_volt = 0;

//flywheel spin

void flywheel_spin_fwd(double flywheel_target_speed_pct) {
  
  flywheel.setVelocity(flywheel_target_speed_pct, pct);
  flywheel.spin(directionType::fwd);
}

//flywheel spin PID code
void flywheel_PID(double flywheel_target_speed_pct){
double averagevolt = 0;
double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;
Controller1.Screen.setCursor(1,1);
Controller1.Screen.print("         ");
wait(20,msec);
 
 while (flyescvar == false) {
    averagevolt = flywheel.voltage();
    error = flywheel_target_speed_volt - averagevolt;
    derivative = preverror - error;
    errorsum += error;
    preverror = error;
    speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
    speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
    wait(20,msec);
  
    if(speed_margin <= speed_marg_pct) {
      flyescvar = true;
    } else {
        flywheel.spin(forward, speed_volt, volt);
    }
    wait(20, msec);
  }
 Controller1.Screen.setCursor(3,9);
 Controller1.Screen.print("DONE");
 wait(20,msec);
 
 // Maintain the speed
 flywheel.spin(forward, speed_volt, volt);
}
bool flywheelStart = false;
void flywheelMovement() {
    if(Controller1.ButtonDown.pressing()){
      flywheel.setVelocity(65, pct);
      flywheel.spin(forward);
      Controller1XY = false;
    } else if(Controller1.ButtonUp.pressing()) {
      flywheel.setVelocity(52, pct);
      flywheel.spin(forward);
      Controller1XY = false;
    } else if(!Controller1XY) {
      flywheel.setStopping(coast);
      flywheel.stop();
    }
    if(Controller1.ButtonDown.pressing()){
      flywheel.setVelocity(65, pct);
      flywheel.spin(forward);
      Controller1XY = false;
    } 
    else if(Controller1.ButtonUp.pressing()) {
      flywheel.setVelocity(52, pct);
      flywheel.spin(forward);
      Controller1XY = false;
    } else if(!Controller1XY) {
      flywheel.setStopping(coast);
      flywheel.stop();
    }
}

 int selected = 0;
std::string autons[9] = {"Disabled", "1 Roller Red perp side", "2 Roller Red parallel side", "3 Roller Blue", "4 Roller Blue perp side" , "5 red shoot disk into high goal parallel side" , "6 blue shoot disk into high goal parallel side", "7 red awp" , "8 blue awp"};
int size = sizeof(autons);

bool elevated = false;

void autonSelector(){
  Controller1.Screen.clearScreen();
  task::sleep(100);
  while(true){
    Controller1.Screen.clearScreen();
    task::sleep(100);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print((autons[selected] + ",").c_str()); //e=mc^2
    Controller1.Screen.newLine();
    Controller1.Screen.print((elevated ? "Elevated" : "Default"));
    task::sleep(100);
     if(Controller1.ButtonRight.pressing()){
      elevated = !elevated;
        if (!elevated) {
          selected = (selected + 1 + size) % size;
        }
     }else if(Controller1.ButtonLeft.pressing()){
       elevated = !elevated;
       if (elevated) {
        selected = (selected - 1 + size) % size;
       }
     }else if(Controller1.ButtonA.pressing()){
       task::sleep(100);
       if(Controller1.ButtonA.pressing()){
         goto slctEnd;
       }
     }
   }
   slctEnd:
   Controller1.rumble("..");
}
void pre_auton(void) {
// Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  autonSelector();
  Expansion.set(false);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
  switch(selected){
    case 0:{ //Disabled
      break;
    }
    case 1:{ //1 Roller Red perp side
    //shoot out disc to low goal
      flywheel.setVelocity(35, percent);
      flywheel.spin(forward);
      wait(1, seconds);
      Intake.spin(forward);
      wait(3, seconds);
      flywheel.stop();
      Intake.stop();
     //turn to realign with the roller
      leftFront.spin(reverse);
      rightBack.spin(forward);
      wait(0.78, seconds);
      rightFront.stop();
      leftBack.stop();
     //move back a bit to get closer to the roller
      wait(0.1, seconds);
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.65, seconds);
      rightFront.stop();
      leftBack.stop();
      wait(0.1, seconds);
      
     //move towards the roller
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(0.3, seconds);
     //stop the motors
      rightFront.stop();
      leftFront.stop();
      
     //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 250, degrees, true);
     //move back a bit
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(0.2, seconds);
      leftFront.stop();
      rightFront.stop();
    
      break;
    }
    case 2:{ //2 Roller Red parallel side 
    //move towards the roller
      rightFront.spinFor(reverse, 40, degrees, false);
      leftFront.spinFor(forward, 40, degrees, false);
    //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 200, degrees);
    //move back to original position
      rightFront.spinFor(forward, 40, degrees, false);
      leftFront.spinFor(reverse, 40, degrees);
      break;
    }
    case 3: { //3 Roller Blue
    //move towards the roller
      rightFront.spinFor(reverse, 40, degrees, false);
      leftFront.spinFor(forward, 40, degrees, false);
    //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 200, degrees);
    //move back to original position
      rightFront.spinFor(forward, 40, degrees, false);
      leftFront.spinFor(reverse, 40, degrees);
      break;
    }
    case 4: { //4 Roller Blue perp side
    //shoot out disc to low goalj
      flywheel.setVelocity(35, percent);
      flywheel.spin(forward);
      wait(1, seconds);
      Intake.spin(forward);
      wait(3, seconds);
      flywheel.stop();
      Intake.stop();
     //turn
      leftFront.spin(forward);
      rightFront.spin(reverse);
      wait(0.78, seconds);
      rightFront.stop();
      leftFront.stop();
     //move back a bit to get closer to the roller
      wait(0.1, seconds);
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.65, seconds);
      rightFront.stop();
      leftBack.stop();
      wait(0.1, seconds);
     //move towards the roller
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(0.57, seconds);
     //stop the motors
      rightFront.stop();
      leftFront.stop();
     //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 250, degrees);
     //move back a bit
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(0.2, seconds);
      leftFront.stop();
      rightFront.stop();
    }
    case 5: { //5 red shoot disk into high goal parallel side
    //set velocity
      Intake.setVelocity(100, percent);
      rightFront.setVelocity(100, percent);
      leftFront.setVelocity(100, percent);
      leftBack.setVelocity(100, percent);
      flywheel.setVelocity(63, percent);
      //move forward torwards disk
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(0.2, seconds);
      rightFront.stop();
      leftFront.stop();
      //turn towards disc
      wait(0.1, seconds);
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.22, seconds);
      rightFront.stop();
      leftBack.stop();
      //move forward and intake the disc
      leftFront.spin(forward);
      rightFront.spin(reverse);
      wait(0.35, seconds);
      Intake.spin(forward);
      wait(0.3, seconds);
      rightFront.setVelocity(25, percent);
      leftFront.setVelocity(25, percent);
      leftFront.spin(forward);
      rightFront.spin(reverse);
      wait(2, seconds);
      rightFront.stop();
      leftFront.stop();
      wait(3.05, seconds);
    
      Intake.stop();
      wait(0.5, seconds);
      //turn to shooting position
      rightFront.setVelocity(70, percent);
      leftFront.setVelocity(70, percent);
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.923, seconds);
      rightFront.stop();
      leftBack.stop();
      //shoot out the disc ._.
      flywheel_PID(80);
      wait(2, seconds);
      Intake.spin(forward);
      wait(4, seconds);
      flywheel.stop();
      Intake.stop();
      break;
      
    }
    case 6: {//6 blue shoot disk into high goal parallel side
     rightBack.setVelocity(100,percent);
     rightFront.setVelocity(100,percent);
     rightMiddle.setVelocity(100,percent);
     leftBack.setVelocity(100,percent);
     leftFront.setVelocity(100,percent);
     leftMiddle.setVelocity(100,percent);
     //turn move
     rightBack.spin(forward);
     rightFront.spin(forward);
     rightMiddle.spin(forward);
     leftBack.spin(reverse);
     leftMiddle.spin(reverse);
     leftFront.spin(reverse);
     wait(4,seconds);
         }
    case 7: {//7 red awp
     //roll roller first
      //move forward towards the roller
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(0.6, seconds);
      rightFront.stop();
      leftFront.stop();
      wait(0.1, seconds);
      //roll the roller
      Intake.spinFor(reverse, 75, degrees);
      //move back to original position
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(0.5, seconds);
      rightFront.stop();
      leftFront.stop();
      //turn 
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.85, seconds);
      rightFront.stop();
      leftBack.stop();
      //move forward towards final turning position
      wait(0.5, seconds);
      rightFront.setVelocity(70, percent);
      leftFront.setVelocity(70, percent);
      rightBack.spin(reverse);
      leftFront.spin(reverse);
      wait(1.22, seconds);
      rightBack.stop();
      leftFront.stop();
      //turn to face the high goal
      wait(0.5, seconds);
      rightFront.spin(reverse);
      leftBack.spin(forward);
      wait(0.79, seconds);
      rightFront.stop();
      leftBack.stop();
      //shoot out the discs
      Intake.setVelocity(100, percent);
      wait(0.25, seconds);
      flywheel_PID(58);
      wait(4, seconds);
      Intake.spin(forward);
      wait(3, seconds);
      flywheel.stop();
      Intake.stop();
    }
     { //8 blue awp
      
    }
}
}

void flywheelUp(void) {
  flywheel_PID(75);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    driveDrive();
    intakeCode();
    rollerCode();
    //flywheelMovement();
    /////////////////                                            //////////////////flywheel pid calling//////////////////////////////////////////////////
    Controller1.ButtonUp.pressed(flywheelUp);
      
    //Controller1.ButtonDown.pressed(flywheelDown);
    if((Controller1.ButtonDown.pressing() == true)){
      flywheel.stop();
    }
    
    //expansion using two pistons being controlled together to trigger the catapults
    //these pistons will use boolean values to send info to the brain
    //so we will be using simple detection for this
    
    if(Controller1.ButtonX.pressing() && Controller1.ButtonY.pressing()){
      Expansion.set(true);
    }
    else if(Controller1.ButtonB.pressing()){
      Expansion.set(false);
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    
  }
}
