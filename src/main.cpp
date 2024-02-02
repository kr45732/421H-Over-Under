/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       krish                                                     */
/*    Created:      2/1/2024, 5:31:03 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
controller Controller = controller(primary);

motor FrontLeft = motor(PORT8, ratio6_1, true);
motor BackLeft = motor(PORT3, ratio6_1, true);
motor_group LeftDrive = motor_group(FrontLeft, BackLeft);

motor FrontRight = motor(PORT4, ratio6_1, false);
motor BackRight = motor(PORT1, ratio6_1, false);
motor_group RightDrive = motor_group(FrontRight, BackRight);

motor Flywheel = motor(PORT9, ratio6_1, false);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

bool flywheelOnFwd = false;
bool flywheelOnRev = false;

void toggleFlywheelFwd() {
  if (flywheelOnFwd) {
    Flywheel.stop();
  } else {
    Flywheel.spin(fwd, 15, volt);
  }

  flywheelOnFwd = !flywheelOnFwd;
}

void toggleFlywheelRev() {
  if (flywheelOnRev) {
    Flywheel.stop();
  } else {
    Flywheel.spin(reverse, 7, volt);
  }

  flywheelOnRev = !flywheelOnRev;
}

void chassis() {
  int deadZone = 18;

  double forward = Controller.Axis3.position() * 1.7 * 3;
  double turnVal = Controller.Axis1.position() * 2.2;

  int leftVoltage = forward + turnVal;
  int rightVoltage = forward - turnVal;

  bool frontLeftMoving = !(-deadZone < leftVoltage && leftVoltage < deadZone);
  bool frontRightMoving =
      !(-deadZone < rightVoltage && rightVoltage < deadZone);
  if (frontLeftMoving || frontRightMoving) {
    LeftDrive.spin(fwd, leftVoltage, rpm);
    RightDrive.spin(fwd, rightVoltage, rpm);
  } else {
    LeftDrive.stop();
    RightDrive.stop();
  }
}

void usercontrol(void) {
  Controller.ButtonR1.pressed(toggleFlywheelFwd);
  Controller.ButtonL1.pressed(toggleFlywheelRev);

  while (true) {
    chassis();

    // Sleep the task for a short amount of time to prevent wasted resources
    wait(20, msec);
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
    wait(100, msec);
  }
}
