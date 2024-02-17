/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       krish                                                     */
/*    Created:      2/1/2024, 5:31:03 PM                                      */
/*    Description:  421H Over Under Code                                      */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// Global instances
competition Competition;
controller Controller = controller(primary);
brain Brain = brain();

// Drive motors
motor FrontLeft = motor(PORT8, ratio6_1, true);
motor BackLeft = motor(PORT3, ratio6_1, true);
motor_group LeftDrive = motor_group(FrontLeft, BackLeft);
motor FrontRight = motor(PORT4, ratio6_1, false);
motor BackRight = motor(PORT1, ratio6_1, false);
motor_group RightDrive = motor_group(FrontRight, BackRight);

// Other motors
motor Flywheel = motor(PORT9, ratio6_1, false);
motor Intake = motor(PORT6, ratio6_1, false);
motor Arm = motor(PORT5, ratio36_1, false);

// Pneumatics
pneumatics LeftPneumatics = pneumatics(Brain.ThreeWirePort.F);
pneumatics RightPneumatics = pneumatics(Brain.ThreeWirePort.E);

// Sensors
// inertial InertialSensor = inertial(PORT6);  // check port

// Function prototypes
void rotateTo(double degrees, double speed, int differenceThreshold = 20);
void move(int degrees, int degreesPerSecond, bool hold = true);
void chassis();
void togglePneumatics();
void resetDriveEncoders();
double avgDriveEncoderValue();

// Global variables
bool pneumaticsOpen = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // InertialSensor.calibrate();
  // while (InertialSensor.isCalibrating()) {
  //   wait(20, msec);
  // }

  resetDriveEncoders();
  // InertialSensor.resetHeading();
  // InertialSensor.resetRotation();
  // wait(20, msec);

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 0);
  Controller.Screen.print("Initialized");
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Deploy intake
  Intake.spinFor(-500, deg, 600, rpm);

  // Autonomous
  move(-1600, 400);
  move(400, 100);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Controller.ButtonR2.pressed(togglePneumatics);

  while (true) {
    chassis();

    if (Controller.ButtonR1.pressing()) {
      Flywheel.spin(fwd, 15, volt);
    } else {
      Flywheel.stop();
    }

    if (Controller.ButtonL1.pressing()) {
      Intake.spin(fwd, 600, rpm);
    } else if (Controller.ButtonL2.pressing()) {
      Intake.spin(fwd, -600, rpm);
    } else {
      Intake.stop();
    }

    if (Controller.ButtonUp.pressing()) {
      Arm.spin(fwd, 100, rpm);
    } else if (Controller.ButtonDown.pressing()) {
      Arm.spin(fwd, -100, rpm);
    } else {
      Arm.stop();
    }

    // Sleep the task for a short amount of time to prevent wasted resources
    wait(20, msec);
  }
}

// Autonomous functions
void rotateTo(double degrees, double speed, int differenceThreshold) {
  double rightDeg = 0;
  double leftDeg = 0;
  double difference = 0;
  bool turnLeft = false;

  // if (InertialSensor.heading() < degrees) {
  //   rightDeg = degrees - InertialSensor.heading();
  //   leftDeg = 360 + InertialSensor.heading() - degrees;
  // } else {
  //   rightDeg = 360 - (InertialSensor.heading() - degrees);
  //   leftDeg = InertialSensor.heading() - degrees;
  // }

  // if (rightDeg > leftDeg) {  // Turning left is shorter
  //   speed *= -1;
  //   difference = leftDeg;
  //   turnLeft = true;
  // } else {  // Turning right is shorter
  //   difference = rightDeg;
  // }

  // if (difference > differenceThreshold) {
  //   LeftDrive.spin(forward, speed, pct);
  //   RightDrive.spin(reverse, speed, pct);

  //   while (difference > differenceThreshold) {
  //     if (InertialSensor.heading() < degrees) {
  //       if (turnLeft) {
  //         difference = 360 + InertialSensor.heading() - degrees;
  //       } else {
  //         difference = degrees - InertialSensor.heading();
  //       }
  //     } else {
  //       if (turnLeft) {
  //         difference = InertialSensor.heading() - degrees;
  //       } else {
  //         difference = 360 - (InertialSensor.heading() - degrees);
  //       }
  //     }
  //     task::sleep(20);
  //   }
  // }

  // LeftDrive.spin(forward, speed * 0.1, pct);
  // RightDrive.spin(reverse, speed * 0.1, pct);

  // int overshootError = 3;
  // if (turnLeft) {
  //   waitUntil(degrees + overshootError >= InertialSensor.heading() &&
  //             InertialSensor.heading() >= degrees);
  // } else {
  //   waitUntil(degrees + overshootError >= InertialSensor.heading() &&
  //             InertialSensor.heading() >= (degrees));
  // }

  LeftDrive.stop();
  RightDrive.stop();
  wait(20, msec);
}

void resetDriveEncoders() {
  FrontLeft.resetPosition();
  FrontRight.resetPosition();
  BackLeft.resetPosition();
  BackRight.resetPosition();
}

double avgDriveEncoderValue() {
  return (fabs(FrontLeft.position(deg)) + fabs(FrontRight.position(deg)) +
          fabs(BackLeft.position(deg)) + fabs(BackRight.position(deg))) /
         4;
}

void moveForward(int left, int right) {
  LeftDrive.spin(fwd, left, rpm);
  RightDrive.spin(fwd, right, rpm);
}

/// @param degrees positive = forward & negative = backward
void move(int degrees, int degreesPerSecond, bool hold) {
  int direction = abs(degrees) / degrees;

  resetDriveEncoders();
  // InertialSensor.resetRotation();

  if (hold) {
    moveForward(
        degreesPerSecond * direction * 0.5,  //- InertialSensor.rotation() * 10,
        degreesPerSecond * direction *
            0.5);  // + InertialSensor.rotation() * 10);
    wait(0.3, sec);
  }

  while (avgDriveEncoderValue() < abs(degrees)) {
    moveForward(
        degreesPerSecond * direction,   // - InertialSensor.rotation() * 10,
        degreesPerSecond * direction);  // + InertialSensor.rotation() * 10);
    wait(20, msec);
  }

  if (hold) {
    moveForward(-600 * direction, -600 * direction);
    wait(125, msec);

    moveForward(0, 0);
    wait(20, msec);
  }
}

// Driver functions
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

void togglePneumatics() {
  if (pneumaticsOpen) {
    LeftPneumatics.close();
    RightPneumatics.close();
  } else {
    LeftPneumatics.open();
    RightPneumatics.open();
  }

  pneumaticsOpen = !pneumaticsOpen;
}

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
