#ifndef MOTOR_H
#define MOTOR_H

// Motor Controls
#define MOTOR_SPINUP_LAG 200          // How long we give the motors before we know that have spun up.
#define MOTOR_MAX_SPEED 2000          //

int MaxMotorSpeed = MOTOR_MAX_SPEED;  //
int DecelerateTime = 4000;            // Ramp D
int AccelerateTime = 0;               // Ramp U
long MotorRampUpPerMS = 0;            //
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100;  // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;

#define COMMAND_REV_NONE 0
#define COMMAND_REV_FULL 1
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
byte MotorSpeedFull = 80;
int MotorStartDwellTime = 0;  // Use this to hold the rev
int MotorStopDwellTime = 0;   // Use this to hold the rev

//----------------------------- Process Motor Speed Subroutine -------------------------------------------
// Calculate the desired motor speed based on where we are, and where we need to be. This takes into account the ramp rates of the motor.
void ProcessMotorSpeed() {
  // Don't do anything if the motor is already running at the desired speed.
  if (CurrentMotorSpeed == TargetMotorSpeed) {
    return;
  }

  unsigned long CurrentTime = millis();  // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if (MSElapsed == 0)  // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if (CurrentMotorSpeed < TargetMotorSpeed) {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if (SpeedDelta < 1) return;                          // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta;  // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if (NewMotorSpeed + 10 >= TargetMotorSpeed) {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if (CurrentMotorSpeed > TargetMotorSpeed) {
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if (SpeedDelta < 1) return;                          // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta;  // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if (NewMotorSpeed - 10 <= TargetMotorSpeed) {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

//------------------------------------------ PROCESS SPEED CONTROL -----------------------------------------------
// We need to set the Target Motor Speed here, as a percentage. Sense check it to ensure it's not too slow, or too fast.
void ProcessSpeedControl() {
  static byte LastSetMaxSpeed = 100;

  if (CommandRev == COMMAND_REV_FULL) SetMaxSpeed = MotorSpeedFull;
  if (CommandRev == COMMAND_REV_NONE) SetMaxSpeed = 0;

  if (LastSetMaxSpeed == SetMaxSpeed) return;  // Speed hasn't changed

  if (CommandRev > COMMAND_REV_NONE) {
    SetMaxSpeed = constrain(SetMaxSpeed, 30, 100);  // Constrain between 30% and 100%
  }

  TargetMotorSpeed = map(SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed);  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;
}

#endif