
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