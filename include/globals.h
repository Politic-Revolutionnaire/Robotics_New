#include "main.h"

#define RIGHT_WHEELS_PORT1_OP 13
#define RIGHT_WHEELS_PORT2_OP 2
#define RIGHT_WHEELS_PORT1_AUTO -13 //Top right
#define RIGHT_WHEELS_PORT2_AUTO -2 //Bottom right
#define LEFT_WHEELS_PORT1 3 //Top left
#define LEFT_WHEELS_PORT2 4 //Bottom left
#define ARM_PORT 7
#define INTAKE_PORT1 5
#define INTAKE_PORT2 6
#define TRAY_PORT 11

extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern pros::Motor left_motor1(LEFT_WHEELS_PORT1);
extern pros::Motor left_motor2(LEFT_WHEELS_PORT2);
extern pros::Motor right_motor1(RIGHT_WHEELS_PORT1_OP,true);
extern pros::Motor right_motor2(RIGHT_WHEELS_PORT2_OP,true);
extern pros::Motor arm (ARM_PORT);
extern pros::Motor intake1 (INTAKE_PORT1,true);
extern pros::Motor intake2 (INTAKE_PORT2);
extern pros::Motor tray (TRAY_PORT);
