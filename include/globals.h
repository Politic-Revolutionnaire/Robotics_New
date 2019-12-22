#include "main.h"
#include "api.h"
#include "pros/api_legacy.h"

//deadports 12

#define RIGHT_WHEELS_PORT1_OP 14
#define RIGHT_WHEELS_PORT2_OP 1
#define RIGHT_WHEELS_PORT1_AUTO -14 //Top right
#define RIGHT_WHEELS_PORT2_AUTO -1 //Bottom right
#define LEFT_WHEELS_PORT1 2 //Top left
#define LEFT_WHEELS_PORT2 3 //Bottom left
#define ARM_PORT 13
#define INTAKE_PORT1 4
#define INTAKE_PORT2 5
#define TRAY_PORT 11
#define BUTTON_PORT 3
#define SWITCH_PORT 4

extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern pros::Motor left_motor1(LEFT_WHEELS_PORT1);
extern pros::Motor left_motor2(LEFT_WHEELS_PORT2);
extern pros::Motor right_motor1(RIGHT_WHEELS_PORT1_OP,true);
extern pros::Motor right_motor2(RIGHT_WHEELS_PORT2_OP,true);
extern pros::Motor arm (ARM_PORT);
extern pros::Motor intake1 (INTAKE_PORT1,true);
extern pros::Motor intake2 (INTAKE_PORT2);
extern pros::Motor tray (TRAY_PORT);
extern pros::ADIDigitalIn button (BUTTON_PORT);
extern pros::ADIDigitalIn switchy (SWITCH_PORT);
