#include "main.h"
#include "api.h"
#include "pros/api_legacy.h"

//deadports 3,12

#define RIGHT_WHEELS_PORT1 9
#define RIGHT_WHEELS_PORT2 10
#define RIGHT_WHEELS_PORT1_OP 9
#define RIGHT_WHEELS_PORT2_OP 10
#define RIGHT_WHEELS_PORT1_AUTO -9 //Top right
#define RIGHT_WHEELS_PORT2_AUTO -10 //Bottom right
#define LEFT_WHEELS_PORT1 2 //Top left
#define LEFT_WHEELS_PORT2 1 //Bottom left
#define ARM_PORT 4
#define INTAKE_PORT1 8
#define INTAKE_PORT2 6
#define TRAY_PORT 5
#define RIGHT_WHEELS_ENCODER1 5
#define RIGHT_WHEELS_ENCODER2 6
#define LEFT_WHEELS_ENCODER1 7
#define LEFT_WHEELS_ENCODER2 8
#define BUTTON_PORT 1
#define ANGLER_PORT 2
#define ARM_HIGH_PORT 3
#define ARM_LOW_PORT 4

extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern pros::Motor left_motor1(LEFT_WHEELS_PORT1);
extern pros::Motor left_motor2(LEFT_WHEELS_PORT2);
extern pros::Motor right_motor1(RIGHT_WHEELS_PORT1_OP,true);
extern pros::Motor right_motor2(RIGHT_WHEELS_PORT2_OP,true);
extern pros::Motor arm (ARM_PORT);
extern pros::Motor intake1 (INTAKE_PORT1, true);
extern pros::Motor intake2 (INTAKE_PORT2);
extern pros::Motor tray (TRAY_PORT, true);
extern pros::ADIDigitalIn button(BUTTON_PORT);
extern pros::ADIDigitalIn arm_upper(ARM_HIGH_PORT);
extern pros::ADIDigitalIn arm_lower(ARM_LOW_PORT);
extern pros::ADIAnalogIn angler (ANGLER_PORT);
extern pros::ADIEncoder right_encoder(RIGHT_WHEELS_ENCODER1, RIGHT_WHEELS_ENCODER2, false);
extern pros::ADIEncoder left_encoder(LEFT_WHEELS_ENCODER1, LEFT_WHEELS_ENCODER2, false);
