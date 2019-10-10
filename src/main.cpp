#include "main.h"
#include "okapi/api.hpp"
#include "globals.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
using namespace okapi;

//Subject to change
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

const double liftP = 1.0;
const double liftI = 0.001;
const double liftD = 0.1;

void autonomous() {
	pros::lcd::set_text(1, "Auton!");
	std::cout << "auto";
	auto chassis = ChassisControllerFactory::create(
		{LEFT_WHEELS_PORT1, LEFT_WHEELS_PORT2},
		{RIGHT_WHEELS_PORT1_AUTO,RIGHT_WHEELS_PORT2_AUTO},
		AbstractMotor::gearset::green, //Gearset (200rpm)
		{3_in, 12.5_in} //Wheel size, wheelbase width orig 4.125, 12.5
	);

	//TODO profile robot to determine actual values for this
	auto profileController = AsyncControllerFactory::motionProfile(
		3, //Max linear velocity
		3, //Max linear acceleration
		4, //Max linear jerk
		chassis //Chassis controller
	);

	auto profileControllerSecondary = AsyncControllerFactory::motionProfile(
		3,
		4,
		8,
		chassis);

	auto liftController = AsyncControllerFactory::posPID(ARM_PORT, liftP, liftI, liftD); //Max 270 degrees

	//Sample movement Code
	profileController.generatePath({
		Point{0_m, 0_m, 0_deg},
		Point{2_m, 0_m, 0_deg}},
		"Test"
	);

	profileController.setTarget("Test");
	profileController.waitUntilSettled();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::lcd::set_text(1, "Op!");
	std::cout << "op";
	left_motor1.set_brake_mode(MOTOR_BRAKE_HOLD);
	left_motor2.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_motor1.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_motor2.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_brake_mode(MOTOR_BRAKE_HOLD);

	//FILE* fileWrite = fopen("/usd/test.txt", "w");

	while (true) {
		std::cout << master.get_analog(ANALOG_LEFT_Y);
		left_motor1.move(master.get_analog(ANALOG_LEFT_Y));
		left_motor2.move(master.get_analog(ANALOG_LEFT_Y));
		right_motor1.move(master.get_analog(ANALOG_RIGHT_Y));
		right_motor2.move(master.get_analog(ANALOG_RIGHT_Y));
		if (master.get_digital(DIGITAL_R1)) {
			arm.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_R2)) {
			arm.move_velocity(-200);
		}
		else {
			arm.move_velocity(0);
		}
		if (master.get_digital(DIGITAL_L1)) {
			intake1.move_velocity(200);//Max rpm
			intake2.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_L2)) {
			intake1.move_velocity(-100);//outtake
			intake2.move_velocity(-100);
		}
		else {
			intake1.move_velocity(0);
			intake2.move_velocity(0);
		}
		if(master.get_digital(DIGITAL_B)) {
			tray.move_velocity(-200);
		}
		else if (master.get_digital(DIGITAL_X)) {
			tray.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_A)) {
			tray.move_velocity(50);
		}
		else {
			tray.move_velocity(0);
		}
		//double leftVelocity = (left_motor1.get_actual_velocity() + left_motor2.get_actual_velocity())/2;
		//double rightVelocity = (right_motor1.get_actual_velocity() + right_motor2.get_actual_velocity())/2;
		//double linearVelocity = (leftVelocity+rightVelocity)/2;
		//std::string output = std::to_string(leftVelocity) + " " + std::to_string(rightVelocity) + " " + std::to_string(linearVelocity);
		//fputs(output.c_str(), fileWrite);
		//pros::delay(10);
	}
}
