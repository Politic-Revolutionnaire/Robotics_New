#include "main.h"
#include "okapi/api.hpp"
#include "globals.h"
#include "api.h"
#include "pros/api_legacy.h"

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

using namespace okapi;

//Wheel size, wheelbase width orig 4.125, 12.5
//±0.005m for 4.105in wheel size
//±0.5° for 9.55in wheelbase
//Wheelbase diameter 12.25in, wheelbase back 10in, wheelbase fron 11.25
//To adjust distance travelled decrease wheel size to increase distance and vice versa
//To adjust turn angle increase chassis size to increase turn angle
auto chassis = ChassisControllerBuilder()
	.withMotors({LEFT_WHEELS_PORT1, LEFT_WHEELS_PORT2},{RIGHT_WHEELS_PORT1_AUTO, RIGHT_WHEELS_PORT2_AUTO})
	.withDimensions(AbstractMotor::gearset::green, {{4.095_in, 9.75_in}, imev5GreenTPR})
	.withGains(
		{0.001, 0, 0.0001}, //Distance
		{0.001, 0, 0.0001}, //Turn
		{0.001, 0, 0.0001}) //Angle
	.withDerivativeFilters(
		{std::make_unique<AverageFilter<3>>()},
		{std::make_unique<AverageFilter<3>>()},
		{std::make_unique<AverageFilter<3>>()})
	.withOdometry()
	.withLogger(
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(),
			"/usd/logging.txt",
			Logger::LogLevel::debug
		)
	)
	.buildOdometry();

auto profile = AsyncMotionProfileControllerBuilder()
	.withLimits({
		1.15, //Max linear velocity
		5.275, //Max linear acceleration
		6}) //Max linear jerk
	.withOutput(chassis)
	.buildMotionProfileController();

int runTime = 1500;
int runSpeed = 200; //rpm
int runDelay = 0;
int nestedTime = 400;
int nestedDelay = 100;
//0 for L path, 1 for Z path skills, 2 for square path, 3 for mischellaneous testing, 4 for Z path auton
int autonMode = 3;
int sideSelector = -1;//1 for red, -1 for blue
int stackDelay = 500;
int stageDelay = 1000;
bool toggleControl = true;
double distance = 0.75;
int a = 1850;
int b = 100;
int c = 200;
int d = 0;
double r = 0.0523875;
bool trigger = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	profile->generatePath({{0_m, 0_m, 0_deg},{0.79_m, (sideSelector)*-0.82_m, 0_deg}}, "S");
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
 float erfInv(float x) {
  float tt1, tt2, lnx, sgn;
  sgn = (x < 0) ? -1.0f : 1.0f;

  x = (1 - x)*(1 + x);        // x = 1 - x*x;
  lnx = logf(x);

  tt1 = 2/(PI*0.147) + 0.5f * lnx;
  tt2 = 1/(0.147) * lnx;

  return(sgn*sqrtf(-tt1 + sqrtf(tt1*tt1 - tt2)));
 }

double toMeters(double d) {
	return d * (2 * PI * r) / (60 * 1000);
}

double toRounds(double d) {
	return 60 * 1000 * d / (2 * PI * r);
}

double inverseGauss(double d) {
	return a * erfInv(erf(-b / a) + (2 * toRounds(d) / (sqrt(PI) * c * a))) + b;
}

double gaussMovement(double t) {
	return toMeters(sqrt(PI) / 2 * c * a * (erf((t - b) / a) - erf(-b / a)));
}

double calculateTime(double d) {
	if (inverseGauss(d) < 2 * b) {
		return 1;
	} else {
		d = d - gaussMovement(2 * b);
		return toRounds(d) / c + 2 * b;
	}
}

double calculateSpeed(double t, double d) {
	if (t < b) {
		return gaussMovement(t);
	} else if (t < calculateTime(d) - b) {
		return c;
	} else {
		return gaussMovement(b + (t - (calculateTime(d) - b)));
	}
}

void moveDistanceSmooth(void* param) {
	int time = pros::c::millis();
	while(pros::c::millis() - time < calculateTime(distance))
	{
		int current = pros::c::millis();
		double speed = calculateSpeed(current - time, distance);
		if(pros::c::millis() - current >= 50 && !trigger)
		{
			int mil = pros::c::millis() - current;
			pros::lcd::set_text(1, std::to_string(mil));
			trigger = true;
		}
		left_motor1.move_velocity(speed);
		left_motor2.move_velocity(speed);
		right_motor1.move_velocity(speed);
		right_motor2.move_velocity(speed);
		/*
		if(left_motor1.get_actual_velocity() != NAN && left_motor2.get_actual_velocity() != NAN
			&& right_motor1.get_actual_velocity() != NAN && right_motor2.get_actual_velocity() != NAN)
		{
				double vel = (left_motor1.get_actual_velocity() + left_motor2.get_actual_velocity()
				+ right_motor1.get_actual_velocity() + right_motor2.get_actual_velocity())/4;

		}
		*/
		pros::delay(1);
	}
}

void backwardTask(void* param) {
	int time = pros::c::millis();
	while(pros::c::millis() - time <= 750)
	{
		left_motor1.move_velocity(-75);
		left_motor2.move_velocity(-75);
		right_motor1.move_velocity(-75);
		right_motor2.move_velocity(-75);
	}
	left_motor1.move_velocity(0);
	left_motor2.move_velocity(0);
	right_motor1.move_velocity(0);
	right_motor2.move_velocity(0);
}

void outtakeTask(void* param) {
	int time = pros::c::millis();
	while(pros::c::millis() - time <= 750)
	{
		intake1.move_velocity(-125);
		intake2.move_velocity(-125);
	}
	intake1.move_velocity(0);
	intake2.move_velocity(0);
}

void trayAdjust(void* param) {
	int trayPos = tray.get_position();
	while(tray.get_position() < trayPos + 550)
	{
		tray.move_velocity(100);
	}
	pros::delay(250);
	trayPos = tray.get_position();
	while(tray.get_position() > trayPos - 550)
	{
		tray.move_velocity(-100);
	}
	tray.move_velocity(0);
}

void trayTask(void* param) {
	//2475
	//1453
	tray.set_zero_position(tray.get_position());
	int trayPos = tray.get_position();
	while(tray.get_position() < trayPos + 2100)//2450
	{
		tray.move_velocity(200);//150
	}
	while(tray.get_position() < trayPos + 2475)//2650
	{
		tray.move_velocity(125);//100
		intake1.move_velocity(150);
		intake2.move_velocity(150);
	}
	tray.move_velocity(0);
	intake1.move_velocity(0);
	intake2.move_velocity(0);
	pros::Task outtake (outtakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outtake");
	pros::delay(125);
	pros::Task backward (backwardTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Backward");
}

void trayTaskOP(void* param) {
	//2475
	//1453
	tray.set_zero_position(tray.get_position());
	int trayPos = tray.get_position();
	while(tray.get_position() < trayPos + 1825)//1900
	{
		tray.move_velocity(200);//125
	}
	while(tray.get_position() < trayPos + 2700)//2650
	{
		tray.move_velocity(100);//75
		intake1.move_velocity(100);
		intake2.move_velocity(100);
	}
	tray.move_velocity(0);
	intake1.move_velocity(0);
	intake2.move_velocity(0);
	pros::Task outtake (outtakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outtake");
	pros::delay(250);
	pros::Task backward (backwardTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Backward");
}

void armTask(void* param) {
	arm.set_zero_position(arm.get_position());
	int armPos = arm.get_position();
	while(arm.get_position() < armPos + 980)
	{
		arm.move_velocity(200);
	}
	arm.move_velocity(0);
}

void intake(void* param) {
	pros::delay(runDelay);
	int time = pros::c::millis();
	while(pros::c::millis() - time <= runTime)
	{
		pros::lcd::set_text(1,std::to_string(tray.get_position()));
		intake1.move_velocity(runSpeed);
		intake2.move_velocity(runSpeed);
	}
	intake1.move_velocity(0);
	intake2.move_velocity(0);
}

void nestedIntake(void* param) {
	pros::delay(runDelay);
	int time = pros::c::millis();
	while(pros::c::millis() - time <= runTime)
	{
		pros::lcd::set_text(1,std::to_string(tray.get_position()));
		intake1.move_velocity(runSpeed);
		intake2.move_velocity(runSpeed);
	}
	intake1.move_velocity(0);
	intake2.move_velocity(0);
	pros::delay(nestedDelay);
	time = pros::c::millis();
	while(pros::c::millis() - time <= nestedTime)
	{
		intake1.move_velocity(runSpeed);
		intake2.move_velocity(runSpeed);
	}
	intake1.move_velocity(0);
	intake2.move_velocity(0);
}

void outtake(void* param) {
	pros::delay(runDelay);
	int time = pros::c::millis();
	while(pros::c::millis() - time <= runTime)
	{
		pros::lcd::set_text(1,std::to_string(tray.get_position()));
		intake1.move_velocity(-runSpeed);
		intake2.move_velocity(-runSpeed);
	}
	intake1.move_velocity(0);
	intake2.move_velocity(0);
}

void autonomous() {
	pros::lcd::set_text(1, "Auton!");
	std::cout << "auto";
	left_motor1.set_brake_mode(MOTOR_BRAKE_HOLD);
	left_motor2.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_motor1.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_motor2.set_brake_mode(MOTOR_BRAKE_HOLD);
	intake1.set_brake_mode(MOTOR_BRAKE_HOLD);
	intake2.set_brake_mode(MOTOR_BRAKE_HOLD);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_encoder_units(MOTOR_ENCODER_DEGREES);
	tray.set_zero_position(tray.get_position());
	if(autonMode == 0)
	{
		//L path: moves forward, turns 90°, moves forward again then back, turns 135° and goes to corner
		runTime = 1000;
		pros::Task deploy (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Deploy");
		pros::delay(1200);
		chassis->setMaxVelocity(150);
		chassis->moveDistance(0.04_m);
		chassis->moveDistance(-0.0325_m);
		runTime = 2150;
		pros::Task consume (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume");
		pros::delay(100);
		chassis->setMaxVelocity(150);
		chassis->moveDistance(1.15_m);
		chassis->setMaxVelocity(50);
		chassis->turnAngle((sideSelector)*-90_deg);
		runTime = 1800;
		runDelay = 200;
		pros::Task consumeMore (nestedIntake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume More");
		pros::c::delay(100);
		chassis->setMaxVelocity(125);
		chassis->moveDistance(1.3_m);
		chassis->setMaxVelocity(200);
		chassis->moveDistance(-1.07_m);
		chassis->setMaxVelocity(50);
		chassis->turnAngle((sideSelector)*-135_deg);
		runDelay = 700;
		runTime = 500;
		runSpeed = 3000;
		pros::Task outsome (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		chassis->setMaxVelocity(135);
		chassis->moveDistance(0.50_m);
		pros::c::delay(2000);
		pros::Task traySome (trayTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
		//Tray stack 3/4 rotation velocity 60 time 750ms
	}
	else if(autonMode == 1)
	{
		//Z path: Moves forward, moves diagonally, moves forward again, returns to corner
		runTime = 900;
		pros::Task deploy (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Deploy");
		pros::delay(1000);
		runTime = 9000;
		pros::Task consume (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume");
		pros::delay(100);
		chassis->setMaxVelocity(120);
		chassis->moveDistance(1.07_m);
		chassis->setMaxVelocity(50);
		chassis->turnAngle((sideSelector)*40_deg);
		chassis->setMaxVelocity(120);
		tray.set_zero_position(tray.get_position());
		pros::Task trayAdj (trayAdjust, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
		chassis->setMaxVelocity(180);
		chassis->moveDistance(-0.92_m);
		chassis->setMaxVelocity(100);
		chassis->turnAngle((sideSelector)*-40_deg);
		chassis->setMaxVelocity(175);
		chassis->moveDistance(1.07_m);
		chassis->setMaxVelocity(100);
		chassis->turnAngle((sideSelector)*135_deg);
		pros::Task traySome (trayTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
		runDelay = 500;
		runTime = 700;
		runSpeed = 50;
		pros::Task outsome (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		chassis->setMaxVelocity(150);
		chassis->moveDistance(1.0_m);
	}
	else if(autonMode == 2)
	{
		runTime = 1000;
		pros::Task deploy (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		pros::delay(1200);
		runTime = 10000;
		pros::Task insome (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Insome");
		chassis->setMaxVelocity(175);
		chassis->moveDistance(1.10_m);
		pros::delay(500);
		chassis->moveDistance(-0.15_m);
		chassis->setMaxVelocity(50);
		chassis->turnAngle((sideSelector)*-137_deg);//Measured is 135°
		chassis->setMaxVelocity(150);
		chassis->moveDistance(1.1_m);
		runDelay = 0;
		runTime = 800;
		runSpeed = 60;
		pros::Task outsome (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		tray.set_zero_position(tray.get_position());
		pros::Task traySome (trayTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
	}
	else if(autonMode == 3)
	{
		//profileRev.generatePath({Point{0_m, 0_m, 0_deg},Point{0.70_m, 0.58_m, 0_deg}},"A");
		//profileRev.setTarget("A");
		//profileRev.waitUntilSettled();
		pros::Task move (moveDistanceSmooth, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Move");
	}
	else if(autonMode == 4)
	{
		//Z path: Moves forward, moves diagonally, moves forward again, returns to corner
		chassis->setMaxVelocity(100);
		runTime = 900;
		pros::Task deploy (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Deploy");
		pros::delay(1000);
		runTime = 8000;
		pros::Task consume (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume");
		pros::delay(100);
		chassis->setMaxVelocity(175);
		chassis->moveDistance(1.13_m);
		chassis->moveDistance(-0.16_m);
		profile->setTarget("S", true);
		profile->waitUntilSettled();
		chassis->setMaxVelocity(115);
		chassis->moveDistance(1.3_m);
		chassis->setMaxVelocity(180);
		chassis->moveDistance(-0.72_m);
		runDelay = 500;
		runTime = 500;
		runSpeed = 75;
		pros::Task outsome (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		chassis->setMaxVelocity(75);
		chassis->turnAngle((sideSelector)*122_deg);
		pros::Task traySome (trayTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
		chassis->setMaxVelocity(125);
		chassis->moveDistance(0.45_m);
	}
	else if (autonMode == 5) {
		runTime = 700;
		pros::Task deploy (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Deploy");
		pros::delay(800);
		runTime = 6000;
		pros::Task consume (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume");
		pros::delay(100);
		chassis->setMaxVelocity(100);
		chassis->moveDistance(2.8_m);
		chassis->setMaxVelocity(75);
		chassis->turnAngle((sideSelector)*45_deg);
		chassis->setMaxVelocity(125);
		chassis->moveDistance(0.5_m);
		runDelay = 0;
		runTime = 200;
		runSpeed = 50;
		pros::Task insome (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "insome");
		pros::delay(250);
		//pros::Task outsome (outtake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outsome");
		pros::Task traySome (trayTaskOP, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "trayTask");
		pros::delay(1000);
		/*
		0.356 m back
		-135
		run intake
		forward 0.05m
		while(button.get_value() != 1)
		{
			tray.move_velocity(-200);
		}
		chassis->setMaxVelocity(75);
		chassis->moveDistance(-0.2_m);
		chassis->turnAngle((sideSelector)*135_deg);
		runTime = 500;
		pros::Task consume2 (intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Consume2");
		chassis->setMaxVelocity(150);
		chassis->moveDistance(0.7_m);
		pros::Task armsome (armTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Move");
		chassis->moveDistance(0.1_m);4
		*/
	}
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
	pros::lcd::set_text(1, "OP");
	std::cout << "op";
	intake1.set_brake_mode(MOTOR_BRAKE_HOLD);
	intake2.set_brake_mode(MOTOR_BRAKE_HOLD);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_brake_mode(MOTOR_BRAKE_HOLD);
	tray.set_encoder_units(MOTOR_ENCODER_DEGREES);
	tray.set_zero_position(tray.get_position());
	arm.set_zero_position(arm.get_position());
	//FILE* fileWrite = fopen("/usd/test.txt", "w");
	if(toggleControl)
	{
		while (true) {
			pros::lcd::set_text(1,std::to_string(arm.get_position()));
			std::cout << master.get_analog(ANALOG_LEFT_Y);
			left_motor1.move(master.get_analog(ANALOG_LEFT_Y));
			left_motor2.move(master.get_analog(ANALOG_LEFT_Y));
			right_motor1.move(master.get_analog(ANALOG_RIGHT_Y));
			right_motor2.move(master.get_analog(ANALOG_RIGHT_Y));
			if (master.get_digital(DIGITAL_R1) && switchy.get_value() != 1) {
				arm.move_velocity(200);
			}
			else if (master.get_digital(DIGITAL_R2)) {
				arm.move_velocity(-100);
			}
			else {
				arm.move_velocity(0);
			}
			if (master.get_digital(DIGITAL_L1)) {
				intake1.move_velocity(200);//Max rpm
				intake2.move_velocity(200);
			}
			else if (master.get_digital(DIGITAL_L2)) {
				intake1.move_velocity(-150);//outtake
				intake2.move_velocity(-150);
			}
			else {
				intake1.move_velocity(0);
				intake2.move_velocity(0);
			}
			if(button.get_value() != 1 && master.get_digital(DIGITAL_B)) {
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
			if(master.get_digital(DIGITAL_DOWN))
			{
				pros::Task outtake (outtakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outtake");
				pros::delay(250);
				pros::Task backward (backwardTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Backward");
			}
			if(master.get_digital(DIGITAL_UP))
			{
				pros::Task trayMove (trayTaskOP, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tray Move");
			}
			if(master.get_digital(DIGITAL_LEFT))
			{
				pros::Task armMove (armTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Move");
			}
			//pros::lcd::set_text(1, std::to_string(time));
			//double leftVelocity = (left_motor1.get_actual_velocity() + left_motor2.get_actual_velocity())/2;
			//double rightVelocity = (right_motor1.get_actual_velocity() + right_motor2.get_actual_velocity())/2;
			//double linearVelocity = (leftVelocity+rightVelocity)/2;
			//std::string output = std::to_string(leftVelocity) + " " + std::to_string(rightVelocity) + " " + std::to_string(linearVelocity);
			//fputs(output.c_str(), fileWrite);
			pros::delay(10);
		}
	}
	else {
		while (true) {
			pros::lcd::set_text(1,std::to_string(arm.get_position()));
			std::cout << master.get_analog(ANALOG_LEFT_Y);
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_RIGHT_X);
			int left = power + turn;
			int right = power - turn;
			left_motor1.move(left);
			left_motor2.move(left);
			right_motor1.move(right);
			right_motor2.move(right);
			if (master.get_digital(DIGITAL_R1)) {
				arm.move_velocity(200);
			}
			else if (master.get_digital(DIGITAL_R2)) {
				arm.move_velocity(-100);
			}
			else {
				arm.move_velocity(0);
			}
			if (master.get_digital(DIGITAL_L1)) {
				intake1.move_velocity(200);//Max rpm
				intake2.move_velocity(200);
			}
			else if (master.get_digital(DIGITAL_L2)) {
				intake1.move_velocity(-150);//outtake
				intake2.move_velocity(-150);
			}
			else {
				intake1.move_velocity(0);
				intake2.move_velocity(0);
			}
			if(button.get_value() != 1 && master.get_digital(DIGITAL_B)) {
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
			if(master.get_digital(DIGITAL_DOWN))
			{
				pros::Task outtake (outtakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Outtake");
				pros::delay(250);
				pros::Task backward (backwardTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Backward");
			}
			if(master.get_digital(DIGITAL_UP))
			{
				pros::Task trayMove (trayTaskOP, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tray Move");
			}
			if(master.get_digital(DIGITAL_LEFT))
			{
				pros::Task armMove (armTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Move");
			}
			//pros::lcd::set_text(1, std::to_string(time));
			//double leftVelocity = (left_motor1.get_actual_velocity() + left_motor2.get_actual_velocity())/2;
			//double rightVelocity = (right_motor1.get_actual_velocity() + right_motor2.get_actual_velocity())/2;
			//double linearVelocity = (leftVelocity+rightVelocity)/2;
			//std::string output = std::to_string(leftVelocity) + " " + std::to_string(rightVelocity) + " " + std::to_string(linearVelocity);
			//fputs(output.c_str(), fileWrite);
			pros::delay(10);
		}
	}
}
