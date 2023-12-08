#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

//initalize devices
pros::Motor rightmai (3, MOTOR_GEARSET_6, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightmbi (6, MOTOR_GEARSET_6, false, MOTOR_ENCODER_DEGREES);
pros::Motor leftmai (2, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftmbi (4, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::Motor launchermi (7, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor intakemi (11, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::IMU inertial_sensor(9);
//initialize motor groups
pros::Motor_Group leftmau({leftmai, leftmbi});
pros::Motor_Group rightmau({rightmai, rightmbi});

//
int autonomous_number = 0;

lemlib::OdomSensors_t sensors{
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&inertial_sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
   	8, // kP
   	30, // kD
   	1, // smallErrorRange
   	100, // smallErrorTimeout
   	3, // largeErrorRange
   	500, // largeErrorTimeout
   	5 // slew rate
};
 
	// turning PID
lemlib::ChassisController_t angularController {
   	4, // kP
   	40, // kD
   	1, // smallErrorRange
   	100, // smallErrorTimeout
   	3, // largeErrorRange
   	500, // largeErrorTimeout
   	0 // slew rate
};

	//initialize drive train
lemlib::Drivetrain_t drivetrain1{
	&leftmau,
	&rightmau,
	10,
	4,
	400
};
lemlib::Chassis chassis(drivetrain1, lateralController, angularController, sensors);

void initialize() {
	//calibrate
	chassis.calibrate();
	chassis.setPose(-40,-60,270);
	pros::lcd::initialize();
	pros::lcd::set_text(1, "hello there!");
	
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
void competition_initialize() {

}

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
void autonomous(){
	chassis.moveTo(-56, -60, 1000);
	chassis.turnTo(-69, -72, 1000);
	chassis.moveTo(-70, -70, 600);
	intakemi.move_relative(360, 100);
	launchermi.move_relative(16 * 360, 100);


	
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
	pros::lcd::set_text(2, "opcontrol");

	//declare devices
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor rightma(3);
	pros::Motor rightmb(6);
	pros::Motor leftma(2);
	pros::Motor leftmb(4);
	pros::Motor launcherm(7);
	pros::Motor intakem(11);

	//set motor groups
	pros::Motor_Group rightm ({rightma, rightmb});
	pros::Motor_Group leftm ({leftma, leftmb});

	//set motor brakes
	rightm.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	leftm.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	intakem.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	launcherm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	//declare static varibles
	double turnimportace = 1;


	while (true) {
		//update live varibles
		double turnval = master.get_analog(ANALOG_RIGHT_X);
		double speedval = master.get_analog(ANALOG_LEFT_Y);
		bool auton_button = master.get_digital_new_press(DIGITAL_B);

		//volt calculations
		double turnvolts = (turnval * 0.12);
		double speedvolts = (speedval * 0.12 * (1 - (std::abs(turnvolts)/12.0)));

		//spin motors	
		rightm.move_voltage((speedvolts - turnvolts) * 1000);
		leftm.move_voltage((speedvolts + turnvolts) * 1000);
		launcherm.move_voltage(12000 * master.get_digital(DIGITAL_R1));
		intakem.move_voltage(12000 * (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)));
		
		if (auton_button){
			autonomous();
			break;
		}
		


		pros::delay(20);
	
	}

}
