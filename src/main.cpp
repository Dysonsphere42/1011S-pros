#include "main.h"
#include "lemlib/api.hpp"

//initalize devices
pros::Motor rightmai (3, MOTOR_GEARSET_6, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightmbi (6, MOTOR_GEARSET_6, false, MOTOR_ENCODER_DEGREES);
pros::Motor leftmai (2, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftmbi (4, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::Motor launchermi (7, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor intakemi (11, MOTOR_GEARSET_6, true, MOTOR_ENCODER_DEGREES);
pros::IMU inertial_sensor(9);
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "hello there!");
	
}

void disabled() {}

void competition_initialize() {

}

void autonomous(){
	pros::Motor_Group rightmu ({3, 6});
	pros::Motor_Group leftmu ({2, 4});
	leftmu.move_relative(2000, 600);
	rightmu.move_relative(2000, 600);
	pros::delay(1000);
	rightmu.move_relative(-100, 600);
	leftmu.move_relative(-100, 600);
	pros::delay(500);
	rightmu.move_relative(100, 600);
	leftmu.move_relative(100, 600);
}
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
