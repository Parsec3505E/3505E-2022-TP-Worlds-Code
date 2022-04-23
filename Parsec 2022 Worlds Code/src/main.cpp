#include "main.h"

// pros::task_t odometry = (pros::task_t)NULL;
// pros::task_t chassisControl = (pros::task_t)NULL;


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
void disabled() {
	//endAllTasks();
}

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


void autonomous() {

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().pros build
 * 
 * 
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

	highNeutralWinPoint();
	//leftSideAuton();
	


}


void opcontrol(){
	endAllTasks();
	//highNeutralWinPoint();
	//odomPrint();
	// Subsystems
	Drivetrain drive = Drivetrain();
	pros::Controller driver(pros::E_CONTROLLER_MASTER);
	Intake intake = Intake();
	Arm arm = Arm();
	//Primary primary_mogo = Primary();
	driver.print(2,2,"Hello");
	Stick stick = Stick();

	// intake.stop();
	// intake.resetEncoder();

	// arm.stop();
	// arm.resetEncoder();
	
	// stick.stop();
	// stick.resetEncoder();

	//initial var declarations
	int clamp_state = 0;
	int	pto_state = 0;
	bool intake_state = false;

	//pros::Controller driver(pros::E_CONTROLLER_MASTER);
	pros::Controller partner(pros::E_CONTROLLER_PARTNER);
	stick.setHold();
	arm.setCoast();
	
	// ###########################################################################

	// pros::Motor left_mtr(1);
	// pros::Motor right_mtr(2);

	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
	// 	int left = master.get_analog(ANALOG_LEFT_Y);
	// 	int right = master.get_analog(ANALOG_RIGHT_Y);

	// 	left_mtr = left;
	// 	right_mtr = right;

	// ###########################################################################

	while (true) {

		//Drive Control
		int power = driver.get_analog(ANALOG_RIGHT_Y);
		int turn = driver.get_analog(ANALOG_LEFT_X);

		int right = power - turn;
		int left = power + turn;

		drive.runRightDrive(right);
		drive.runLeftDrive(left);

		//Arm Control
		if (partner.get_digital(DIGITAL_R2)) {
			arm.runArmVelocity(90);
		}
		else if (partner.get_digital(DIGITAL_R1)) {
			arm.runArmVelocity(-90);
		}
		else {
			arm.stop();
		}

		//Clamp Control
		// bool piston_button = partner.get_digital_new_press(DIGITAL_B);
		// if(piston_button && clamp_state % 2 == 0) {
		// 	primary_mogo.triggerMogoClamp(true);
		// 	clamp_state++;
		// }
		// else if(piston_button && clamp_state % 2 != 0) {
		// 	primary_mogo.triggerMogoClamp(false);
		// 	clamp_state++;
		// }

		//Intake/Outtake Control
		if(partner.get_digital_new_press(DIGITAL_UP) && intake_state)
		{
			intake.intake(90);
			intake_state = true;
		}
		else if(partner.get_digital_new_press(DIGITAL_DOWN) && intake_state)
		{
			intake.outtake(90);
			intake_state = true;
		}
		else if(partner.get_digital_new_press(DIGITAL_LEFT))
		{
			intake.stop();
			intake_state = true;
		}
		else if (driver.get_digital(DIGITAL_R2)) {
			intake.intake(127);
			intake_state = false;
		}
		else if (driver.get_digital(DIGITAL_R1)) {
			intake.outtake(127);
			intake_state = false;
		}
		else if(intake_state == false)
		{
			intake.stop();
			intake_state = true;
		}

		//Spinner Control
		// if (driver.get_digital(DIGITAL_L2) || partner.get_digital(DIGITAL_A)) {
		// 	primary_mogo.spin(100);
		// }
		// else if (driver.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_Y)) {
		// 	primary_mogo.spin(-100);
		// }
		// else {
		// 	primary_mogo.stop();
		// }

		//Stick Control
		if (partner.get_digital(DIGITAL_L1)) {
			stick.runStick(-30);
		}
		else if (partner.get_digital(DIGITAL_L2)) {
			stick.runStick(30);
		}
		else {
			stick.stop();
		}

		//PTO Control
		bool pto_button = partner.get_digital_new_press(DIGITAL_X);
		if(pto_button && pto_state % 2 == 0) {
			intake.triggerPTOLock(true);
			pto_state++;
		}
		else if(pto_button && pto_state % 2 != 0) {
			intake.triggerPTOLock(false);
			pto_state++;
		}

		

		pros::delay(20);
	}
}
