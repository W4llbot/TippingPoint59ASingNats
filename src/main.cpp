#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Motor Init
	Motor FL(FLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BLU(BLUPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BLD(BLDPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BRU(BRUPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BRD(BRDPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor arm(armPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor intake(intakePort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);

	arm.tare_position();

	// Pneumatic init
	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

	// Mech tasks
	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	Task tiltControlTask(tiltControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tilt Control Task");
	Task intakeControlTask(intakeControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Control Task");

	// Base tasks
	Task sensorsTask(Sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensor Task");
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
void autonomous() {
	double start = millis();
	setOffset(-79.5);
	baseTurn(-79.5);
	delay(100);
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");
	std::vector<Node> testPath = {Node(0, 0), Node(0, 72)};
	std::vector<Node> moveTurnPath = {Node(0, 0), Node(0, 24), Node(48, 48)};
	std::vector<Node> reverseMoveTurnPath = {Node(48, 48), Node(24, 24), Node(0, 0)};
	std::vector<Node> straightPath = {Node(0, 0), Node(0, -24), Node(24, -36)};

	setMaxRPMV(500);
	baseMove(-5);
	waitPP(700);

	delay(200);

  setArmClampState(false);
	baseMove(5);
	waitPP(700);

	enableBase(true, false);
	baseTurn(22);
	waitTurn(1000);


	setMaxRPMV(500);
	std::vector<Node> initEdgeTurn = {position, Node(14, 58)};
	double smooth = 0.75;
	basePP(initEdgeTurn, 1-smooth, smooth, 20);

	// waitArmClamp(15000);
	// delay(50);
	waitPP(2000);
	setArmPos(2);
	setIntake(127);

	// std::vector<Node> moveToPlatform1 = {position, Node(40, 93)};
	// basePP(moveToPlatform1, 1-smooth, smooth, 12);
	// waitPP(2000);

	// setMaxRPMV(300);
	std::vector<Node> moveToRings1 = {position, Node(23, 76), Node(58, 81)};
	basePP(moveToRings1, 1-smooth, smooth, 10);
	waitPP(2000);

	// delay(1000);

  enableBase(true, true);
  baseMove(-5);
  waitPP(700);

	baseTurn(-12);
	waitTurn(1000);

  delay(200);

	baseMove(15);
	waitPP(1000);

	setArmPos(1);
	delay(500);
	setArmClampState(false);
	delay(500);

  printf("\n1 goal in %.2f\n", millis() - start);

  baseMove(-15);
  waitPP(2000);

  setArmPos(0);
  // baseTurn(50, 57, 0.14, false);
  baseTurn(calcBaseTurn(48, 57, false));
  waitTurn(2000);
  setTiltState(false);
  delay(300);


	setCurvK(0.000000000000001);
	setArmClampState(false);
	baseMove(50, 57, false);
	waitPP(2000);

	setArmPos(1);
  std::vector<Node> disposeGoal = {position, Node(83, 14)};
	basePP(disposeGoal, 1-smooth, smooth, 10);
	waitPP(2000);
	setArmPos(0);
	delay(300);
  setArmClampState(false);
  delay(200);

	setCurvK(0.0000000000000004);

	printf("\ngoal dispoosed in %.2f\n", millis() - start);

	// delay(200);
  baseTurn(-90);
  waitTurn(1000);
	delay(300);

  setTiltState(false);
  baseMove(100, 22, true);
  waitPP(2000);
  // basePP(2000);

	baseMove(15);
	waitPP(1000);

	baseTurn(-2);
	waitTurn(1000);

	setArmClampState(false);
	setIntake(127);
	baseMove(83, 56, false);
	waitPP(2000);

	setArmPos(2);
  std::vector<Node> moveToGoal = {position, Node(71, 72), Node(59, 96)};
	basePP(moveToGoal, 1-smooth, smooth, 6);
	waitPP(3000);

	baseTurn(-7);
	waitTurn(1000);
	setArmPos(1);
	delay(300);
	setArmClampState(false);

	


  // 48, 57
  // (83, 108), 21

	// 83, 71, 59
	// 56 72 81

	// baseMove(-15);
	// waitPP(1000);

	// setArmPos(0);
	// baseTurn(46, 56);
	// waitTurn(1000);
  //
	// baseMove(46, 56);
	// waitPP(2000);
  //
	// setArmPos(2);
	// baseTurn(2);
	// waitTurn(1000);
  //
	// baseMove(33);
	// waitPP(2000);
  //
	// setArmPos(1);
	// delay(500);
  //
	// baseMove(4);
	// waitPP(2000);
  //
	// setArmClampState(false);
	// delay(500);
  //
	// baseMove(-15);
	// waitPP(1000);
  //
	// setTiltState(false);
  //
  //

	// delay(2000);
	//
	// baseTurn(0, 0);
	// waitTurn(10000000);
	//
	// baseMove(0, 0);
	// waitPP(100000);
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
	Motor FL(FLPort);
	Motor BLU(BLUPort);
	Motor BLD(BLDPort);
	Motor FR(FRPort);
	Motor BRU(BRUPort);
	Motor BRD(BRDPort);

	FL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BLU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BLD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Controller master(E_CONTROLLER_MASTER);

	int armPos = 0;
	bool armClampActive = false;
	bool tilterActive = false;
	bool tankDrive = true;
	while(true) {
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;

		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		}else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);

			left = power + turn;
			right = power - turn;
		}

		FL.move(left);
		BLU.move(left);
		BLD.move(left);
		FR.move(right);
		BRU.move(right);
		BRD.move(right);

		if(master.get_digital_new_press(DIGITAL_L1) && armPos < 2) setArmPos(++armPos);
		else if(master.get_digital_new_press(DIGITAL_L2) && armPos > 0) setArmPos(--armPos);

		if(master.get_digital_new_press(DIGITAL_R2)) toggleArmClampState();

		if(master.get_digital_new_press(DIGITAL_X)) toggleTiltState();

		setIntake(master.get_digital(DIGITAL_R1)*127);

		encdPrintTerminal();

		delay(5);
  }
	}
