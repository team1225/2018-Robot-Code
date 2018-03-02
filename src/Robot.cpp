/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <stack>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DriverStation.h>
#include <PIDController.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <DriverStation.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <Timer.h>
#include <AnalogGyro.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include "ADIS16448_IMU/ADIS16448_IMU.h"
#include "RobotMap.h"

#include "Subsystems/Claw.h"
#include "Subsystems/Lifter.h"

class Robot : public frc::TimedRobot {
public:
	/* Set up Hardware 
	 * WPI_TalonSRX(CAN Id) CRTE CAN Motor Controllers
	 * Faults Fault monitoring for WPI_TalonSRXs
	 * DifferntialDrive(Motor, Motor) Combined drive object for use w/ two wheels
	 * ADIS16448 IMU, used here as an advanced gyro (https://github.com/juchong/ADIS16448-RoboRIO-Driver)
	 * Claw(PCM Id, PCM Channels) Abstraction of the robot's Claw
	 * Lifter(PCM Id, Default position, PCM Channels) Abstraction of the robot's Lifter and Arm
	 * Joystick(USB Port) Joystick input from the Driver Station
	 * SendableChooser<type> Option menu on the Smart Dashboard and ShuffleBoard
	 * const std::string constant strings, used in the SendableChoosers
	 * */
	WPI_TalonSRX rightDrive{11};
	WPI_TalonSRX leftDrive{10};

	Faults _faults_L;
	Faults _faults_R;

	DifferentialDrive robotDrive{
		leftDrive,
		rightDrive
	};

	ADIS16448_IMU imu;

	Claw claw{
		PCM_ARM,
		CLAW_FWD_CHANNEL, CLAW_BWD_CHANNEL,
		LEFT_RAM_FWD_CHANNEL, LEFT_RAM_BWD_CHANNEL,
		RIGHT_RAM_FWD_CHANNEL, RIGHT_RAM_BWD_CHANNEL
	};
	Lifter arm{
		PCM_ARM, Lifter::Position::kUp,
		ARM_FWD_CHANNEL, ARM_BWD_CHANNEL};
	Lifter lift{
		PCM_BODY, Lifter::Position::kDown,
		LIFT_FWD_CHANNEL, LIFT_BWD_CHANNEL
	};

	Joystick joystick{0};

	SendableChooser<std::string> startingPosition;
	const std::string
		pos1 = "Position 1",
		pos2 = "Position 2",
		pos3 = "Position 3";
	SendableChooser<std::string> autoDropLeft;
	const std::string
		autoDropLeftYes = "Do Drop Cube on Left Switch",
		autoDropLeftNo = "Do NOT Drop Cube on Left Switch";
	SendableChooser<std::string> autoDropRight;
	const std::string
		autoDropRightYes = "Do Drop Cube on Right Switch",
		autoDropRightNo = "Do NOT Drop Cube on Right Switch";
	SendableChooser<std::string> autoDelayLeft;
	const std::string
		autoDelayLeftYes = "Delay 5 sec on Left Switch",
		autoDelayLeftNo = "Do Not Delay 5 Seconds on Left Switch";
	SendableChooser<std::string> autoDelayRight;
	const std::string
		autoDelayRightYes = "Delay 5 sec on Right Switch",
		autoDelayRightNo = "Do Not Delay 5 Seconds on Right Switch";

	void DisabledInit() {
		robotDrive.ArcadeDrive(0, 0, false);
	}

	void DisabledPeriodic() {}

	void TeleopInit() {
		robotDrive.SetSafetyEnabled(true);
	}

	void TeleopPeriodic() {

		/* get gamepad stick values */
		double forw = -0.40 * joystick.GetRawAxis(1); /* positive is forward */
		double turn = +0.40 * joystick.GetRawAxis(2); /* positive is right */

		/* deadband gamepad 10% */
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		frc::SmartDashboard::PutNumber("Volts",
			((leftDrive.GetMotorOutputVoltage() + rightDrive.GetMotorOutputVoltage()) / 2)
		);
		frc::SmartDashboard::PutNumber("Amps",\
			((leftDrive.GetOutputCurrent() + rightDrive.GetOutputCurrent()) / 2)
		);

		/* drive robot */
		robotDrive.ArcadeDrive(forw, turn, false);
		
		/* lift/lower arm */
		if (joystick.GetRawButton(1) && !joystickButton1DBounce) {
			joystickButton1DBounce = true;
			arm.Toggle();
		} else if (!joystick.GetRawButton(1)) {
			joystickButton1DBounce = false;
		}

		/* lift/drop lift */
		if (joystick.GetRawButton(4) && !joystickButton4DBounce) {
			joystickButton4DBounce = true;
			lift.Toggle();
		} else if (!joystick.GetRawButton(4)) {
			joystickButton4DBounce = false;
		}

		/* close/open grabber */
		if (joystick.GetRawButton(2) && !joystickButton2DBounce) {
			joystickButton2DBounce = true;
			claw.Toggle();
		} else if (!joystick.GetRawButton(2)) {
			joystickButton2DBounce = false;
		}

		/* fire grabber */
		if (joystick.GetRawButton(3) && !joystickButton3DBounce) {
			joystickButton3DBounce = true;
			claw.Fire();
		} else if (!joystick.GetRawButton(3)) {
			joystickButton3DBounce = false;
		}

		/* drive motor at least 25%, Talons will auto-detect if sensor is out of phase */
		leftDrive.GetFaults(_faults_L);
		rightDrive.GetFaults(_faults_R);

		if (_faults_L.SensorOutOfPhase) {
			std::cout << " Left drive sensor is out of phase\n";
		}
		if (_faults_R.SensorOutOfPhase) {
			std::cout << " Right drive sensor is out of phase\n";
		}

		// output gyro angle
		std::cout << "Gyro Angle:" << imu.GetAngle();
	}

	void AutoDrive(int targetPosition) {
		double currentPosition = ((leftDrive.GetSelectedSensorPosition(0) + rightDrive.GetSelectedSensorPosition(0))/2),
			period = 0.05, effortDrive = 0.30, kP_turn = 0.20, currentAngle,
			startingPosition = currentPosition;
		targetPosition = targetPosition + startingPosition;
		imu.Reset();
		while ((currentPosition >= targetPosition) && IsEnabled() && IsAutonomous()) {
			currentPosition = ((leftDrive.GetSelectedSensorPosition(0) +
						rightDrive.GetSelectedSensorPosition(0))/2);
			currentAngle = imu.GetAngle();

			robotDrive.ArcadeDrive(effortDrive, -(currentAngle * kP_turn));
			frc::Wait(period);
		}
		robotDrive.ArcadeDrive(0, 0);
	}

	void AutoTurn(int targetAngle) {
		double period = 0.05, currentAngle = imu.GetAngle(), effortTurn = 0.20;
		imu.Reset();
		while ((currentAngle >= targetAngle) && IsEnabled() && IsAutonomous()) {
			currentAngle = imu.GetAngle();
			robotDrive.ArcadeDrive(0, effortTurn);
			frc::Wait(period);
		}
		robotDrive.ArcadeDrive(0, 0);
	}

	void AutonomousInit() {
		// Disabling Safety
		robotDrive.SetSafetyEnabled(false);

		while (!autoActions.empty()) { autoActions.pop(); }

		// Collect Options
		std::string targets = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		SwitchTargetPos switchTarget;
		if ((targets[0] == 'L') || (targets[0] == 'l')) {
			switchTarget = SwitchTargetPos::Left;
		}
		else { //if ((targets[0] == 'R') || (targets[0] == 'r')) {
			switchTarget = SwitchTargetPos::Right;
		}
		std::string optionStart = startingPosition.GetSelected();
		std::string optionDropLeft = autoDropLeft.GetSelected();
		std::string optionDropRight = autoDropRight.GetSelected();
		std::string optionDelayLeft = autoDelayLeft.GetSelected();
		std::string optionDelayRight = autoDelayRight.GetSelected();
		// Output to console, for debugging
		if (switchTarget == SwitchTargetPos::Left){
			std::cout << "Switch target is Left from " << targets << "/n";
		}
		else {
			std::cout << "Switch target is Right from " << targets << "/n";
		}
		std::cout << "Starting from " << optionStart << "\n";
		std::cout << "Power Cube Drop Left? " << optionDropLeft << "\n";
		std::cout << "Power Cube Drop Right? " << optionDropRight << "\n";
		std::cout << "Delay Option Left: " << optionDelayLeft << "\n";
		std::cout << "Delay Option Right: " << optionDelayRight << "\n";

		if ((optionDelayLeft == autoDelayLeftYes) &&
				(switchTarget == SwitchTargetPos::Left)) {
			frc::Wait(5);
		}
		if ((optionDelayRight == autoDelayRightYes) &&
				(switchTarget == SwitchTargetPos::Right)) {
			frc::Wait(5);
		}

		/*
		 * Setting actions for the Autonomous loop
		 * parsing is based on starting position,
		 * 		switch's target side, and a per-side drop setting.
		 * actions are stacked onto the stack autoActions in reverse as
		 * 		to be popped in the right order
		 */

		if (optionStart == pos1) {
			if ((switchTarget == SwitchTargetPos::Left)
					&& (optionDropLeft == autoDropLeftYes)) {
				std::cout << "Going from Pos1 to the Left Switch to drop, then back up\n";
				autoActions.push(AutoActionTags::Backup);
				autoActions.push(AutoActionTags::DropCube);
				autoActions.push(AutoActionTags::TurnLeft);
				autoActions.push(AutoActionTags::Drive14ft);
			}
			else {
				std::cout << "Going from Pos1, past the switch to the left.\n";
				autoActions.push(AutoActionTags::TurnRight);
				autoActions.push(AutoActionTags::Drive17ft);
			}
		}

		else if (optionStart == pos2) {
			if (switchTarget == SwitchTargetPos::Left) {
				if (optionDropLeft == autoDropLeftYes) {
					std::cout << "Going from Pos2 to the Left switch to drop.\n";
					autoActions.push(AutoActionTags::DropCube);
					autoActions.push(AutoActionTags::TurnRight);
					autoActions.push(AutoActionTags::Drive6ft);
					autoActions.push(AutoActionTags::TurnRight);
					autoActions.push(AutoActionTags::Drive8ft);
					autoActions.push(AutoActionTags::TurnLeft);
					autoActions.push(AutoActionTags::Drive6ft);
				}
				else if (optionDropLeft == autoDropLeftNo) {
					std::cout << "Going from Pos2 to the auto line, ignoring the Left Switch.\n";
					autoActions.push(AutoActionTags::Drive12ft);
				}
			}
			else if (switchTarget == SwitchTargetPos::Right) {
				if (optionDropLeft == autoDropLeftYes) {
					std::cout << "Going from Pos2 to the Right switch to drop.\n";
					autoActions.push(AutoActionTags::DropCube);
					autoActions.push(AutoActionTags::TurnLeft);
					autoActions.push(AutoActionTags::Drive6ft);
					autoActions.push(AutoActionTags::TurnLeft);
					autoActions.push(AutoActionTags::Drive8ft);
					autoActions.push(AutoActionTags::TurnRight);
					autoActions.push(AutoActionTags::Drive6ft);
				}
				else if (optionDropLeft == autoDropLeftNo) {
					std::cout << "Going from Pos2 to the auto line, ignoring the Right Switch.\n";
					autoActions.push(AutoActionTags::Drive12ft);
				}
			}
		}

		else if (optionStart == pos3) {
			if ((switchTarget == SwitchTargetPos::Right)
					&& (optionDropRight == autoDropRightYes)) {
				std::cout << "Going from Pos1 to the Right Switch to drop, then back up\n";
				autoActions.push(AutoActionTags::Backup);
				autoActions.push(AutoActionTags::DropCube);
				autoActions.push(AutoActionTags::TurnLeft);
				autoActions.push(AutoActionTags::Drive14ft);
			}
			else {
				std::cout << "Going from Pos1, past the switch to the right.\n";
				autoActions.push(AutoActionTags::TurnLeft);
				autoActions.push(AutoActionTags::Drive17ft);
			}
		}
	}
	
	void AutonomousPeriodic() {
		int action = autoActions.top();
		autoActions.pop();
		switch (action) {
		case AutoActionTags::Drive6ft:
			std::cout << "Driving 6 feet with PID\n";
			AutoDrive(DRIVE_BITS_TO_INCHES * 72);
			break;
		case AutoActionTags::Drive8ft:
			std::cout << "Driving 8 feet with PID\n";
			AutoDrive(DRIVE_BITS_TO_INCHES * 96);
			break;
		case AutoActionTags::Drive12ft:
			std::cout << "Driving 12 feet with PID\n";
			AutoDrive(DRIVE_BITS_TO_INCHES * 144);
			break;
		case AutoActionTags::Drive14ft:
			std::cout << "Driving 14 feet with PID\n";
			AutoDrive(DRIVE_BITS_TO_INCHES * 168);
			break;
		case AutoActionTags::Drive17ft:
			std::cout << "Driving 17 feet with PID\n";
			AutoDrive(DRIVE_BITS_TO_INCHES * 204);
			break;

		case AutoActionTags::TurnLeft:
			std::cout << "Turning left with PID\n";
			AutoTurn(TURN_LEFT);
			frc::Wait(1);
			break;
		case AutoActionTags::TurnRight:
			std::cout << "Turning right with PID\n";
			AutoTurn(TURN_RIGHT);
			break;

		case AutoActionTags::DropCube:
			std::cout << "Dropping the cube, FIRE!\n";
			claw.Fire();
			break;

		default:
			//llvm::outs() << "Invalid Autonomous action detected! Skipping...\n";
			break;
		}
	}

	void RobotInit() {

		/* Set motor inverts */
		leftDrive.SetInverted(false);
		rightDrive.SetInverted(false);

		// Sensor Phase
		leftDrive.SetSensorPhase(true);
		rightDrive.SetSensorPhase(true);

		// And Safety Expiration
		robotDrive.SetExpiration(0.05);

		/* Send auto options */
		// Starting Position
		startingPosition.AddDefault(pos1, pos1);
		startingPosition.AddObject(pos2, pos2);
		startingPosition.AddObject(pos3, pos3);
		frc::SmartDashboard::PutData("Starting Position", &startingPosition);
		// Whether or not to drop, left and right
		autoDropLeft.AddDefault(autoDropLeftYes, autoDropLeftYes);
		autoDropLeft.AddObject(autoDropLeftNo, autoDropLeftNo);
		frc::SmartDashboard::PutData("Auto Drop Left", &autoDropLeft);
		autoDropRight.AddDefault(autoDropRightYes, autoDropRightYes);
		autoDropRight.AddObject(autoDropRightNo,autoDropRightNo);
		frc::SmartDashboard::PutData("Auto Drop Right", &autoDropRight);
		// Whether or not to wait, left and right
		autoDelayLeft.AddDefault(autoDelayLeftNo, autoDelayLeftNo);
		autoDelayLeft.AddObject(autoDelayLeftYes, autoDelayLeftYes);
		frc::SmartDashboard::PutData("Auto Delay Left", &autoDelayLeft);
		autoDelayRight.AddDefault(autoDelayRightNo, autoDelayRightNo);
		autoDelayRight.AddObject(autoDelayRightYes, autoDelayRightYes);
		frc::SmartDashboard::PutData("Auto Delay Right", &autoDelayRight);

		// Calibrate Gyro
		imu.Calibrate();
	}

private:
	bool joystickButton1DBounce = false;
	bool joystickButton2DBounce = false;
	bool joystickButton3DBounce = false;
	bool joystickButton4DBounce = false;
	std::stack<AutoActionTags> autoActions;
};

START_ROBOT_CLASS(Robot)
