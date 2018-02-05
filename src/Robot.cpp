/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <Timer.h>
#include <AnalogGyro.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include "RobotMap.h"

#include "Subsystems/Claw.h"
#include "Subsystems/Lifter.h"

class Robot : public frc::TimedRobot {
public:
	/* Set up Hardware 
	 * WPI_TalonSRX(CAN Id) CRTE CAN Motor Controllers
	 * Faults Fault monitoring for WPI_TalonSRXs
	 * DifferntialDrive(Motor, Motor) Combined drive object for use w/ two wheels
	 * AnalogGyro(Location) Device to track the current rotation
	 * Claw(PCM Channels) Abstraction of the robot's Claw
	 * Lifter(PCM Channels) Abstraction of the robot's Lifter arm
	 * Joystick Joystick input from the Driver Station
	 * SendableChooser Option menu on the Smart Dashboard and ShuffleBoard
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

	AnalogGyro gyro{1};

	Claw claw{
		CLAW_FWD_CHANNEL, CLAW_BWD_CHANNEL,
		LEFT_RAM_FWD_CHANNEL, LEFT_RAM_BWD_CHANNEL,
		RIGHT_RAM_FWD_CHANNEL, RIGHT_RAM_BWD_CHANNEL
	};
	Lifter lifter{LIFTER_FWD_CHANNEL, LIFTER_BWD_CHANNEL};

	Joystick joystick{0};

	SendableChooser<std::string> startingPosition;
	const std::string
		pos1 = "Position 1",
		pos2 = "Position 2",
		pos3 = "Position 3";
	SendableChooser<std::string> autoDrop;
	const std::string
		autoDropYes = "Do Drop Cube",
		autoDropNo = "Do NOT Drop Cube";
	SendableChooser<std::string> autoDelay;
	const std::string
		autoDelayYes = "Delay 5 sec",
		autoDelayNo = "Do Not Delay 5 Seconds";

	void TeleopPeriodic() {

		/* get gamepad stick values */
		double forw = -joystick.GetRawAxis(1); /* positive is forward */
		double turn = +joystick.GetRawAxis(0); /* positive is right */

		/* deadband gamepad 10% */
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		frc::SmartDashboard::PutNumber("Volts",
			((leftDrive.GetMotorOutputVoltage() + rightDrive.GetMotorOutputVoltage()) / 2)
		);
		frc::SmartDashboard::PutNumber("Amps",
			((leftDrive.GetOutputCurrent() + rightDrive.GetOutputCurrent()) / 2)
		);
		/* drive robot */
		robotDrive.ArcadeDrive(forw, turn, false);
		
		/* lift/lower lifter */
		if (joystick.GetRawButton(1) && !joystickButton1DBounce) {
			joystickButton1DBounce = true;
			lifter.Toggle();
		} else if (!joystick.GetRawButton(1)) {
			joystickButton1DBounce = false;
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
		std::cout << "Gyro Angle:" << gyro.GetAngle();
	}

	void AutonomousInit() {
		// Collect Options
		std::string targets = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		char optionTarget = targets[0];
		std::string optionStart = startingPosition.GetSelected();
		std::string optionDrop = autoDrop.GetSelected();
		std::string optionDelay = autoDelay.GetSelected();
		// Output to console, for debugging
		std::cout << "Switch target is " << optionTarget << " from " << targets << "\n";
		std::cout << "Starting from " << optionStart << "\n";
		std::cout << "Power Cube Drop? " << optionDrop << "\n";
		std::cout << "Delay Option: " << optionDelay << "\n";
	}

	void AutonomousPeriodic() {
	}

	void RobotInit() {

		/* Set motor inverts */
		leftDrive.SetInverted(false);
		rightDrive.SetInverted(false);

		leftDrive.SetSensorPhase(true);
		rightDrive.SetSensorPhase(true);

		/* Send auto options */
		// Starting Position
		startingPosition.AddDefault(pos1, pos1);
		startingPosition.AddObject(pos2, pos2);
		startingPosition.AddObject(pos3, pos3);
		// Whether or not to drop
		autoDrop.AddDefault(autoDropYes, autoDropYes);
		autoDrop.AddObject(autoDropNo,autoDropNo);
		// Whether or not to wait
		autoDelay.AddDefault(autoDelayNo, autoDelayNo);
		autoDelay.AddObject(autoDelayYes, autoDelayYes);

		// Calibrate Gyro
		gyro.Calibrate();
	}

private:
	bool joystickButton1DBounce = false;
	bool joystickButton2DBounce = false;
	bool joystickButton3DBounce = false;
};

START_ROBOT_CLASS(Robot)
