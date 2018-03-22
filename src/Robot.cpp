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
#include "Commands/ClawPull.h"

class Robot : public frc::TimedRobot {
public:
	/* Set up Hardware 
	 * WPI_TalonSRX(CAN Id) CRTE CAN Motor Controllers
	 * Faults Fault monitoring for WPI_TalonSRXs
	 * DifferntialDrive(Motor, Motor) Combined drive object for use w/ two wheels
	 * ADIS16448 IMU, used here as an advanced gyro (https://github.com/juchong/ADIS16448-RoboRIO-Driver)
	 * Claw(Front/Rear PWMs, DIO Port) Abstraction of the robot's Claw
	 * Lifter(PCM Id, Default position, PCM Channels) Abstraction of the robot's Lifter and Arm
	 * Joystick(USB Port) Joystick input from the Driver Station
	 * SendableChooser<type> Option menu on the Smart Dashboard and ShuffleBoard
	 * const std::string constant strings, used in the SendableChoosers
	 * */
	WPI_TalonSRX rightDrive{TALON_SRX_DRIVE_RIGHT_CHANNEL};
	WPI_TalonSRX leftDrive{TALON_SRX_DRIVE_LEFT_CHANNEL};

	Faults _faults_L;
	Faults _faults_R;

	DifferentialDrive robotDrive{
		leftDrive,
		rightDrive
	};

	ADIS16448_IMU imu;

	Claw claw{
		CLAW_FRONT_PWM,
		CLAW_REAR_PWM,
		CLAW_SWITCH_PORT
	};
	ClawPull pullCmd{
		claw
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
		startSwitchLeft = "Aiming at the switch's left",
		startSwitchRight = "Aiming at the switch's right",
		startSwitchOff  = "Aiming past the switch";

	void DisabledInit() {
		robotDrive.ArcadeDrive(0, 0, false);
	}

	void DisabledPeriodic() {}

	void TeleopInit() {
		robotDrive.SetSafetyEnabled(true);
	}

	void TeleopPeriodic() {

		/* get gamepad stick values */
		double forwMod = 0.70;
		double turnMod = 0.50;
		if (joystick.GetRawButton(12)) {
			 turnMod = 0.70;
		}
		double forw = -forwMod * joystick.GetRawAxis(1); /* positive is forward */
		double turn = +turnMod * joystick.GetRawAxis(2); /* positive is right */

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
		
		/* lift/lower arm */
		if (joystick.GetRawButton(4) && !joystickButton4DBounce
				&& (lift.GetPosition() == Lifter::Position::kDown)) { // Y Button
			joystickButton4DBounce = true;
			arm.Toggle();
		} else if (!joystick.GetRawButton(4)) {
			joystickButton4DBounce = false;
		}

		/* lift/drop lift */
		if (joystick.GetRawButton(9) && !joystickButton9DBounce) { // X Button
			joystickButton9DBounce = true;
			arm.Lift();
			lift.Toggle();
		} else if (!joystick.GetRawButton(9)) {
			joystickButton9DBounce = false;
		}

		/* Pull/Eject cubes w/ the Claw */
		if (joystick.GetRawButton(6) && !joystickButton6DBounce) { // Left Button
			joystickButton6DBounce = true;
			if (pullCmd.IsRunning()) {
				pullCmd.Cancel();
			}
			else {
				pullCmd.Start();
			}
		}
		else {
			joystickButton6DBounce = false;
			if (joystick.GetRawButton(10)) { // Left Trigger
				claw.Push();
			} else {
				claw.Stop();
			}
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

	void AutonomousInit() {
		robotDrive.SetSafetyEnabled(false);

		robotDrive.ArcadeDrive(0.45, 0);
		frc::Wait(5);
		robotDrive.ArcadeDrive(0, 0);

		std::string targets = DriverStation::GetInstance().GetGameSpecificMessage();

		if ((startingPosition.GetSelected() == startSwitchLeft) &&
				((targets[0] == 'L') || (targets[0] == 'l'))) {
			claw.Push();
			frc::Wait(0.5);
			claw.Stop();
		} else
		if ((startingPosition.GetSelected() == startSwitchRight) &&
				((targets[0] == 'R') || (targets[0] == 'r'))) {
			claw.Push();
			frc::Wait(0.5);
			claw.Stop();
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
		startingPosition.AddDefault(startSwitchLeft,startSwitchLeft );
		startingPosition.AddObject(startSwitchRight, startSwitchRight);
		startingPosition.AddObject(startSwitchOff, startSwitchOff);
		frc::SmartDashboard::PutData("Starting Position", &startingPosition);

		// Calibrate Gyro
		imu.Calibrate();
	}

private:
	bool joystickButton4DBounce = false;
	bool joystickButton9DBounce = false;
	bool joystickButton6DBounce = false;
	std::stack<AutoActionTags> autoActions;
};

START_ROBOT_CLASS(Robot)
