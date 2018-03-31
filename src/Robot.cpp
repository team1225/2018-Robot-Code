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
#include "Subsystems/Buddy.h"

class Robot : public frc::TimedRobot {
public:
	/* Set up Hardware 
	 * WPI_TalonSRX(CAN Id) CRTE CAN Motor Controllers
	 * Faults Fault monitoring for WPI_TalonSRXs
	 * DifferntialDrive(Motor, Motor) Combined drive object for use w/ two wheels
	 * ADIS16448 IMU, used here as an advanced gyro (https://github.com/juchong/ADIS16448-RoboRIO-Driver)
	 * Claw(Front/Rear PWMs, DIO Port) Abstraction of the robot's Claw
	 * Lifter(PCM Id, Default position, PCM Channels) Abstraction of the robot's Lifter and Arm
	 * Buddy(left/right PWM, left/right Start Angle, left/right Target Angle) Abstraction of the buddy system
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
		CLAW_SWITCH_PORT,
		PCM_ARM, CLAW_RAM_FWD, CLAW_RAM_BWD
	};

	Lifter arm{
		PCM_ARM, Lifter::Position::kUp,
		ARM_FWD_CHANNEL, ARM_BWD_CHANNEL};
	Lifter lift{
		PCM_BODY, Lifter::Position::kDown,
		LIFT_FWD_CHANNEL, LIFT_BWD_CHANNEL
	};

	Buddy buddy{
		BUDDY_LEFT_PWM, BUDDY_RIGHT_PWM,
		BUDDY_LEFT_START, BUDDY_RIGHT_START,
		BUDDY_LEFT_TARGET, BUDDY_RIGHT_TARGET
	};

	Joystick joystick{0};

	SendableChooser<std::string> startingPosition;
	const std::string
		startLeft = "Starting on the left of the switch",
		startRight = "Starting  on the right of the switch";
	SendableChooser<std::string> targetPreference;
	const std::string
		prefNone = "Just drive, no cube",
		prefSwitch = "Target the switch, Scale is fallback",
		prefScale = "Target the scale, Switch is fallback";

	void DisabledInit() {
		robotDrive.ArcadeDrive(0, 0, false);
	}

	void DisabledPeriodic() {}

	void TeleopInit() {
		robotDrive.SetSafetyEnabled(true);
	}

	void TeleopPeriodic() {

		/* get gamepad stick values */
		double forwMod = 0.80;
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
		frc::SmartDashboard::PutNumber("Left Clicks",
			(leftDrive.GetSelectedSensorPosition(0))
		);
		frc::SmartDashboard::PutNumber("Right Clicks",
                        (rightDrive.GetSelectedSensorPosition(0))
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
		if (joystick.GetRawButton(10) && !joystickButton10DBounce) { // Start Button
			joystickButton10DBounce = true;
			arm.Lift();
			lift.Toggle();
		} else if (!joystick.GetRawButton(10)) {
			joystickButton10DBounce = false;
		}

		/* Pull/Eject cubes w/ the Claw */
		if (joystick.GetRawButton(6)) { // Right Bumper
			claw.Pull();
		} else
		if (joystick.GetRawButton(1)) { // X Button
			claw.PushFaster();
			frc::Wait(0.50);
		} else
		if (joystick.GetRawButton(7)) { // Left Trigger
			claw.PushSlow();
		} else
		if (joystick.GetRawButton(5)) { // Left Bumper
			claw.PushFast();
		} else {
			claw.Stop();
		}

		/* Drop buddy platforms */
		if (joystick.GetRawButton(9)) { // Back Button
			buddy.Go();
			buddyDroped = true;
		} else
		if (buddyDroped) {
			buddy.Release();
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

	void DriveStraight() {
		double motorDiff = abs(leftDrive.GetSelectedSensorPosition(0)
				- rightDrive.GetSelectedSensorPosition(0));
		motorDiff = motorDiff * (1/(4028*4));
		if (motorDiff > 0.20) { motorDiff = 0.20; }

		robotDrive.ArcadeDrive(AUTO_DRIVE_SPEED, motorDiff);
	}

	void AutonomousInit() {
		std::string targets = DriverStation::GetInstance().GetGameSpecificMessage();
		autoTimer = new Timer();

		if (targetPreference.GetSelected() == prefScale) {
			if (startingPosition.GetSelected() == startLeft) {
				if (targets[1] == 'L')
					autoTarget = AutoTargets::LeftScale;
				else if (targets[0] == 'L')
					autoTarget = AutoTargets::LeftSwitch;
			}
			else if (startingPosition.GetSelected() == startRight) {
				if (targets[1] == 'R')
					autoTarget = AutoTargets::RightScale;
				else if (targets[0] == 'R')
					autoTarget = AutoTargets::RightSwitch;
			}
		}

		else if (targetPreference.GetSelected() == prefSwitch) {
			if (startingPosition.GetSelected() == startLeft) {
				if (targets[0] == 'L')
					autoTarget = AutoTargets::LeftSwitch;
				else if (targets[1] == 'L')
					autoTarget = AutoTargets::LeftScale;
			}
			else if (startingPosition.GetSelected() == startRight) {
				if (targets[0] == 'R')
					autoTarget = AutoTargets::RightSwitch;
				else if (targets[1] == 'R')
					autoTarget = AutoTargets::RightScale;
			}
		}

		else if ((targetPreference.GetSelected() == prefScale) 
				&& (autoTarget == AutoTargets::None)) {
			autoTarget = AutoTargets::NoneScale;
		}

		autoTimer->Reset();
		leftDrive.SetSelectedSensorPosition(0, 0, 100);
		rightDrive.SetSelectedSensorPosition(0, 0, 100);

		autoTimer->Start();
	}

	void AutonomousPeriodic() {
		double curTime = autoTimer->Get();

		/* Drive for 3 Seconds */
		if (IsBetween(curTime, 0, 2.25)) {
			DriveStraight();
			claw.Stop();
			lift.Drop();
			arm.Lift();
		}

		/* Paths Diverge */
		switch (autoTarget) {
			case AutoTargets::LeftScale:
				if (IsBetween(curTime, 2.25, 6)) {
					DriveStraight();
					claw.Stop();
					lift.Lift();
				}
				if (IsBetween(curTime, 6, 6.5)) {
					robotDrive.ArcadeDrive(0, AUTO_TURN_SPEED);
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (IsBetween(curTime, 6.5, 7.5)) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop(); //claw.PushFaster();
					lift.Lift();
					arm.Lift();
				}
				if (IsBetween(curTime, 7.5, 8)) {
					robotDrive.ArcadeDrive(-AUTO_DRIVE_SPEED, 0);
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (curTime > 8) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
					//lift.Drop();
					//arm.Drop();
					autoTarget = AutoTargets::None;
				}
				break;
			case AutoTargets::RightScale:
				if (IsBetween(curTime, 2.25, 6)) {
					DriveStraight();
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (IsBetween(curTime, 6, 6.5)) {
					robotDrive.ArcadeDrive(0, -AUTO_TURN_SPEED);
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (IsBetween(curTime, 6.5, 7.5)) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop(); //claw.PushFaster();
					lift.Lift();
					arm.Lift();
				}
				if (IsBetween(curTime, 7.5, 8)) {
					robotDrive.ArcadeDrive(-AUTO_DRIVE_SPEED, 0);
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (curTime > 8) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
					//lift.Drop();
					//arm.Drop();
					autoTarget = AutoTargets::None;
				}
				break;
			case AutoTargets::NoneScale:
				if (IsBetween(curTime, 2.25, 6)) {
					DriveStraight();
					claw.Stop();
					lift.Lift();
					arm.Lift();
				}
				if (curTime > 6) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
					lift.Lift();
					arm.Lift();
					autoTarget = AutoTargets::None;
				}
				break;
			case AutoTargets::LeftSwitch:
				if (IsBetween(curTime, 2.25, 4)) {
					robotDrive.ArcadeDrive(AUTO_DRIVE_SPEED, AUTO_TURN_SPEED);
					claw.Stop();
				}
				if (IsBetween(curTime, 4, 5)) {
					robotDrive.ArcadeDrive(0, 0);
					claw.PushSlow();
				}
				if (curTime > 5) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
					lift.Lift();
					arm.Drop();
					autoTarget = AutoTargets::None;
				}
				break;
			case AutoTargets::RightSwitch:
				if (IsBetween(curTime, 2.25, 4)) {
					robotDrive.ArcadeDrive(AUTO_DRIVE_SPEED, -AUTO_TURN_SPEED);
					claw.Stop();
				}
				if (IsBetween(curTime, 4, 5)) {
					robotDrive.ArcadeDrive(0, 0);
					claw.PushSlow();
				}
				if (curTime > 5) {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
					autoTarget = AutoTargets::None;
				}
				break;
			case AutoTargets::None:
				if (IsBetween(curTime, 2.25, 4)) {
					DriveStraight();
					claw.Stop();
				}
				else {
					robotDrive.ArcadeDrive(0, 0);
					claw.Stop();
				}
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
		startingPosition.AddDefault(startLeft,startLeft );
		startingPosition.AddObject(startRight, startRight);
		frc::SmartDashboard::PutData("Starting Position", &startingPosition);
		// Target Preference
		targetPreference.AddDefault(prefScale, prefScale);
		targetPreference.AddObject(prefSwitch, prefSwitch);
		targetPreference.AddObject(prefNone, prefNone);
		frc::SmartDashboard::PutData("Target Preference", &targetPreference);

		// Calibrate Gyro
		imu.Calibrate();
	}

	void TestInit() {}
	void TestPeriodic() {
		if (joystick.GetRawButton(9)) { // Back Button
			buddy.Return();
			buddyDroped = false;
		}
	}

private:
	bool joystickButton4DBounce = false;
	bool joystickButton10DBounce = false;
	bool buddyDroped = false;

	frc::Timer* autoTimer;
	AutoTargets autoTarget = AutoTargets::None;
};

START_ROBOT_CLASS(Robot)
