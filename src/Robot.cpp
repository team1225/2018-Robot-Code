/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */

#include <iostream>
#include <string>

#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "ctre/Phoenix.h"
#include "Drive/DifferentialDrive.h"
#include "Joystick.h"


class Robot : public frc::TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonSRX * rightDrive = new WPI_TalonSRX(11);
	WPI_TalonSRX * leftDrive = new WPI_TalonSRX(10);
	WPI_TalonSRX * arms = new WPI_TalonSRX(20);

	Faults _faults_L;
	Faults _faults_R;
	Faults _faults_A;

	DifferentialDrive * robotDrive = new DifferentialDrive(
			*leftDrive,
			*rightDrive);

	DoubleSolenoid * grabber = new DoubleSolenoid(0, 1);

	Joystick * joystick = new Joystick(0);

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

		std::stringstream work;

		/* get gamepad stick values */
		double forw = -1 * joystick->GetRawAxis(1); /* positive is forward */
		double turn = +1 * joystick->GetRawAxis(0); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		frc::SmartDashboard::PutNumber("Clicks", leftDrive->GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutNumber("Volts", leftDrive->GetMotorOutputVoltage());
		frc::SmartDashboard::PutNumber("Amps", leftDrive->GetOutputCurrent());


		/* drive robot */
		robotDrive->ArcadeDrive(forw, turn, false);
		
		/* run arms */
		if (joystick->GetRawButton(6)) {
			arms->Set(1.00);
		} else if (joystick->GetRawButton(8)) {
			arms->Set(-1.00);
		} else {
			arms->Set(0);
		}

		/* close/open grabber */
		if (joystick->GetRawButton(6) && !joystickButton6DBounce) {
			joystickButton6DBounce = true;
			if (grabber->Get() == DoubleSolenoid::kOff) {
				grabber->Set(DoubleSolenoid::kReverse);
			} else if (grabber->Get() == DoubleSolenoid::kForward) {
				grabber->Set(DoubleSolenoid::kReverse);
			} else if (grabber->Get() == DoubleSolenoid::kReverse) {
				grabber->Set(DoubleSolenoid::kForward);
			}
		} else { joystickButton6DBounce = false; }

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront->GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront->GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = leftDrive->GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = rightDrive->GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* drive motor at least 25%, Talons will auto-detect if sensor is out of phase */
		leftDrive->GetFaults(_faults_L);
		rightDrive->GetFaults(_faults_R);
		arms->GetFaults(_faults_A);

		if (_faults_L.SensorOutOfPhase) {
			work << " L sensor is out of phase";
		}
		if (_faults_R.SensorOutOfPhase) {
			work << " R sensor is out of phase";
		}

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void AutonomousInit() {
		
	}
	
	
	/*{ demo Auto
		leftDrive->SetSafetyEnabled(false);
		// Configure _leftFront and _rightFront to stop when leaving thr range of +100 rotations
		leftDrive->ConfigForwardSoftLimitThreshold(+100*4096, 60000);
		leftDrive->ConfigForwardSoftLimitEnable(true, 60000);
		rightDrive->ConfigForwardSoftLimitThreshold(+100*4096, 60000);
		rightDrive->ConfigForwardSoftLimitEnable(true, 60000);
		
		//Drive until the motors stop
		leftDrive->Set(1.00);
		rightDrive->Set(1.00);
		while((leftDrive->Get() == 0.00) && (rightDrive->Get() == 0.00)) { frc::Wait(0.05); }
		
		// Rotate for 5 seconds
		leftDrive->Set(0.50);
		rightDrive->Set(0.50);
		frc::Wait(5.00);
		leftDrive->Set(0.00);
		rightDrive->Set(0.00);
	} */

	void AutonomousPeriodic() {
	}

	void RobotInit() {

		/* [3] Adjust inverts so all motor drive in the correction direction */
		leftDrive->SetInverted(false);
		rightDrive->SetInverted(false);
		arms->SetInverted(false);


		/* [4] adjust sensor phase so sensor moves
		 * positive when Talon LEDs are green */
		leftDrive->SetSensorPhase(true);
		rightDrive->SetSensorPhase(true);
		arms->SetSensorPhase(true);

		/* Set 2 second ramp up */
		//leftDrive->ConfigOpenloopRamp(.5,11000);
		//leftDrive->ConfigClosedloopRamp(.5, 1000);
		//rightDrive->ConfigOpenloopRamp(.5, 1000);
		//rightDrive->ConfigClosedloopRamp(.5, 1000);
		//arms->ConfigOpenloopRamp(.5, 1000);
		//arms->ConfigClosedloopRamp(.5, 1000);

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
	}

private:
	bool joystickButton6DBounce = false;
};

START_ROBOT_CLASS(Robot)
