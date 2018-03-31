/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Claw.h"

#include <Talon.h>
#include <Spark.h>
#include <DriverStation.h>
#include <DoubleSolenoid.h>
#include <Timer.h>

Claw::Claw(
	int frontPwm,
	int backPwm,
	int cubeSwitchPort,
	int ramPcmId, int ramFwdPcm, int ramBwdPcm
) : 
	Subsystem("ExampleSubsystem"),
	frontMotors {frontPwm},
	backMotors {backPwm},
	cubeSwitch {cubeSwitchPort},
	ram {ramPcmId, ramFwdPcm, ramBwdPcm}
{
	ram.Set(DoubleSolenoid::kReverse);
}

void Claw::Spin(double speed) {
	frontMotors.Set(speed);
	backMotors.Set(speed);
}

void Claw::PushSlow() {
	this->Spin(-0.50);
}
void Claw::PushFast() {
	this->Spin(-0.85);
}
void Claw::PushFaster() {
	this->Spin(0.00); // Coast
	ram.Set(DoubleSolenoid::kForward);
}

void Claw::Pull() {
	if (!cubeSwitch.Get()) { switchHit = true; }
	if (hasCube) {
		if (cubeSwitch.Get()) { hasCube = false; }
	} else
	if (switchHit) {
		this->Spin(0.30);
		cubePullCount++;
	}
	else {
		this->Spin(0.75);
	}

	if ((cubePullCount >= CUBE_PULL_COUNT_MAX) && !cubeSwitch.Get()) {
		hasCube = true;
	}
}

void Claw::Stop() {
	this->Spin(0.20);
	ram.Set(DoubleSolenoid::kReverse);
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
