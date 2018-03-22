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

Claw::Claw(
	int frontPwm,
	int backPwm,
	int cubeSwitchPort
) : 
	Subsystem("Claw"),
	frontMotors {frontPwm},
	backMotors {backPwm},
	cubeSwitch {cubeSwitchPort}
{}

bool Claw::SwitchPressed() {
	return !cubeSwitch.Get();
}

void Claw::Spin(double speed) {
	frontMotors.Set(speed);
	backMotors.Set(speed);
}

void Claw::Push() {
	this->Spin(-1.00);
}

void Claw::Stop() {
	this->Spin(0.00);
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}
