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
	Subsystem("ExampleSubsystem"),
	frontMotors {frontPwm},
	backMotors {backPwm},
	cubeSwitch {cubeSwitchPort}
{}

void Claw::Push() {
	frontMotors.Set(-1.00);
	backMotors.Set(-1.00);
}

void Claw::Pull() {
	if (!cubeSwitch.Get()) { switchHit = true; }
	if (hasCube) {
		if (cubeSwitch.Get()) { hasCube = false; }
	} else
	if (switchHit) {
		frontMotors.Set(0.30);
		backMotors.Set(0.30);
		cubePullCount++;
	}
	else {
		frontMotors.Set(-0.75);
		backMotors.Set(-0.75);
	}

	if ((cubePullCount >= CUBE_PULL_COUNT_MAX) && !cubeSwitch.Get()) {
		hasCube = true;
	}
}

void Claw::Stop() {
	frontMotors.Set(0.00);
	backMotors.Set(0.00);
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
