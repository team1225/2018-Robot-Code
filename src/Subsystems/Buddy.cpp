/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Buddy.h"

#include <Servo.h>

Buddy::Buddy(
		int leftPwm, int rightPwm,
		int leftStart, int leftTarget,
		int rightStart, int rightTarget) :
	Subsystem("Buddy"),
	leftServo{leftPwm},
	rightServo{rightPwm}
{
	this->Release();
}

void Buddy::Release() {
	leftServo.SetOffline();
	rightServo.SetOffline();
}

void Buddy::Go() {
	leftServo.SetAngle(leftTarget);
	rightServo.SetAngle(rightTarget);
}

void Buddy::Return() {
	leftServo.SetAngle(leftStart);
	rightServo.SetAngle(rightStart);
}

void Buddy::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
