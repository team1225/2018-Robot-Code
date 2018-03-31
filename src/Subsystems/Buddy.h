/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Commands/Subsystem.h>
#include <Servo.h>

class Buddy : public Subsystem {
private:
	Servo leftServo;
	Servo rightServo;
	int leftStart, leftTarget;
	int rightStart, rightTarget;

public:
	Buddy(
		int leftPwm, int rightPwm,
		int leftStart, int leftTarget,
		int rightStart, int rightTarget
	);

	void Go();
	void Return();
	void Release();
	void InitDefaultCommand() override;
};
