/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef Claw_H
#define Claw_H

#include <Commands/Subsystem.h>

#include <DigitalInput.h>
#include <Spark.h>
#include <Talon.h>

class Claw : public Subsystem {
private:
	Spark frontMotors;
	Talon backMotors;
	DigitalInput cubeSwitch;

public:
	Claw(
		int frontPwm,
		int backPwm,
		int cubeSwitchPort
	);

	bool SwitchPressed();
	void Spin(double speed);
	void Push();
	void Stop();
	void InitDefaultCommand();
};

#endif  // Claw_H
