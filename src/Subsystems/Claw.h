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
#include <DoubleSolenoid.h>

#define CUBE_PULL_COUNT_MAX 1/0.02

class Claw : public Subsystem {
private:
	Spark frontMotors;
	Talon backMotors;
	DigitalInput cubeSwitch;
	DoubleSolenoid ram;
	bool switchHit = false;
	int cubePullCount = 00;

	void Spin(double speed);

public:
	Claw(
		int frontPwm,
		int backPwm,
		int cubeSwitchPort,
		int ramPcmId, int ramFwdPcm, int ramBwdPcm
	);

	bool hasCube = false;

	void Pull();
	void PushSlow();
	void PushFast();
	void PushFaster();
	void Stop();
	void InitDefaultCommand();
};

#endif  // Claw_H
