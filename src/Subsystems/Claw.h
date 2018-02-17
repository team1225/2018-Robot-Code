/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef Claw_H
#define Claw_H

#include <Commands/Subsystem.h>

#include <DoubleSolenoid.h>

class Claw : public Subsystem {
private:
	DoubleSolenoid theClaw;
	DoubleSolenoid leftRam;
	DoubleSolenoid rightRam;
	void PushRam();
	void PullRam();

public:
	Claw(
			int clawForwardChannel, int clawReverseChannel,
			int leftRamForwardChannel, int leftRamReverseChannel,
			int rightRamForwardChannel, int rightRamReverseChannel
			);
	void Open();
	void Close();
	void Toggle();
	void Fire();
	bool GetPosition();
	void InitDefaultCommand();
};

#endif  // Claw_H
