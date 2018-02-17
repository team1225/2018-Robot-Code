/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef Lifter_H
#define Lifter_H

#include <Commands/Subsystem.h>

#include <DoubleSolenoid.h>

class Lifter : public Subsystem {
private:
	DoubleSolenoid theLifter;
	void SolonoidInit();

public:
	Lifter(int forwardChannel, int reverseChannel);
	void Lift();
	void Drop();
	void Toggle();
	bool GetPosition();
	void InitDefaultCommand();
};

#endif  // Lifter_H
