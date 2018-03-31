/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lifter.h"

#include <DoubleSolenoid.h>
#include <Timer.h>

Lifter::Lifter(int pcmId, Lifter::Position defaultPosition,
		int forwardChannel, int reverseChannel) :
	Subsystem("Lifter"),
	theLifter{pcmId, forwardChannel, reverseChannel}
{
	if (defaultPosition == Lifter::Position::kUp)
		this->Lift();
	else
		this->Drop();
}

void Lifter::Lift() {
	theLifter.Set(DoubleSolenoid::kForward);
}

void Lifter::Drop() {
	theLifter.Set(DoubleSolenoid::kReverse);
}

void Lifter::Toggle() {
	if (this->GetPosition() == Lifter::Position::kDown) {
		this->Lift();
	}
	else if (this->GetPosition() == Lifter::Position::kUp) {
		this->Drop();
	}
	else {
		this->Drop();
	}
}

Lifter::Position Lifter::GetPosition() {
	if (theLifter.Get() == DoubleSolenoid::kForward) {
			return Lifter::Position::kUp;
		}
		else if (theLifter.Get() == DoubleSolenoid::kReverse) {
			return Lifter::Position::kDown;
		}
		else {
			theLifter.Set(DoubleSolenoid::kReverse);
			return Lifter::Position::kDown;
		}
}

void Lifter::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
