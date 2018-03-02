/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Claw.h"

#include <DoubleSolenoid.h>
#include <Timer.h>

Claw::Claw(
	int pcmId,
	int clawForwardChannel, int clawReverseChannel,
	int leftRamForwardChannel, int leftRamReverseChannel,
	int rightRamForwardChannel, int rightRamReverseChannel
) :
	Subsystem("ExampleSubsystem"),
	theClaw {pcmId, clawForwardChannel, clawReverseChannel},
	leftRam {pcmId, leftRamForwardChannel, leftRamReverseChannel},
	rightRam {pcmId, rightRamForwardChannel, rightRamReverseChannel}
{
	this->Close();
	this->PullRam();
}

void Claw::Open() {
	theClaw.Set(DoubleSolenoid::kForward);
}

void Claw::Close() {
	theClaw.Set(DoubleSolenoid::kReverse);
}

void Claw::Toggle() {
	if (this->GetPosition() == Claw::Position::kClosed) {
		this->Open();
	}
	else if (this->GetPosition() == Claw::Position::kOpen) {
		this->Close();
	}
	else {
		this->Open();
	}
}

void Claw::PushRam() {
	leftRam.Set(DoubleSolenoid::kForward);
	rightRam.Set(DoubleSolenoid::kForward);
}

void Claw::PullRam() {
	leftRam.Set(DoubleSolenoid::kReverse);
	rightRam.Set(DoubleSolenoid::kReverse);
}

void Claw::Fire() {
	this->Open();
	this->PushRam();
	frc::Wait(2);
	this->PullRam();
}

Claw::Position Claw::GetPosition() {
	if (theClaw.Get() == DoubleSolenoid::kForward) {
		return Claw::Position::kOpen;
	}
	else if (theClaw.Get() == DoubleSolenoid::kReverse) {
		return Claw::Position::kClosed;
	}
	else {
		theClaw.Set(DoubleSolenoid::kReverse);
		return Claw::Position::kClosed;
	}
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
