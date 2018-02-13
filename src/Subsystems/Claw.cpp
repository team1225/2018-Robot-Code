#include "Claw.h"
#include "../RobotMap.h"

#include <DoubleSolenoid.h>
#include <Timer.h>

Claw::Claw(
		int clawForwardChannel, int clawReverseChannel,
		int leftRamForwardChannel, int leftRamReverseChannel,
		int rightRamForwardChannel, int rightRamReverseChannel
		): Subsystem("ExampleSubsystem") {
	theClaw = new DoubleSolenoid(clawForwardChannel, clawReverseChannel);
	leftRam = new DoubleSolenoid(leftRamForwardChannel, leftRamReverseChannel);
	rightRam = new DoubleSolenoid(rightRamForwardChannel, rightRamReverseChannel);
	this->Open();
	this->PullRam();
}

void Claw::Open() {
	theClaw->Set(DoubleSolenoid::kReverse);
}

void Claw::Close() {
	theClaw->Set(DoubleSolenoid::kForward);
}

void Claw::Toggle() {
	if (this->GetPosition() == CLAW_CLOSED) {
		this->Open();
	}
	else if (this->GetPosition() == CLAW_OPEN) {
		this->Close();
	}
	else {
		this->Open();
	}
}

void Claw::PushRam() {
	leftRam->Set(DoubleSolenoid::kForward);
	rightRam->Set(DoubleSolenoid::kForward);
}

void Claw::PullRam() {
	leftRam->Set(DoubleSolenoid::kReverse);
	rightRam->Set(DoubleSolenoid::kReverse);
}

void Claw::Fire() {
	this->Open();
	this->PushRam();
	frc::Wait(2);
	this->PullRam();
}

bool Claw::GetPosition() {
	if (theClaw->Get() == DoubleSolenoid::kForward) {
			return CLAW_CLOSED;
		}
		else if (theClaw->Get() == DoubleSolenoid::kReverse) {
			return CLAW_OPEN;
		}
		else {
			theClaw->Set(DoubleSolenoid::kReverse);
			return CLAW_CLOSED;
		}
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
