#include "Lifter.h"
#include "../RobotMap.h"

#include <DoubleSolenoid.h>
#include <Timer.h>

Lifter::Lifter(int forwardChannel, int reverseChannel) : Subsystem("ExampleSubsystem") {
	theLifter = new DoubleSolenoid(forwardChannel, reverseChannel);
	this->SolonoidInit();
}

void Lifter::SolonoidInit() {
	theLifter->Set(DoubleSolenoid::kReverse);
	theLifter->Set(DoubleSolenoid::kForward);
	theLifter->Set(DoubleSolenoid::kOff);
}

void Lifter::Lift() {
	theLifter->Set(DoubleSolenoid::kForward);
}

void Lifter::Drop() {
	theLifter->Set(DoubleSolenoid::kOff);
}

void Lifter::Toggle() {
	if (this->GetPosition() == LIFTER_DOWN) {
		this->Lift();
	}
	else if (this->GetPosition() == LIFTER_UP) {
		this->Drop();
	}
	else {
		this->Drop();
	}
}

bool Lifter::GetPosition() {
	if (theLifter->Get() == DoubleSolenoid::kForward) {
			return LIFTER_UP;
		}
		else if (theLifter->Get() == DoubleSolenoid::kOff) {
			return LIFTER_DOWN;
		}
		else {
			theLifter->Set(DoubleSolenoid::kOff);
			return LIFTER_DOWN;
		}
}

void Lifter::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.