#ifndef Lifter_H
#define Lifter_H

#include <Commands/Subsystem.h>

#include <DoubleSolenoid.h>

class Lifter : public Subsystem {
private:
	DoubleSolenoid * theLifter;
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
