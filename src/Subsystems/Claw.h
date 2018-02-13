#ifndef Claw_H
#define Claw_H

#include <Commands/Subsystem.h>

#include <DoubleSolenoid.h>

class Claw : public Subsystem {
private:
	DoubleSolenoid * theClaw;
	DoubleSolenoid * leftRam;
	DoubleSolenoid * rightRam;
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
