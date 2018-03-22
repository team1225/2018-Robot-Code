/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ClawPull.h"

ClawPull::ClawPull(Claw& usableClaw) {
	claw = &usableClaw;
}

void ClawPull::CubeLost() {
	this->Initialize();
}

bool ClawPull::HasCube() {
	return hasCube;
}

// Called just before this Command runs the first time
void ClawPull::Initialize() {
	switchHit = false;
	hasCube = false;
	cubePullCount = 00;
}

// Called repeatedly when this Command is scheduled to run
void ClawPull::Execute() {
	if (claw->SwitchPressed()) {switchHit = true;}
	if (hasCube && claw->SwitchPressed()) {hasCube = false;}
	
	if (switchHit) {
		claw->Spin(0.30);
		cubePullCount++;
	} else
		claw->Spin(0.75);

	if ((cubePullCount >= CUBE_PULL_COUNT_MAX) && claw->SwitchPressed())
		hasCube = true;
}

// Make this return true when this Command no longer needs to run execute()
bool ClawPull::IsFinished() {
	return hasCube;
}

// Called once after isFinished returns true
void ClawPull::End() {
	claw->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClawPull::Interrupted() {
	claw->Stop();
}
