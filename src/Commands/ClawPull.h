/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Commands/Command.h>
#include "../Subsystems/Claw.h"

#define CUBE_PULL_COUNT_MAX 1/0.02

class ClawPull: public frc::Command {
private:
	bool switchHit = false;
	int cubePullCount = 00;
	bool hasCube = false;
	Claw* claw;
public:
	ClawPull(Claw& usableClaw);
	void CubeLost();
	bool HasCube();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};
