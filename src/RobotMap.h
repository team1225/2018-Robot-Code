/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Team 1225. All Rights Reserved.                    */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* Define hardware locations */

#ifndef SRC_ROBOTMAP_H_
#define SRC_ROBOTMAP_H_

#define TALON_SRX_DRIVE_LEFT_CHANNEL 10
#define TALON_SRX_DRIVE_RIGHT_CHANNEL 11

#define PCM_ARM 05
#define PCM_BODY 06

#define ARM_FWD_CHANNEL 0 // PCM_ARM
#define ARM_BWD_CHANNEL 1

#define LIFT_FWD_CHANNEL 0 // PCM_BODY
#define LIFT_BWD_CHANNEL 1

#define CLAW_FWD_CHANNEL 2 // PCM_ARM
#define CLAW_BWD_CHANNEL 3

#define LEFT_RAM_FWD_CHANNEL 4 // PCM_ARM
#define LEFT_RAM_BWD_CHANNEL 5
#define RIGHT_RAM_FWD_CHANNEL 6
#define RIGHT_RAM_BWD_CHANNEL 7

enum SwitchTargetPos { Left, Right };

/* Define Auto actions */

enum AutoActionTags {
	Drive6ft,
	Drive8ft,
	Drive12ft,
	Drive14ft,
	Drive17ft,
	Backup,
	TurnLeft,
	TurnRight,
	DropCube
};

/* Define Modifiers */

#define DRIVE_BITS_TO_INCHES (4096 / (6*3.1415))

#define TURN_ANGLE_OVERSTOP 10
#define DRIVE_POSITION_OVERSTOP 100

#endif /* SRC_ROBOTMAP_H_ */
