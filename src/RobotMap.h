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

#define CLAW_FRONT_PWM 1
#define CLAW_REAR_PWM 0
#define CLAW_SWITCH_PORT 0
#define CLAW_RAM_FWD 2
#define CLAW_RAM_BWD 3

#define PCM_ARM 05
#define PCM_BODY 06

#define ARM_FWD_CHANNEL 0 // PCM_ARM
#define ARM_BWD_CHANNEL 1

#define LIFT_FWD_CHANNEL 0 // PCM_BODY
#define LIFT_BWD_CHANNEL 1

/* Buddy System */

#define BUDDY_LEFT_PWM 8
#define BUDDY_RIGHT_PWM 9

#define BUDDY_LEFT_START 0
#define BUDDY_LEFT_TARGET 90

#define BUDDY_RIGHT_START 0
#define BUDDY_RIGHT_TARGET 90

/* Auto Stuff */
enum AutoTargets {
	LeftSwitch, RightSwitch,
	LeftScale, RightScale,
	NoneScale, None
};
#define AUTO_DRIVE_SPEED 0.75
#define AUTO_TURN_SPEED 0.70
bool IsBetween(float number, double lower, double upper) {
	if ((number > lower) && (number <= upper))
		return true;
	else
		return false;
}

/* Define Modifiers */

#define DRIVE_BITS_TO_INCHES (4096 / (6*3.1415))

#endif /* SRC_ROBOTMAP_H_ */
