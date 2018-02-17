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


/* Define global macros */

#define CLAW_OPEN false
#define CLAW_CLOSED true

#define LIFTER_UP true
#define LIFTER_DOWN false

/* Define Auto actions */

#define AUTO_DRIVE_6FT 00
#define AUTO_DRIVE_8FT 01
#define AUTO_DRIVE_12FT 02
#define AUTO_DRIVE_17FT 03

#define AUTO_TURN_LEFT 10
#define AUTO_TURN_RIGHT 11

#define AUTO_DROP_CUBE 20

/* Define Constraints */

#define PID_DRIVE_KF 0.2
#define PID_DRIVE_KP 0.2
#define PID_DRIVE_KI 0
#define PID_DRIVE_KD 0

#define PID_POSITION_6FT 3000
#define PID_POSITION_8FT 3000
#define PID_POSITION_12FT 3000
#define PID_POSITION_14FT 3000
#define PID_POSITION_17FT 3000

#define PID_TURN_KF 0.2
#define PID_TURN_KP 0.2
#define PID_TURN_KI 0
#define PID_TURN_KD 0
#define PID_TURN_LEFT -270
#define PID_TURN_RIGHT 90

#define PID_ALLOWABLE_ERROR 400

#endif /* SRC_ROBOTMAP_H_ */
