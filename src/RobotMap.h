/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* Define hardware locations */

#ifndef SRC_ROBOTMAP_H_
#define SRC_ROBOTMAP_H_

#define TALON_SRX_DRIVE_LEFT_CHANNEL 10
#define TALON_SRX_DRIVE_RIGHT_CHANNEL 11

#define LIFTER_FWD_CHANNEL 0
#define LIFTER_BWD_CHANNEL 1

#define CLAW_FWD_CHANNEL 2
#define CLAW_BWD_CHANNEL 3

#define LEFT_RAM_FWD_CHANNEL 4
#define LEFT_RAM_BWD_CHANNEL 5
#define RIGHT_RAM_FWD_CHANNEL 6
#define RIGHT_RAM_BWD_CHANNEL 7


/* Define global macros */

#define CLAW_OPEN false
#define CLAW_CLOSED true

#define LIFTER_UP true
#define LIFTER_DOWN false

#endif /* SRC_ROBOTMAP_H_ */