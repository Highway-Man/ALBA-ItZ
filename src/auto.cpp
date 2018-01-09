/*
 * 	Copyright 2017 Jacob Knaup
 *
 * 	This file is part of the firmware for ALBA's Open Source In the Zone Robot and is built upon the PURDUE ROBOTICS OS (PROS).
 *
 * 	The firmware for ALBA's Open Source In the Zone Robot is free software: you can redistribute it and/or modify
 * 	it under the terms of the GNU General Public License as published by
 * 	the Free Software Foundation, either version 3 of the License, or
 * 	(at your option) any later version, while keeping with the terms of the license for PROS.
 *
 * 	The firmware for ALBA's Open Source In the Zone Robot is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 * 	You should have received a copy of the GNU General Public License
 * 	along with The firmware for ALBA's Open Source In the Zone Robot.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
* File for autonomous code.
*
* This file should contain the user autonomous() function and any functions related to it.
*
* Copyright (c) 2011-2014, Purdue University ACM SIG BOTS. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
********************************************************************************/

#include "main.h"
#include "subsystems.h"
#include "PID.hpp"

//deploy intake, raise lift & drive to fence, outtake
//only a framework; will need to be adjusted on actual field
void standardAuton(){
	moveLift(400, 20);
	moveClaw(0);
	delay(1000);
	moveLift(220, 10);
	moveClaw(1);
	moveLift(880, 10);
	moveClaw(0);
	moveLift(220, 10);
	moveClaw(1);
	moveLift(840, 10);
	moveClaw(0);
	moveLift(220, 10);
	moveClaw(1);
	moveLift(800, 10);
	moveClaw(0);
	moveLift(220, 0);
}

void scoreMobileGoal(){
	moveFourbar(0);
	driveSet(-40,-40);
	delay(3000);
	driveSet(0,0);
	moveFourbar(1);
	driveSet(40,40);
	delay(3000);
	driveSet(-40,40);
	delay(800);
	driveSet(40,40);
	delay(700);
	driveSet(-40,40);
	delay(1000);
	driveSet(-127,-127);
	delay(1500);
	driveSet(0,0);
	moveFourbar(0);
	driveSet(50,50);
	delay(300);
	driveSet(0,0);
	moveFourbar(1);
	driveSet(127,127);
	delay(300);
	driveSet(0,0);
	moveFourbar(0);
	driveSet(127,127);
	delay(600);
	moveFourbar(1);
	driveSet(0,0);

}

void defense(){
	driveSet(127,127);
	delay(2000);
	driveSet(-40,40);
	delay(900);
	driveSet(127,127);
	delay(2000);
	driveSet(0,0);
}

void driveStr8(){
	driveSet(127,127);
	delay(6000);
	driveSet(0,0);
	delay(6000);
	driveSet(127,127);
	delay(6000);
	driveSet(0,0);

}

/**
* Runs the user autonomous code.
*
* This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart the task, not re-start it from where it left off.
*
* Code running in the autonomous task cannot access information from the VEX Joystick. However, the autonomous function can be invoked from another task if a VEX Competition Switch is not available, and it can access joystick information if called in this way.
*
* The autonomous task may exit, unlike operatorControl() which should never exit. If it does so, the robot will await a switch to another mode or disable/enable cycle.
*/
void autonomous() {
	scoreMobileGoal();
	driveStr8();
}
