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

void progSkills(){
	while(powerLevelMain() < 6.0){
		delay(25);
	}
	clawSet(127);
	delay(200);
	clawSet(10);
	chainbarSet(-50);
	delay(700);
	chainbarSet(-10);
	closeGate(0);
	moveFourbar(0);
	drive.moveBeyond(-46.0, 2.0);
	driveSet(-10, -10);
	moveFourbar(1);
	closeGate(1);
	delay(100);
	turn.moveToUntil(0.0, 1.0, 1000);
	chainbarSet(40);
	delay(500);
	clawSet(-127);
	chainbarSet(-50);
	delay(500);
	chainbarSet(-10);
	clawSet(0);
	drive.moveTo(-3.0, 0.5);
	turn.moveTo(45.0, 1.0);
	encZero(0.0);
	drive.moveTo(23.0, 0.5);
	turn.moveTo(135.0, 1.0);
	drive.moveFor(-127,1500);
	driveSet(-10,-10);
	closeGate(0);
	moveFourbar(0);
	encZero(0.0);
	drive.moveFor(80,100);
	moveFourbar(1);
	closeGate(1);
	drive.moveFor(-127,500);
	drive.moveFor(0, 100);
	drive.moveWhile(80, 17.0, 1.0, 1500);
	driveSet(0, 0);
	delay(200);
	drive.moveFor(-30,1500);
	encZero(0.0);
	/////////////////////////////////////////
	imuZero();
	drive.moveToUntil(2.0, 0.2, 1000);
	turn.moveToUntil(45.0-135.0, 1.0, 3000);
	drive.moveFor(70, 500);
	drive.moveFor(20, 2000);
	encZero(0.0);
	drive.moveTo(-2.0, 0.5);
	turn.moveToUntil(45.0-135.0, 1.0, 1000);
	drive.moveTo(-21.0, 0.5);
	turn.moveToUntil(90.0-135.0, 1.0, 3000);
	encZero(0.0);
	drive.moveTo(48.0, 1.0);
	turn.moveToUntil(90.0-135.0, 1.0, 1000);
	drive.moveToUntil(78.0, 1.0, 4000);
	turn.moveToUntil(30.0-135.0, 1.0, 2000);
	drive.moveTo(66.0, 1.0);
	turn.moveToUntil(180.0-135.0, 1.0, 3000);
	encZero(0.0);
	drive.moveToUntil(4.0, 1.0, 1000);
	//turn.moveToUntil(135.1-135.0, 1.0, 1000);
	closeGate(0);
	moveFourbar(0);
	drive.moveBeyond(-20.0, 2.0);
	driveSet(-10, -10);
	moveFourbar(1);
	closeGate(1);
	delay(100);
	drive.moveTo(-12, 4.0);
	turn.moveTo(90.0-135.0, 1.0);
	drive.moveToUntil(-60.0, 4.0, 3000);
	drive.moveFor(-30, 2000);
	turn.moveToUntil(135.0-135.0, 3.0, 1000);
	drive.moveFor(-30, 500);
	closeGate(0);
	moveFourbar(0);
	drive.moveFor(127, 200);
	moveFourbar(1);
	closeGate(1);
}

int encOffset=292;
void auton(int stationary){

	// clawSet(127);
	// lift.moveTo(60, 10);
	// lift.stack(1);
	while(powerLevelMain() < 6.0){
		delay(25);
	}
	clawSet(127);
	chainbarSet(-50);
	delay(600);
	clawSet(15);
	chainbarSet(-10);
	if(stationary){
		drive.moveTo(3.0, 1.0);
		lift.moveBeyond(100.0, 20.0);
		clawSet(-127);
		delay(200);
		lift.moveToUntil(230.0, 20.0, 2000);
		chainbarSet(10);
		clawSet(0);
		turn.moveToUntil(-90.0, 5.0, 2000);
		drive.moveBeyond(-19.0, 2.0);
		turn.moveTo(-135.0,1.0);
		delay(20);
		imuZero();
		gyroZero(0.0);
		encZero(0.0);
		delay(500);
	}


	closeGate(0);
	//while(1);
	moveFourbar(0);
	drive.moveBeyond(-40.0, 2.0);
	fourbarSet(-127);
	delay(200);
	fourbarSet(-12);
	drive.moveBeyond(-50.0,2.0);
	moveFourbar(1);
	closeGate(1);
	delay(100);
	if(!stationary){
		chainbarSet(40);
		turn.moveToUntil(0.0, 3.0, 500);
		clawSet(-127);
		chainbarSet(-50);
		delay(500);
		chainbarSet(-10);
		clawSet(0);
	}

	//turn.moveToUntil(0.0,5.0,1000);
	lift.moveToUntil(230.0, 20.0, 2000);
	drive.moveToUntil(-30.0, 0.5, 2000);
	turn.moveToUntil(-85.0, 3.0, 1000);
	drive.moveFor(127, 2000);
	encZero(0.0);
	gyroZero(-90.0);
	drive.moveWhile(-80, -19.0, 1.0, 2000);
	drive.moveTo(-17.3, 1.0);
	brakeSet(50);
	clawSet(127);
	lift.moveBeyond(50, 10);
	brakeSet(10);
	clawSet(10);
	lift.stack(2,1);
	lift.stack(3,1);
	lift.stack(4,1);
	lift.stack(5,1);
	lift.stack(6, 1);
	if(stationary)
		lift.stack(6,1);
	lift.stack(7,0);

	brakeSet(-127);
	delay(200);
	drive.moveToUntil(-4.0, 1.0, 2000);
	turn.moveToUntil(0.0+5.0, 3.0, 2000);

	drive.moveToUntil(27.0, 1.0,2000);
	turn.moveToUntil(45.0+5.0, 3.0,2000);
	encZero(0.0);
	drive.moveToUntil(23.0, 1.0,2000);
	lift.moveToUntil(190, 20, 1000);
	chainbarSet(-10);
	turn.moveToUntil(135.0+10.0, 3.0,2000);
	delay(500);
	drive.moveFor(-127,1500);
	driveSet(-10,-10);
	delay(500);
	closeGate(0);
	moveFourbar(0);
	drive.moveFor(-127,200);
	drive.moveFor(127,200);
	moveFourbar(1);
	drive.moveFor(127,500);
	// drive.moveFor(0, 100);
	// drive.moveWhile(80, 17.0, 1.0, 1500);
	// driveSet(0, 0);
	// delay(200);
	// drive.moveFor(-30,1500);
	// encZero(0.0);
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
	// turn.moveToUntil(0.0,1.0,500);
	// driveTurn(0);
	// clawSet(-127);
	// chainbarSet(-50);
	// delay(500);
	// chainbarSet(-10);
	// clawSet(0);
	// drive.moveTo(10.0, 0.5);
	// drive.moveTo(0.0,0.5);
	// turn.moveToUntil(-90.0, 1.0,1000);
	auton(0);
}
