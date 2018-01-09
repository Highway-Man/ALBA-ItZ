#include "subsystems.h"
#include "main.h"

//functions to set our subsystem motors to control values. Some motors need to be reversed
void lDriveSet(short control) {
	motorSet(lDrive, control);
}
void rDriveSet(short control) {
	motorSet(rDrive, -control);
}
void driveForward(short control){
	lDriveSet(control);
	rDriveSet(control);
}
void driveTurn(short control){
	lDriveSet(control);
	rDriveSet(-control);
}
int lDriveGet(){
	return motorGet(lDrive);
}
int rDriveGet(){
	return -motorGet(rDrive);
}
void lDriveBrake(){
	motorSet(lDrive, -12);
}
void rDriveBrake(){
	motorSet(rDrive, -12);
}
void chainbarSet(int control) {
	motorSet(olArm, -control);
	motorSet(ilArm, control);
	motorSet(orArm, control);
	motorSet(irArm, -control);
}
void chainbarControl(int target) {
	static int last, command, deltaMax = 50;
	last = motorGet(ilArm);
	if (target - last > deltaMax)
		command = last + deltaMax;
	else if (target - last < -deltaMax)
		command = last - deltaMax;
	else
		command = target;
	chainbarSet(command);
}
void clawSet(int control) {
	motorSet(claw, control);
}
int clawPosition = 0;
void closeClaw(int close) {
	static int time = 0;
	static int last = 1;
	if (close != last)
		time = 0;
	if (close && time < 500)
		clawSet(127);
	else if (close)
		clawSet(15);
	else if (time < 500)
		clawSet(-127);
	else
		clawSet(-10);
	time += 25;
	last = close;
}
void fourbarSet(int control) {
	motorSet(lMogo, control);
	motorSet(rMogo, control);
}
void closeGate(int close){
	digitalWrite(3,!close);
	digitalWrite(4,!close);
}
void brakeSet(int control){
	motorSet(brake, control);
}

void prepChainbar() {
	chainbarSet(-50);
	delay(300);
	chainbarSet(-10);
	delay(50);
	encoderReset(armEnc);
}
struct PIDcont arm;
const int armHeightMax = 8;
void updateArmTarget() {
	if (arm.height == 0 && arm.bottom == 0)
		arm.target = -10;//-60
	else if(arm.height == 0 && arm.bottom == 1)
		arm.target = 65;
	else if (arm.height == 1)
		arm.target = 280;//840
	else if (arm.height == 2)
		arm.target = 265;//800
	else if (arm.height == 3)
		arm.target = 250;//775
	else if (arm.height == 4)
		arm.target = 240;//745
	else if (arm.height == 5)
		arm.target = 230;//700
	else if (arm.height == 6)
		arm.target = 220;//640
	else if (arm.height == 7)
		arm.target = 205;//600
	else if (arm.height == 8)
		arm.target = 200;//600
}

void checkStackRelease() {
	static int waitGoDown = 0;
	//890, 850, 810, 785, 765, 750, 733, 670,
	if (arm.error < 30 && arm.target > 100) {
		clawPosition = 0;
		if(waitGoDown>3){
			waitGoDown=0;
			if(arm.bottom == 0)
				arm.target = -10;
			else
				arm.target = 60;
		}
		waitGoDown++;
	}
}

//set both sides of drive
void driveSet(int left, int right){
	lDriveSet(left);
	rDriveSet(right);
}

//wait for lift to be within 10 ticks of target
void waitForLift(int target, int margin){
	while(abs(encoderGet(armEnc) - target) > margin)
		delay(20);
}

void positionController(){
	arm.kP = 1.5;//.86
	arm.pos = encoderGet(armEnc);
	arm.error = arm.target - arm.pos;
	arm.P = arm.error*arm.kP;
	if(abs(arm.P) > 100)
		arm.P = 110*arm.P/abs(arm.P);
	chainbarControl(arm.P);
}
int gTarget;
void liftTask(void * parameter){
	while(1){
		positionController();
		checkStackRelease();
		closeClaw(clawPosition);
		printf("%ld, %ld, %d; ", arm.target, arm.pos, motorGet(ilArm));
		delay(25);
	}
}

void moveLift(int target, int margin){
	gTarget = target;
	waitForLift(target, margin);
}

void moveFourbar(int up){
	if(up)
		fourbarSet(127);
	else
		fourbarSet(-127);
	delay(700);
	if(up)
		fourbarSet(5);
	else
		fourbarSet(-10);
}

void moveClaw(int close){
	if(close){
		clawSet(127);
		delay(200);
		clawSet(15);
	}
	else{
		clawSet(-127);
		delay(150);
		clawSet(-10);
	}
}
//890, 850, 810, 785, 765, 750, 733, 670,
