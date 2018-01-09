#ifndef _SUBSYSTEMS_H
#define _SUBSYSTEMS_H

//subsystem motor functions
void lDriveSet(short control);
void rDriveSet(short control);
void driveForward(short control);
void driveTurn(short control);
void chainbarSet(int control);
void fourbarSet(int control);
void clawSet(int control);
void chainbarControl(int target);
void brakeSet(int control);

extern int gTarget;
void liftTask(void * parameter);

struct PIDcont{
	long error, derivative, P, target, pos;
	short height, bottom;
	float kP;
};
extern struct PIDcont arm;
void positionController(void);
void checkStackRelease(void);
void updateArmTarget(void);
extern const int armHeightMax;

void closeClaw(int close);
extern int clawPosition;

void fourbarSet(int control);
void closeGate(int close);
void prepChainbar();
void updateArmTarget();

void checkStackRelease() ;
//set both sides of drive
void driveSet(int left, int right);
void positionController();
extern int gTarget;
void liftTask(void * parameter);

void moveLift(int target, int margin);
void moveFourbar(int up);
void moveClaw(int close);

#endif
