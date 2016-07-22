/*
FILENAME... BeckDriver.cpp
USAGE...	Motor driver support for the Beckhoff KL2541 controller.

Davide Marcato
June 17, 2016

*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
//#include <stdlib.h>

#include <iocsh.h>
//#include <epicsThread.h>

#include <asynInt32SyncIO.h>
#include <asynUInt32Digital.h>
#include <asynUInt32DigitalSyncIO.h>

#include <epicsStdlib.h>
#include <dbAccess.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include <cantProceed.h>
#include "BeckDriver.h"


#define PLC_LOOP_US 1

#include <vector>
#include <cmath>

static std::vector<BeckController *> _controllers;

BeckController::BeckController(const char *portName, const char *beckDriverPName, double movingPollPeriod, double idlePollPeriod )
  :  asynMotorController(portName, getBeckMaxAddr(beckDriverPName), 0,
						 asynUInt32DigitalMask, // Add asynUInt32Digital interface to read single bits
						 asynUInt32DigitalMask, // Add asynUInt32Digital callbacks to read single bits
						 ASYN_CANBLOCK | ASYN_MULTIDEVICE,
						 1, // autoconnect
						 0, 0)	// Default priority and stack size
{
	BeckAxis *pAxis; //set but not used not to be eliminated by compiler

	beckDriverPName_ = (char *) mallocMustSucceed(strlen(beckDriverPName)+1, "Malloc failed\n");

	strcpy(beckDriverPName_, beckDriverPName);
	int nAxis=getBeckMaxAddr(beckDriverPName);

	printf("Now create axis\n");
	int i = 0;
	for (i=0; i<nAxis; i++){
		pAxis = new BeckAxis(this, i);
		printf("Axis n: %d successfully created\n", i);
	}

	startPoller(movingPollPeriod, idlePollPeriod, 2); //TODO
}

BeckAxis* BeckController::getAxis(asynUser *pasynUser)
{
	return static_cast<BeckAxis*>(asynMotorController::getAxis(pasynUser));
}

BeckAxis* BeckController::getAxis(int axisNo)
{
	return static_cast<BeckAxis*>(asynMotorController::getAxis(axisNo));
}

void BeckController::report(FILE *fp, int level)
{
	fprintf(fp, "Beckoff motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
	this->portName, 1, movingPollPeriod_, idlePollPeriod_);

	// Call the base class method
	asynMotorController::report(fp, level);
}

extern "C" int BeckCreateController(const char *portName, const char *beckDriverPName, int movingPollPeriod, int idlePollPeriod )
{
	BeckController *ctrl = new BeckController(portName, beckDriverPName, movingPollPeriod/1000., idlePollPeriod/1000.);
	printf("Controller %p\n", ctrl);
	_controllers.push_back(ctrl);
	return(asynSuccess);
}



/************************************************/

BeckAxis::BeckAxis(BeckController *pC, int axis) :
		asynMotorAxis(pC, axis),
		pC_(pC)
{
	asynStatus status;
	static const char *functionName = "BeckAxis::BeckAxis";

	/* Connect to inputs */
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &statusByte_, "SB");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &dataIn_, "DI");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &statusWord_, "SW");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	/* Connect to outputs */
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &controlByte_, "CB");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &dataOut_, "DO");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &controlWord_, "CW");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	/* Connect to registers */
	//R0 	Actual position (low-order word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r0_, "R00");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R1 	Actual position (high-order word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r1_, "R01");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R2 	Set target position or position (low-order word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r2_, "R02");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R3 	Set target position or position (high-order word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r3_, "R03");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R7 	Command register
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r7_, "R07");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R8 	Terminal type
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r8_, "R08");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R31 	Code word register
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r31_, "R31");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R32 	Feature register 1
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r32_, "R32");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R33 	Full motor steps
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r33_, "R33");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R35 	Maximum coil current A
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r35_, "R35");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R36 	Maximum coil current B
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r36_, "R36");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R38 	Min. velocity vmin
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r38_, "R38");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R39 	Max. velocity vmax
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r39_, "R39");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R40 	Max. acceleration amax
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r40_, "R40");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R42 	Coil current IS, a > ath
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r42_, "R42");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R43 	Coil current IS, a ≤ ath
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r43_, "R43");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R44 	Coil current IS, v = 0 (automatic)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r44_, "R44");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R46 	Step size per quarter period
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r46_, "R46");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R50 	Emergency acceleration ae
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r50_, "R50");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R52 	Feature-Register 2
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r52_, "R52");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R53 	Referencing speed backward vref,b
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r53_, "R53");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R54 	Referencing speed forward vref,f
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r54_, "R54");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R55 	Referencing position (lower value word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r55_, "R55");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R56 	Referencing position (higher value word)
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r56_, "R56");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}

	//R58 	max. deceleration adec
	status = pasynInt32SyncIO->connect(pC_->beckDriverPName_, axis, &r58_, "R58");
	if (status) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff driver\n", functionName);
	}


	//printf("Interruptions are %d", interruptAccept);
	setDoubleParam(pC_->motorPosition_, 0);
	setIntegerParam(pC_->motorStatusDone_, 1);
	setDoubleParam(pC_->motorVelBase_, 100);
	setDoubleParam(pC_->motorAccel_, 100);
	setDoubleParam(pC_->motorVelocity_, 100);
	setDoubleParam(pC->motorEncoderPosition_, 0);
	setIntegerParam(pC_->motorStatusDone_, 1);

	moveDone=true;
	movePend=false;

	pasynInt32SyncIO->write(controlByte_, 0x21, 500);

}

void BeckAxis::report(FILE *fp, int level)
{
	fprintf(fp, "Axis %d status: %s\n", axisNo_, movePend ? "Moving" : "Idle");
	asynMotorAxis::report(fp, level);
}

/**
 * private method to be reused inside class
 */
asynStatus BeckAxis::setAcclVelo(double min_velocity, double max_velocity, double acceleration) {
	pasynInt32SyncIO->write(r38_, (int) min_velocity, 500);
	pasynInt32SyncIO->write(r39_, (int) max_velocity, 500);
	pasynInt32SyncIO->write(r40_, (int) acceleration, 500);
	pasynInt32SyncIO->write(r58_, (int) acceleration, 500);
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set coil currents of the motor
 */
asynStatus BeckAxis::initCurrents(double maxAmp, double autoHoldinCurr, double highAccCurr, double lowAccCurr) {
	printf("-%s(%.2f, %.2f, %.2f, %.2f)\n", __FUNCTION__, maxAmp, autoHoldinCurr, highAccCurr, lowAccCurr);
	epicsInt32 termType = 0;
	double fullScaleCurr;
	int setMaxCurrentA, setMaxCurrentB, setHoldCurr, setHighAccCurr, setLowAccCurr;
	double setMaxAmp;
	int percent = -1;

	//write passcode in register 31 to enable writing to static memory
	pasynInt32SyncIO->write(r31_, 0x1235, 500);

	//read controller code and convert ampere to %
	pasynInt32SyncIO->read(r8_, &termType, 500);
	switch (termType) {
		case 2531: fullScaleCurr = 1.5; break;
		case 2541: fullScaleCurr = 5.0;	break;
		default: {
			printf("Error: Cannot recognize controller type %d\n", termType);
			return asynError;
		}
	}
	printf("Terminal: Beckhoff KL%d  -  Max available ampere: %.2lf\n", termType, fullScaleCurr);

	//R35: Maximum coil current A (in % to fullScale of device)
	//R36: Maximum coil current B (in % to fullScale of device)
	if (maxAmp>=0){
		pasynInt32SyncIO->read(r35_, &setMaxCurrentA, 500);
		pasynInt32SyncIO->read(r36_, &setMaxCurrentB, 500);

		if (maxAmp>fullScaleCurr) {
			printf("Warning: Cannot set max current higher than full scale, reverting to %.2lfA\n", fullScaleCurr);
			maxAmp = fullScaleCurr;
		}
		percent = round( maxAmp / fullScaleCurr *100 );
		if (setMaxCurrentA!=percent) {
			printf("-R35 0x%04x -> 0x%04x: max current A (%%)\n", setMaxCurrentA, percent);
			pasynInt32SyncIO->write(r35_, percent, 500);
		}
		if (setMaxCurrentB!=percent) {
			printf("-R36 0x%04x -> 0x%04x: max current B (%%)\n", setMaxCurrentB, percent);
			pasynInt32SyncIO->write(r36_, percent, 500);
		}
	}

	//readback set maxAmp to check validity
	pasynInt32SyncIO->read(r35_, &setMaxCurrentA, 500);
	pasynInt32SyncIO->read(r36_, &setMaxCurrentB, 500);


	//lower current to minimum common if found different
	if (setMaxCurrentA>setMaxCurrentB) {
		printf("Found different max currents in coils A and B, reverting to minor one: %.2lf\n", setMaxCurrentB/100*fullScaleCurr);
		setMaxCurrentA = setMaxCurrentB;
		pasynInt32SyncIO->write(r35_, setMaxCurrentA, 500);
	}
	if (setMaxCurrentB>setMaxCurrentA) {
		printf("Found different max currents in coils A and B, reverting to minor one: %.2lf\n", setMaxCurrentA/100*fullScaleCurr);
		setMaxCurrentB = setMaxCurrentA;
		pasynInt32SyncIO->write(r36_, setMaxCurrentA, 500);
	}

	//check if the writing was unsuccessful
	setMaxAmp = ((double) setMaxCurrentA)/100.0*fullScaleCurr;
	if (maxAmp>=0 && abs(setMaxAmp-maxAmp)>0.01) {
		printf("Error: Failed to write maxCurrent %.2lf vs %.2lf\n", maxAmp, setMaxAmp);
		return asynError;
	}

	//R44: Coil current, v = 0 (automatic) (in % to maxAmp)
	if (autoHoldinCurr>=0) {
		pasynInt32SyncIO->read(r44_, &setHoldCurr, 500);
		if (autoHoldinCurr>setMaxAmp) {
			printf("Warning: Cannot set holding current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			autoHoldinCurr = setMaxAmp;
		}
		percent = round( autoHoldinCurr / setMaxAmp *100 );
		if (setHoldCurr!=percent) {
			printf("-R44 0x%04x -> 0x%04x: auto holding current (%)\n", setHoldCurr, percent);
			pasynInt32SyncIO->write(r44_, percent, 500);
		}
	}

	//R42: Coil current, a > ath (in % to maxAmp)
	if (highAccCurr>=0) {
		pasynInt32SyncIO->read(r42_, &setHighAccCurr, 500);

		if (highAccCurr>setMaxAmp) {
			printf("Warning: Cannot set over acceleration current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			highAccCurr = setMaxAmp;
		}
		percent = round( highAccCurr / setMaxAmp *100 );
		if (setHighAccCurr!=percent) {
			printf("-R42 0x%04x -> 0x%04x: coil current a>ath (%)\n", setHighAccCurr, percent);
			pasynInt32SyncIO->write(r42_, percent, 500);
		}
	}

	//R43: Coil current, a <= ath (in % to maxAmp)
	if (lowAccCurr>=0) {
		pasynInt32SyncIO->read(r43_, &setLowAccCurr, 500);

		if (lowAccCurr>setMaxAmp) {
			printf("Warning: Cannot set sub acceleration current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			lowAccCurr = setMaxAmp;
		}
		percent = round( lowAccCurr / setMaxAmp *100 );
		if (setLowAccCurr!=percent) {
			printf("-R43 0x%04x -> 0x%04x: coil current a>ath (%)\n", setLowAccCurr, percent);
			pasynInt32SyncIO->write(r43_, percent, 500);
		}
	}

	//remove passcode from register 31
	pasynInt32SyncIO->write(r31_, 0, 500);

	printf("Currents written to the controller!\n");
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set parameters for the homing procedure
 */
asynStatus BeckAxis::initHomingParams(int refPosition, bool NCcontacts, double speedToHome, double speedFromHome, double emergencyAccl){
	printf("-%s(%d, %d, %.2f, %.2f, %.2f)\n", __FUNCTION__, refPosition, NCcontacts, speedToHome, speedFromHome, emergencyAccl);
	epicsInt32 oldValue, featureReg;
	pasynInt32SyncIO->read(r55_, &oldValue, 500);
	if (oldValue!=(refPosition & 0xFFFF)){
		printf("-R55: 0x%04x -> 0x%04x \t reference position (low word)\n", oldValue, refPosition & 0xFFFF);
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r55_, refPosition & 0xFFFF, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	pasynInt32SyncIO->read(r56_, &oldValue, 500);
	if (oldValue!=((refPosition>>16) & 0xFFFF)){
		printf("-R56: 0x%04x -> 0x%04x \t reference position (high word)\n", oldValue, (refPosition>>16) & 0xFFFF);
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r56_, (refPosition>>16) & 0xFFFF, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	pasynInt32SyncIO->read(r52_, &featureReg, 500);
	oldValue = featureReg;
	/* Set bits that are set in the value and set in the mask */
	featureReg |=  (((NCcontacts<<15) + (NCcontacts<<14)) & 0xC000);
	/* Clear bits that are clear in the value and set in the mask */
	featureReg  &= (((NCcontacts<<15) + (NCcontacts<<14)) | ~0xC000);
	if (featureReg!=oldValue){
		printf("-R52: 0x%04x -> 0x%04x \t feature register 2\n", oldValue, featureReg);
		printf("-Warning: changing type of contacts usually requires a reboot or a softReset of the Beckhoff module!\n");
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r52_, featureReg, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	if (speedToHome>=0){
		pasynInt32SyncIO->read(r53_, &oldValue, 500);
		if (oldValue!=((int) speedToHome)){
			printf("-R53: 0x%04x -> 0x%04x \t speed to home\n", oldValue, (int) speedToHome);
			pasynInt32SyncIO->write(r31_, 0x1235, 500);
			pasynInt32SyncIO->write(r53_, (int) speedToHome, 500);
			pasynInt32SyncIO->write(r31_, 0, 500);
		}
	}

	if (speedFromHome>=0){
		pasynInt32SyncIO->read(r54_, &oldValue, 500);
		if (oldValue!=((int) speedFromHome)){
			printf("-R54: 0x%04x -> 0x%04x \t speed from home\n", oldValue, (int) speedFromHome);
			pasynInt32SyncIO->write(r31_, 0x1235, 500);
			pasynInt32SyncIO->write(r54_, (int) speedFromHome, 500);
			pasynInt32SyncIO->write(r31_, 0, 500);
		}
	}

	if (emergencyAccl>=0){
		printf("-R50: 0x%04x \t emergency acceleration\n", (int) emergencyAccl);
		pasynInt32SyncIO->write(r50_, (int) emergencyAccl, 500);
	}

	return asynSuccess;
}

/**
 * To be called by shell command
 * Set the step resolution of the controller
 */
asynStatus BeckAxis::initStepResolution(int microstepPerStep, int stepPerRevolution){
	printf("-%s(%d, %d)\n", __FUNCTION__, microstepPerStep, stepPerRevolution);
	epicsInt32 oldValue;
	if (microstepPerStep>0) {
		if (microstepPerStep > 64) {
			printf("Maximum microstep resolution is 64, setting 64!\n");
			microstepPerStep = 64;
		}
		if (microstepPerStep < 1) {
			printf("Minimum microstep resolution is 1, setting 1!\n");
			microstepPerStep = 1;
		}
		microstepPerStep = round(log2(microstepPerStep));

		pasynInt32SyncIO->read(r46_, &oldValue, 500);
		if (oldValue!=microstepPerStep){
			printf("-R46: 0x%04x -> 0x%04x \t microstep per step (equivalent to %d -> %d microstep)\n", oldValue, (int) microstepPerStep, (int) pow(2, oldValue), (int) pow(2, microstepPerStep));
			pasynInt32SyncIO->write(r31_, 0x1235, 500);
			pasynInt32SyncIO->write(r46_, microstepPerStep, 500);
			pasynInt32SyncIO->write(r31_, 0, 500);
		}
	}

	if (stepPerRevolution>0) {
		pasynInt32SyncIO->read(r33_, &oldValue, 500);
		if (oldValue!=stepPerRevolution){
			printf("-R33: 0x%04x -> 0x%04x \t step per revolution\n", oldValue, stepPerRevolution);
			pasynInt32SyncIO->write(r31_, 0x1235, 500);
			pasynInt32SyncIO->write(r33_, stepPerRevolution, 500);
			pasynInt32SyncIO->write(r31_, 0, 500);
		}
	}

	return asynSuccess;
}

/**
 * To be called by shell command
 * Restore the BEckhoff module to its factory settings
 */
asynStatus BeckAxis::hardReset() {
	printf("Attention - Restoring factory setting of axis %d!\n", axisNo_);
	pasynInt32SyncIO->write(r31_, 0x1235, 500);
	pasynInt32SyncIO->write(r7_, 0x7000, 500);
	pasynInt32SyncIO->write(r31_, 0, 500);
	return asynSuccess;
}


/**
 * To be called by shell command
 * Restore the Beckhoff module to the values saved in static memory
 */
asynStatus BeckAxis::softReset() {
	printf("Attention - Restoring saved setting of axis %d!\n", axisNo_);
	pasynInt32SyncIO->write(r31_, 0x1235, 500);
	pasynInt32SyncIO->write(r7_, 0x8000, 500);
	pasynInt32SyncIO->write(r31_, 0, 500);
	return asynSuccess;
}

/**
 * To be called by shell command - mandatory
 * Set general parameters
 */
asynStatus BeckAxis::init(bool encoder, bool watchdog) {
	printf("-%s(%d, %d)\n", __FUNCTION__, encoder, watchdog);
	//stop motor TODO how?
	pasynInt32SyncIO->write(controlByte_, 0x21, 500);

	//set feature register 1
	epicsInt32 featureReg, value, oldValue;
	value = 0x18	//path control mode
		  + 0x2 	//enable autostop
		  + (!encoder<<15) + (!encoder<<11) + (!watchdog<<2);
	pasynInt32SyncIO->read(r32_, &featureReg, 500);
	oldValue = featureReg;
	/* Set bits that are set in the value and set in the mask */
	featureReg |=  (value & 0x881e);
	/* Clear bits that are clear in the value and set in the mask */
	featureReg  &= (value | ~0x881e);
	if (featureReg!=oldValue) {
		printf("-FeatureReg1 0x%04x -> 0x%04x: encoder %s and watchdog %s\n", oldValue, featureReg, encoder ? "enabled" : "disabled", watchdog ? "present" : "absent");
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r32_, featureReg, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	//set feature register 2
	value = 0x8;	//enable idle
	pasynInt32SyncIO->read(r52_, &featureReg, 500);
	oldValue = featureReg;
	/* Set bits that are set in the value and set in the mask */
	featureReg |=  (value & 0x8);
	/* Clear bits that are clear in the value and set in the mask */
	featureReg  &= (value | ~0x8);
	if (featureReg!=oldValue) {
		printf("-FeatureReg2 0x%04x -> 0x%04x: idle active\n", oldValue, featureReg);
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r52_, featureReg, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	return asynSuccess;
}

/**
 * Method to execute movement
 */
asynStatus BeckAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
	//TODO check for overflow in relative mode
	//TODO verify stall or unable to end movement

	printf("-%s(%.2f, %i, %.2f, %.2f, %.2f)\n", __FUNCTION__, position, relative, min_velocity, max_velocity, acceleration);

	//This to prevent to put movePend to 1 if cannot move
	if (movePend) return asynSuccess;

	setAcclVelo(min_velocity, max_velocity, acceleration);

	//pasynInt32SyncIO->write(controlByte_, 0x21, 500);
	int newPos = (relative ? currPos : 0 ) + position;

	pasynInt32SyncIO->write(r2_, newPos & 0xFFFF, 500);
	pasynInt32SyncIO->write(r3_, (newPos>>16) & 0xFFFF, 500);
	pasynInt32SyncIO->write(dataOut_, 0, 500);

	movePend=true;
	pasynInt32SyncIO->write(controlByte_, 0x25, 500);
	return asynSuccess;
}

/**
 * Method to execute the homing
 */
asynStatus BeckAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards){
	printf("-%s(%.2f, %.2f, %.2f, %d)\n", __FUNCTION__, min_velocity, max_velocity, acceleration, forwards);
	epicsInt32 featureReg, oldValue;
	setAcclVelo(min_velocity, max_velocity, acceleration);

//	pasynInt32SyncIO->write(controlByte_, 0x21, 500);

	//set feature register 2
	pasynInt32SyncIO->read(r52_, &featureReg, 500);
	oldValue = featureReg;
	/* Set bits that are set in the value and set in the mask */
	featureReg |=  (forwards & 0x1);
	/* Clear bits that are clear in the value and set in the mask */
	featureReg  &= (forwards | ~0x1);
	if (featureReg!=oldValue) {
		printf("-FeatureReg2 0x%04x -> 0x%04x: idle active\n", oldValue, featureReg);
		pasynInt32SyncIO->write(r31_, 0x1235, 500);
		pasynInt32SyncIO->write(r52_, featureReg, 500);
		pasynInt32SyncIO->write(r31_, 0, 500);
	}

	pasynInt32SyncIO->write(r7_, 0x520, 500);
	pasynInt32SyncIO->write(dataOut_, 0, 500);

	movePend=true;
	pasynInt32SyncIO->write(controlByte_, 0x25, 500);
	return asynSuccess;
}

/**
 * Poller to update motor status on the record
 */
asynStatus BeckAxis::poll(bool *moving) {
	//printf("Polling\n");

	epicsInt32 pLow, pHigh;
	epicsInt32 statusByte, statusWord;
	bool lHigh, lLow;
	bool regAccess, error, warning, idle, ready;
	epicsInt32 loadAngle;

	//update position
	pasynInt32SyncIO->read(r0_, &pLow, 500);
	pasynInt32SyncIO->read(r1_, &pHigh, 500);
	currPos = pLow + (pHigh<<16);
	setDoubleParam(pC_->motorPosition_, currPos);

	//update limit switches
	pasynInt32SyncIO->read(statusWord_, &statusWord, 500);
	lHigh = statusWord & 0x1;
	lLow = statusWord & 0x2;
	setIntegerParam(pC_->motorStatusHighLimit_, lHigh);
	setIntegerParam(pC_->motorStatusLowLimit_, lLow);

	//set moveDone flag
	/*moveDone = movePend ? ((statusWord & 0x8) || lHigh || lLow) : true;
	*moving = moveDone ? false : true;
	setIntegerParam(pC_->motorStatusDone_, moveDone);*/
	moveDone=!movePend;
	pC_->modbusMutex.lock();
	if (moveDone) {
		pasynInt32SyncIO->write(controlByte_, 0x21, 500);
		movePend = false;
	}
	else{
		pasynInt32SyncIO->write(controlByte_, 0x25, 500);
		printf("-current position: %.2f\n", currPos);
	}

	//set status
	pasynInt32SyncIO->read(statusByte_, &statusByte, 500);
	pC_->modbusMutex.unlock();
	regAccess = statusByte & 0x80;
	error = statusByte & 0x40;
	setIntegerParam(pC_->motorStatusProblem_, error);
	warning = statusByte & 0x20;
	idle = statusByte & 0x10;
	movePend = !idle;
	*moving = idle;
	setIntegerParam(pC_->motorStatusDone_, !movePend);
	loadAngle = statusByte & 0xE;
	ready = statusByte & 0x1;
	setIntegerParam(pC_->motorStatusPowerOn_, ready);

	//printf("Moving: %d\t MoveDone: %d\t SwL: %d\t SwH: %d\n", movePend, moveDone, llow, lhigh);
	callParamCallbacks();

	return asynSuccess;
}








/**
 * Config Controller Functions
 */
BeckController * findBeckControllerByName(const char *name) {
	for(std::vector<BeckController *>::iterator i = _controllers.begin(); i != _controllers.end(); i++ ){
		if (strcmp(name, (*i)->portName) == 0)
			return *i;
	}
	return 0;
}

extern "C" int BeckConfigController(const char *ctrlName, int axisRange, const char *cmd, const char *cmdArgs) {
	BeckController *ctrl = findBeckControllerByName(ctrlName);
	if (ctrl == NULL) {
		epicsStdoutPrintf("Cannot find controller %s!\n", ctrlName);
		return 0;
	}
	BeckAxis *axis = ctrl->getAxis(axisRange);
	if (axis == NULL) {
		epicsStdoutPrintf("Cannot find axis %d!\n", axisRange);
	}

	if (strcmp(cmd, "initCurrents") == 0) {
		char *maxCurrStr=0;
		char *autoHoldinCurrStr=0;
		char *highAccCurrStr=0;
		char *lowAccCurrStr=0;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,],%m[^,],%m[^,]", &maxCurrStr,
																  &autoHoldinCurrStr,
																  &highAccCurrStr,
																  &lowAccCurrStr);

		double maxCurr=-1;
		double autoHoldinCurr=-1;
		double highAccCurr=-1;
		double lowAccCurr=-1;

		switch (nPar) {
			case 4: epicsScanDouble(lowAccCurrStr, &lowAccCurr);
			case 3: epicsScanDouble(highAccCurrStr, &highAccCurr);
			case 2: epicsScanDouble(autoHoldinCurrStr, &autoHoldinCurr);
			case 1: epicsScanDouble(maxCurrStr, &maxCurr); break;
			default: {
				printf ("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}

		}
		axis->initCurrents(maxCurr, autoHoldinCurr, highAccCurr, lowAccCurr);
	}
	else if (strcmp(cmd, "softReset") ==0 ) {
		axis->softReset();
	}
	else if (strcmp(cmd, "hardReset") ==0 ) {
			axis->hardReset();
	}
	else if (strcmp(cmd, "init") ==0 ) {
		char *encoderStr=0;
		char *watchdogStr=0;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,]", &encoderStr,
													&watchdogStr);
		double encoder = 0;
		double watchdog = 0;

		switch (nPar) {
			case 2: epicsScanDouble(watchdogStr, &watchdog);
			case 1: epicsScanDouble(encoderStr, &encoder); break;
			default: {
				printf ("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}
		axis->init((bool) encoder, (bool) watchdog);
	}
	else if (strcmp(cmd, "initHomingParams") ==0 ) {
		char *refPositionStr;
		char *NCcontactsStr;
		char *speedToHomeStr;
		char *speedFromHomeStr;
		char *emergencyAcclStr;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,],%m[^,],%m[^,],%m[^,]",&refPositionStr,
																		&NCcontactsStr,
																		&speedToHomeStr,
																		&speedFromHomeStr,
																		&emergencyAcclStr);
		double refPosition = 0;
		double NCcontacts = 0;
		double speedToHome = -1, speedFromHome = -1, emergencyAccl = -1;

		switch (nPar) {
			case 5: epicsScanDouble(emergencyAcclStr, &emergencyAccl);
			case 4: epicsScanDouble(speedFromHomeStr, &speedFromHome);
			case 3: epicsScanDouble(speedToHomeStr, &speedToHome);
			case 2: epicsScanDouble(NCcontactsStr, &NCcontacts);
			case 1: epicsScanDouble(refPositionStr, &refPosition); break;
			default: {
				printf ("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}
		axis->initHomingParams((int) refPosition, (bool) NCcontacts, speedToHome, speedFromHome, emergencyAccl);
	}
	else if (strcmp(cmd, "initStepResolution") ==0 ) {
		char *microstepPerStepStr;
		char *stepPerRevolutionStr;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,]",&microstepPerStepStr,
												   &stepPerRevolutionStr);
		double microstepPerStep = -1;
		double stepPerRevolution = -1;

		switch (nPar) {
			case 2: epicsScanDouble(stepPerRevolutionStr, &stepPerRevolution);
			case 1: epicsScanDouble(microstepPerStepStr, &microstepPerStep); break;
			default: {
				printf ("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}
		axis->initStepResolution((int) microstepPerStep, (int) stepPerRevolution);
	}
	else {
		epicsStdoutPrintf("BeckConfigController: Command \"%s\" not found!\n", cmd);
	}
	//ADD: set/getMicrostep - Encoder related fields - referencing position
	return(asynSuccess);
}








/**
 * Code for iocsh registration
 */
static const iocshArg BeckCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateControllerArg1 = {"Driver port name", iocshArgString};
static const iocshArg BeckCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg BeckCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};

static const iocshArg BeckConfigControllerArg0 = {"Controller Name", iocshArgString};
static const iocshArg BeckConfigControllerArg1 = {"Axis Number", iocshArgInt};
static const iocshArg BeckConfigControllerArg2 = {"Command", iocshArgString};
static const iocshArg BeckConfigControllerArg3 = {"CommandArgs", iocshArgString};

static const iocshArg * const BeckCreateControllerArgs[] = {&BeckCreateControllerArg0,
															&BeckCreateControllerArg1,
															&BeckCreateControllerArg2,
															&BeckCreateControllerArg3};
static const iocshArg * const BeckConfigControllerArgs[] = {&BeckConfigControllerArg0,
															&BeckConfigControllerArg1,
															&BeckConfigControllerArg2,
															&BeckConfigControllerArg3};

static const iocshFuncDef BeckCreateControllerDef = {"BeckCreateController", 4, BeckCreateControllerArgs};
static const iocshFuncDef BeckConfigControllerDef = {"BeckConfigController", 4, BeckConfigControllerArgs};

static void BeckCreateControllerCallFunc(const iocshArgBuf *args) {
	BeckCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}
static void BeckConfigControllerCallFunc(const iocshArgBuf *args) {
	BeckConfigController(args[0].sval, args[1].ival, args[2].sval, args[3].sval);
}

static void BeckRegister(void) {
	iocshRegister(&BeckCreateControllerDef, BeckCreateControllerCallFunc);
	iocshRegister(&BeckConfigControllerDef, BeckConfigControllerCallFunc);
}
extern "C" {
	epicsExportRegistrar(BeckRegister);
}
