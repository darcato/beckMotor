/**
 *  BeckDriver.cpp
 *  Implementation of record motor API to support beckhoff KL2541 stepper motor driver
 *
 *  Created on: June 17, 2016
 *      Authors: Damiano Bortolato, Davide Marcato.
 *      Mail: damiano.bortolato@lnl.infn.it davide.marcato@lnl.infn.it
 *      Project: SPES project at Laboratori Nazionali di Legnaro, INFN, Italy
 *
 *
 *  This file is part of Beckhoff KL2541 device support for epics motor.
 *
 *  This device support is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This device support is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this device support.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
//#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynInt32SyncIO.h>
#include <asynInt32ArraySyncIO.h>
#include <asynUInt32Digital.h>
#include <asynUInt32DigitalSyncIO.h>

#include <epicsStdlib.h>
#include <dbAccess.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include <cantProceed.h>
#include "BeckController.h"

#define OUTOFSWITCH_STEPS 200

#include <cmath>

//A vector to store all the controllers, in order to search into it to find controller by its name
static std::vector<BeckController *> _controllers;

//BeckController  :  asynMotorController  :  asynPortDriver
//Represent a number of KL2541 Beckhoff modules
BeckController::BeckController(const char *portName, const char *beckDriverPName, double movingPollPeriod, double idlePollPeriod )
  :  asynMotorController(portName, getBeckMaxAddr(beckDriverPName), 0,
						 asynUInt32DigitalMask, // Add asynUInt32Digital interface to read single bits
						 asynUInt32DigitalMask, // Add asynUInt32Digital callbacks to read single bits
						 ASYN_CANBLOCK | ASYN_MULTIDEVICE,
						 1, // autoconnect
						 0, 0)	// Default priority and stack size
{
	asynStatus status;
	//static const char *functionName = "BeckController::BeckController";

	BeckAxis *pAxis; //set but not used not to be eliminated by compiler

	beckDriverPName_ = (char *) mallocMustSucceed(strlen(beckDriverPName)+1, "Malloc failed\n");
	strcpy(beckDriverPName_, beckDriverPName);

	//create each axis of this controller
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,"Now create %d axis\n", numAxes_);
	for (int i=0; i<numAxes_; i++){
		pAxis = new BeckAxis(this, i);
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,"Axis n: %d successfully created\n", i);
		std::array<epicsInt32, 3> axisRegs;
		memInp_cache.push_back(axisRegs);;
	}

	//arrays to keep values polled from the controller
	//the axis pollers will read this instead of doing modbus I/O -> single big readings instead of small ones = efficiency
	r1_cache = new epicsInt32[numAxes_];

	char name[10];
	for (int i=0; i < KL2541_N_REG; i++){
		sprintf(name, "R%02i", i);
		r_.push_back(new asynInt32ArrayClient(beckDriverPName_, 0, name));
	}

	memInp_ = new asynInt32ArrayClient(beckDriverPName_, 0, "MI");
	memOut_ = new asynInt32ArrayClient(beckDriverPName_, 0, "MO");
	statusByte_ = new asynInt32ArrayClient(beckDriverPName_, 0, "SB");
	dataIn_ = new asynInt32ArrayClient(beckDriverPName_, 0, "DI");
	statusWord_ = new asynInt32ArrayClient(beckDriverPName_, 0, "SW");
	controlByte_ = new asynInt32ArrayClient(beckDriverPName_, 0, "CB");
	dataOut_ = new asynInt32ArrayClient(beckDriverPName_, 0, "DO");
	controlWord_ = new asynInt32ArrayClient(beckDriverPName_, 0, "CW");

	//initialize the cache
	size_t nin;
	status = r_[1]->read(r1_cache, numAxes_, &nin);
	if (status!=asynSuccess) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Warning: could not initialize r1 cache\n");
	}

	//start the poller of each axis with two polling times: while the axis is moving and while it is not
	startPoller(movingPollPeriod, idlePollPeriod, 2);
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
	fprintf(fp, "Beckhoff motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
	this->portName, 1, movingPollPeriod_, idlePollPeriod_);

	// Call the base class method
	asynMotorController::report(fp, level);
}


/**
 * This is called once before calling in sequence all the axis polls,
 * so this performs the actual modbus IO, reading all the memory for most efficiency,
 * while axis polls simply access to the cached data and interpret it.
 */
asynStatus BeckController::poll() {
	//asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "polling...\n");
	pHighAlreadyRead = false;

	size_t nin;
	memInp_->read(memInp_cache.data()->data(), 3*numAxes_, &nin);

	return asynSuccess;
}

//function called from iocsh to create a controller and save it into the _controllers vector
extern "C" int BeckCreateController(const char *portName, const char *beckDriverPName, int movingPollPeriod, int idlePollPeriod ) {
	BeckController *ctrl = new BeckController(portName, beckDriverPName, movingPollPeriod/1000., idlePollPeriod/1000.);
	epicsStdoutPrintf("Controller %p\n", ctrl);
	_controllers.push_back(ctrl);
	return(asynSuccess);
}



/************************************************/

//create an axis representing a single beckhoff module and its motor
BeckAxis::BeckAxis(BeckController *pC, int axis) :
		asynMotorAxis(pC, axis),
		pC_(pC)
{
	//asynStatus status;
	//static const char *functionName = "BeckAxis::BeckAxis";


	char name[10];
	for (int i=0; i < KL2541_N_REG; i++){
		sprintf(name, "R%02i", i);
		r_.push_back(new asynInt32Client(pC_->beckDriverPName_, axis, name));
		ru_.push_back(new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, name));
	}

	statusByte_ = new asynInt32Client(pC_->beckDriverPName_, axis, "SB");
	dataIn_ = new asynInt32Client(pC_->beckDriverPName_, axis, "DI");
	statusWord_ = new asynInt32Client(pC_->beckDriverPName_, axis, "SW");
	controlByte_ = new asynInt32Client(pC_->beckDriverPName_, axis, "CB");
	dataOut_ = new asynInt32Client(pC_->beckDriverPName_, axis, "DO");
	controlWord_ = new asynInt32Client(pC_->beckDriverPName_, axis, "CW");

	statusByteBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "SB");
	dataInBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "DI");
	statusWordBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "SW");
	controlByteBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "CB");
	dataOutBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "DO");
	controlWordBits_ = new asynUInt32DigitalClient(pC_->beckDriverPName_, axis, "CW");

	//set convenient parameters to initialize record motor
	setDoubleParam(pC_->motorPosition_, 0);
	setIntegerParam(pC_->motorStatusDone_, 1);
	setDoubleParam(pC_->motorVelBase_, 100);
	setDoubleParam(pC_->motorAccel_, 100);
	setDoubleParam(pC_->motorVelocity_, 100);
	setDoubleParam(pC->motorEncoderPosition_, 0);
	setIntegerParam(pC_->motorStatusDone_, 1);

	//initialize local parameters
	movePend=false;
	limitSwitchDownIsInputOne = 0;  //to invert the limit switches, based on how they are cabled
	curr_min_velo = 0;
	curr_max_velo = 0;
	curr_home_velo = 0;
	curr_acc = 0;
	curr_forw = 0;
	startingHome = false;
	exitingLimSw = false;
	microstepPerStep = 64;
	lHigh = false;
	lLow = false;
	currPos = 0;
	lastDir = 0;
	r_[2]->read(&lastr2);
	r_[3]->read(&lastr3);

	//give current to the motor (enable)
	controlByte_->write(0x21);
}

//a report function for the record
void BeckAxis::report(FILE *fp, int level) {
	fprintf(fp, "Axis %d status: %s\n", axisNo_, movePend ? "Moving" : "Idle");
	asynMotorAxis::report(fp, level);
}

//a convenient function to read the current position, stored in currPos
asynStatus BeckAxis::updateCurrentPosition() {
	epicsInt32 pLow, pHigh; //it is stored in 2 registers, to be read and recombined
	pLow = pC_->memInp_cache[axisNo_][DI];
	if (((((epicsInt32) currPos) & 0x8000)!=(pLow&0x8000) || pLow==0) && !pC_->pHighAlreadyRead) {  //most significant bit of pLow has changed, or pLow is 0 (after homing)
		size_t nin;
		pC_->r_[1]->read(pC_->r1_cache, pC_->numAxes_, &nin);
		pC_->pHighAlreadyRead = true;
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%d) Updated r1_cache: 0x%04x\n", __FUNCTION__, axisNo_, pC_->r1_cache[axisNo_]);
	}
	pHigh = pC_->r1_cache[axisNo_];

	currPos = pLow + (pHigh<<16);
	//epicsStdoutPrintf("high: %d   --  low: %d  --  tot: %.2f\n", pHigh, pLow, currPos);
	return asynSuccess;
}

//convenient function to set acceleration and velocity of the motor
asynStatus BeckAxis::setAcclVelo(double min_velocity, double max_velocity, double acceleration) {
	//info at http://infosys.beckhoff.com/italiano.php?content=../content/1040/bk9000/html/bt_bk9000_title.htm&id=259
	if (min_velocity!=curr_min_velo) {
		curr_min_velo = min_velocity;
		min_velocity = (int) min_velocity * 0.016384;  //vel=mstep/sec/16Mhz*262144
		r_[38]->write(min_velocity);
	}
	if (max_velocity!=curr_max_velo) {
		curr_max_velo = max_velocity;
		max_velocity = (int) max_velocity * 0.016384;
		r_[39]->write(max_velocity);
	}
	if (acceleration!=curr_acc) {
		curr_acc = acceleration;
		acceleration = (int) acceleration * 1.073742/1000;  //accl = mstep/s^2*2^38/(16Mhz)^2
		r_[40]->write(acceleration);
		r_[58]->write(acceleration);
	}
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set coil currents of the motor
 */
asynStatus BeckAxis::initCurrents(double maxAmp, double autoHoldinCurr, double highAccCurr, double lowAccCurr) {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%.2f, %.2f, %.2f, %.2f)\n", __FUNCTION__, maxAmp, autoHoldinCurr, highAccCurr, lowAccCurr);
	epicsInt32 termType = 0;
	double fullScaleCurr;
	int setMaxCurrentA, setMaxCurrentB, setHoldCurr, setHighAccCurr, setLowAccCurr;
	double setMaxAmp;
	int percent = -1;

	//write passcode in register 31 to enable writing to static memory
	r_[31]->write(0x1235);

	//read controller code and convert ampere to %
	r_[8]->read(&termType);
	switch (termType) {
		case 2531: fullScaleCurr = 1.5; break;
		case 2541: fullScaleCurr = 5.0;	break;
		default: {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Error: Cannot recognize controller type %d\n", termType);
			return asynError;
		}
	}
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Terminal: Beckhoff KL%d  -  Max available ampere: %.2lf\n", termType, fullScaleCurr);

	//R35: Maximum coil current A (in % to fullScale of device)
	//R36: Maximum coil current B (in % to fullScale of device)
	if (maxAmp>=0){
		r_[35]->read(&setMaxCurrentA);
		r_[36]->read(&setMaxCurrentB);

		if (maxAmp>fullScaleCurr) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Warning: Cannot set max current higher than full scale, reverting to %.2lfA\n", fullScaleCurr);
			maxAmp = fullScaleCurr;
		}
		percent = round( maxAmp / fullScaleCurr *100 );
		if (setMaxCurrentA!=percent) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R35 0x%04x -> 0x%04x: max current A (%%)\n", setMaxCurrentA, percent);
			r_[35]->write(percent);
		}
		if (setMaxCurrentB!=percent) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R36 0x%04x -> 0x%04x: max current B (%%)\n", setMaxCurrentB, percent);
			r_[36]->write(percent);
		}
	}

	//readback set maxAmp to check validity
	r_[35]->read(&setMaxCurrentA);
	r_[36]->read(&setMaxCurrentB);


	//lower current to minimum common if found different
	if (setMaxCurrentA>setMaxCurrentB) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Found different max currents in coils A and B, reverting to minor one: %.2lf\n", setMaxCurrentB/100*fullScaleCurr);
		setMaxCurrentA = setMaxCurrentB;
		r_[35]->write(setMaxCurrentA);
	}
	if (setMaxCurrentB>setMaxCurrentA) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Found different max currents in coils A and B, reverting to minor one: %.2lf\n", setMaxCurrentA/100*fullScaleCurr);
		setMaxCurrentB = setMaxCurrentA;
		r_[36]->write(setMaxCurrentB);
	}

	//check if the writing was unsuccessful
	setMaxAmp = ((double) setMaxCurrentA)/100.0*fullScaleCurr;
	if (maxAmp>=0 && abs(setMaxAmp-maxAmp)>0.01) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Error: Failed to write maxCurrent %.2lf vs %.2lf\n", maxAmp, setMaxAmp);
		return asynError;
	}

	//R44: Coil current, v = 0 (automatic) (in % to maxAmp)
	if (autoHoldinCurr>=0) {
		r_[44]->read(&setHoldCurr);
		if (autoHoldinCurr>setMaxAmp) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Warning: Cannot set holding current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			autoHoldinCurr = setMaxAmp;
		}
		percent = round( autoHoldinCurr / setMaxAmp *100 );
		if (setHoldCurr!=percent) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R44 0x%04x -> 0x%04x: auto holding current (%%)\n", setHoldCurr, percent);
			r_[44]->write(percent);
		}
	}

	//R42: Coil current, a > ath (in % to maxAmp)
	if (highAccCurr>=0) {
		r_[42]->read(&setHighAccCurr);

		if (highAccCurr>setMaxAmp) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Warning: Cannot set over acceleration current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			highAccCurr = setMaxAmp;
		}
		percent = round( highAccCurr / setMaxAmp *100 );
		if (setHighAccCurr!=percent) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R42 0x%04x -> 0x%04x: coil current a>ath (%%)\n", setHighAccCurr, percent);
			r_[42]->write(percent);
		}
	}

	//R43: Coil current, a <= ath (in % to maxAmp)
	if (lowAccCurr>=0) {
		r_[43]->read(&setLowAccCurr);

		if (lowAccCurr>setMaxAmp) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Warning: Cannot set sub acceleration current higher than maximum coil current, reverting to %.2lfA\n", setMaxAmp);
			lowAccCurr = setMaxAmp;
		}
		percent = round( lowAccCurr / setMaxAmp *100 );
		if (setLowAccCurr!=percent) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R43 0x%04x -> 0x%04x: coil current a>ath (%%)\n", setLowAccCurr, percent);
			r_[43]->write(percent);
		}
	}

	//remove passcode from register 31
	r_[31]->write(0);

	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Currents written to the controller!\n");
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set parameters for the homing procedure
 */
asynStatus BeckAxis::initHomingParams(int refPosition, bool NCcontacts, bool lsDownOne, int homeAtStartup, double homingSpeed, double emergencyAccl){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%d, %d, %d, %d, %.2f, %.2f)\n", __FUNCTION__, refPosition, NCcontacts, lsDownOne, homeAtStartup, homingSpeed, emergencyAccl);
	epicsInt32 oldValue;
	epicsUInt32 oldRegister, featureReg;

	r_[55]->read(&oldValue);
	if (oldValue!=(refPosition & 0xFFFF)){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R55: 0x%04x -> 0x%04x \t reference position (low word)\n", oldValue, refPosition & 0xFFFF);
		r_[31]->write(0x1235);
		r_[55]->write(refPosition & 0xFFFF);
		r_[31]->write(0);
	}

	r_[56]->read(&oldValue);
	if (oldValue!=((refPosition>>16) & 0xFFFF)){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R56: 0x%04x -> 0x%04x \t reference position (high word)\n", oldValue, (refPosition>>16) & 0xFFFF);
		r_[31]->write(0x1235);
		r_[56]->write((refPosition>>16) & 0xFFFF);
		r_[31]->write(0);
	}

	ru_[52]->read(&oldRegister, 0xC000);
	featureReg = (NCcontacts<<15) + (NCcontacts<<14);
	if (featureReg!=oldRegister){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R52: 0x%04x -> 0x%04x \t feature register 2\n", oldRegister, featureReg);
		epicsStdoutPrintf("\nport %s axis %d - WARNING: changing type of contacts usually requires a reboot or a softReset of the Beckhoff module!\n", pC_->portName, axisNo_);
		r_[31]->write(0x1235);
		ru_[52]->write(featureReg, 0xC000);
		r_[31]->write(0);
	}

	if (homingSpeed>=0){
		r_[53]->read(&oldValue);
		if (oldValue!=((int) homingSpeed)){
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R53: 0x%04x -> 0x%04x \t speed to home\n", oldValue, (int) homingSpeed);
			r_[31]->write(0x1235);
			r_[53]->write((int) homingSpeed);
			r_[54]->write((int) homingSpeed);
			r_[31]->write(0);
		}
	}

	if (emergencyAccl>=0){
		r_[50]->read(&oldValue);
		if (oldValue!=((int) emergencyAccl)){
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R50: 0x%04x -> 0x%04x \t emergency acceleration\n", oldValue, (int) emergencyAccl);
			r_[50]->write((int) emergencyAccl);
		}
	}

	if (homeAtStartup!=0) {
		home(100, 500, 1000, (homeAtStartup>0) ? 1 : 0);
	}
	limitSwitchDownIsInputOne = lsDownOne;
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set the step resolution of the controller
 */
asynStatus BeckAxis::initStepResolution(int microstepPerStep, int stepPerRevolution){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%d, %d)\n", __FUNCTION__, microstepPerStep, stepPerRevolution);
	epicsInt32 oldValue;
	if (microstepPerStep>0) {
		if (microstepPerStep > 64) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Maximum microstep resolution is 64, setting 64!\n");
			microstepPerStep = 64;
		}
		if (microstepPerStep < 1) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Minimum microstep resolution is 1, setting 1!\n");
			microstepPerStep = 1;
		}

		this->microstepPerStep = microstepPerStep;
		microstepPerStep = round(log2(microstepPerStep));

		r_[46]->read(&oldValue);
		if (oldValue!=microstepPerStep){
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R46: 0x%04x -> 0x%04x \t microstep per step (equivalent to %d -> %d microstep)\n", oldValue, (int) microstepPerStep, (int) pow(2, oldValue), this->microstepPerStep);
			r_[31]->write(0x1235);
			r_[46]->write(microstepPerStep);
			r_[31]->write(0);
		}
	}

	if (stepPerRevolution>0) {
		r_[33]->read(&oldValue);
		if (oldValue!=stepPerRevolution){
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R33: 0x%04x -> 0x%04x \t step per revolution\n", oldValue, stepPerRevolution);
			r_[31]->write(0x1235);
			r_[33]->write(stepPerRevolution);
			r_[31]->write(0);
		}
	}

	return asynSuccess;
}

/**
 * To be called by shell command
 * Restore the BEckhoff module to its factory settings
 */
asynStatus BeckAxis::hardReset() {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Attention - Restoring factory setting of axis %d!\n", axisNo_);
	r_[31]->write(0x1235);
	r_[7]->write(0x7000);
	r_[31]->write(0);
	return asynSuccess;
}


/**
 * To be called by shell command
 * Restore the Beckhoff module to the values saved in static memory
 */
asynStatus BeckAxis::softReset() {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Attention - Restoring saved setting of axis %d!\n", axisNo_);
	r_[31]->write(0x1235);
	r_[7]->write(0x8000);
	r_[31]->write(0);
	return asynSuccess;
}

/**
 * To be called by shell command - mandatory
 * Set general parameters
 */
asynStatus BeckAxis::init(bool encoder, bool watchdog) {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%d, %d)\n", __FUNCTION__, encoder, watchdog);

	//reset precedent errors
	controlByteBits_->write(0x40, 0x40);
	controlByteBits_->write(0, 0x40);

	//stop motor
	stop(0.0);

	//set feature register 1
	epicsUInt32 featureReg, value;
	value = 0x18	//path control mode
		  + 0x2 	//enable autostop
		  + (!encoder<<15) + (!encoder<<11) + (!watchdog<<2);

	ru_[32]->read(&featureReg, 0x881e);
	if (featureReg!=value) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-FeatureReg1 0x%04x -> 0x%04x: encoder %s and watchdog %s\n", featureReg, value, encoder ? "enabled" : "disabled", watchdog ? "present" : "absent");
		r_[31]->write(0x1235);
		ru_[32]->write(value, 0x881e);
		r_[31]->write(0);
	}

	//set feature register 2
	value = 0x8;	//enable idle
	ru_[52]->read(&featureReg, 0x8);
	if (featureReg!=value) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-FeatureReg2 0x%04x -> 0x%04x: idle active\n", featureReg, value);
		r_[31]->write(0x1235);
		ru_[52]->write(value, 0x8);
		r_[31]->write(0);
	}

	return asynSuccess;
}

/**
 * To be called by shell command - mandatory
 * Set encoder parameters
 * ppr = pulse per revolution
 * invert = if an external encoder is installed opposite the stepper motor (e.g. the encoder shows a negative rotation when the motor rotates in positive direction).
 */
asynStatus BeckAxis::initEncoder(int ppr, bool invert){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%d, %d)\n", __FUNCTION__, ppr, invert);

	epicsUInt32 featureReg, value;
	epicsInt32 oldValue;

	//set reg 34 = number of increments issued by the encoder connected to the KL2541 during a complete turn (default: 4000).
	r_[34]->read(&oldValue);
	if (oldValue!=ppr){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-R34: 0x%04x -> 0x%04x \n", oldValue, ppr);
		r_[31]->write(0x1235);
		r_[34]->write(ppr);
		r_[31]->write(0);
	}

	//set feature register 1
	value = (invert << 6);
	ru_[32]->read(&featureReg, 0x40);
	if (featureReg!=value) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-FeatureReg1 0x%04x -> 0x%04x: encoder %s \n", featureReg, value, invert ? "inverted" : "not inverted");
		r_[31]->write(0x1235);
		ru_[32]->write(value, 0x40);
		r_[31]->write(0);
	}

	return asynSuccess;
}

//Method to execute movement
asynStatus BeckAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
	//TODO check for overflow in relative mode

	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%.2f, %i, %.2f, %.2f, %.2f)\n", __FUNCTION__, position, relative, min_velocity, max_velocity, acceleration);

	//This to prevent to put movePend to 1 if cannot move
	if (movePend) return asynSuccess;

	setAcclVelo(min_velocity, max_velocity, acceleration);
	exitingLimSw = false;

	int newPos = (relative ? currPos : 0 ) + position;
	if (newPos==currPos) return asynSuccess;

	if (lLow) {
		if (newPos < currPos) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,"Already at limit switch in negative direction!\n");
			return asynSuccess;
		} else {
			exitingLimSw = true;
		}
	} else if (lHigh) {
		if (newPos > currPos) {
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,"Already at limit switch in positive direction!\n");
			return asynSuccess;
		} else {
			exitingLimSw = true;
		}
	}

	int newr2 = newPos & 0xFFFF;
	int newr3 = (newPos>>16) & 0xFFFF;
	if (newr2!=lastr2) {
		r_[2]->write(newr2);
		lastr2 = newr2;
	}
	if (newr3!=lastr3) {
		r_[3]->write(newr3);
		lastr3 = newr3;
	}
	int curr_dataOut;
	dataOut_->read(&curr_dataOut);  //read from cache
	if (curr_dataOut!=0){
		dataOut_->write(0);
	}

	int goCmd = exitingLimSw ? 0x5 : 0x25;

	//start movement
	lastDir = (newPos-currPos) / abs(newPos-currPos);
	movePend=true;
	setIntegerParam(pC_->motorStatusDone_, false);
	callParamCallbacks();
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-starting position:\t%10.2f\n", currPos);
	int curr_controByte;
	controlByte_->read(&curr_controByte);  //read from cache
	if (curr_controByte & 0x4) {  //we need a rising edge on bit 2 of controlByte -- if it is currently at 1, first zero it, then put to 1
		controlByte_->write(0x1);
	}
	controlByte_->write(goCmd);  //the movement should now start
	epicsThreadSleep(0.050); //wait at least 50ms before polling to let the controller update moveDone bit
	return asynSuccess;
}

//Method to execute the homing
asynStatus BeckAxis::home(double min_velocity, double home_velocity, double acceleration, int forward){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%.2f, %.2f, %.2f, %d)\n", __FUNCTION__, min_velocity, home_velocity, acceleration, forward);
	startingHome = false;

	//set runtime parameters
	setAcclVelo(min_velocity, curr_max_velo, acceleration);

	//set homing velocity and direction
	if (home_velocity!=curr_home_velo or forward!=curr_forw){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s: updating velocity and direction\n", __FUNCTION__);
		r_[31]->write(0x1235);  //enable write to protected regs
		if (home_velocity!=curr_home_velo) {
			curr_home_velo = home_velocity;
			home_velocity = (int) home_velocity* 0.016384;  //vel=mstep/sec/16Mhz*262144
			r_[53]->write(home_velocity);
			r_[54]->write(home_velocity);
		}
		if (forward!=curr_forw) {
			curr_forw = forward;
			forward = (bool) forward;
			ru_[52]->write(forward, 0x1);
		}
		r_[31]->write(0);  //disable write on protected regs
	}

	//cannot start homing until both limit switches are not pressed
	if (lLow || lHigh) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s: limits switch, moving out...\n", __FUNCTION__);
		lastDir = lLow ? -1 : 1;
		move(-lastDir*OUTOFSWITCH_STEPS*microstepPerStep, 1, curr_min_velo, curr_max_velo, curr_acc);  //exit limit switches
		startingHome = true;
		return asynSuccess;
	}

	//start homing
	dataOut_->write(0);
	controlByte_->write(1);
	r_[7]->write(0x520);

	movePend=true;
	controlByte_->write(0x25);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"Homing started!! ----------------------------------------------------\n");

	//update lastDir, it is the contrary of the homing direction, as the homing ends moving away from limit switch
	lastDir = forward ? -1 : 1;
	epicsThreadSleep(0.050); //wait at least 50ms before polling to let the controller update moveDone bit
	return asynSuccess;
}

//Method to stop motor movement
asynStatus BeckAxis::stop(double acceleration){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-%s(%.2f) -moving: %d\n", __FUNCTION__, acceleration, movePend);

	controlByteBits_->write(1<<1, 0x2);
	controlByteBits_->write(0, 0x2);

	return asynSuccess;
}


//Poller to update motor status on the record
asynStatus BeckAxis::poll(bool *moving) {
	//epicsStdoutPrintf("- %02d poll -- homing: %d  -- exitingLimSw: %d\n", axisNo_, startingHome, exitingLimSw);
	epicsInt32 statusByte, statusWord;
	bool regAccess, error, ready, moveDone; //warning
	bool partialLHigh, partialLLow;
	//epicsInt32 loadAngle;

	//update position
	updateCurrentPosition();
	setDoubleParam(pC_->motorPosition_, currPos);

	//update limit switches
	statusWord = pC_->memInp_cache[axisNo_][SW];

	if (limitSwitchDownIsInputOne) {
		partialLHigh = statusWord & 0x2;
		partialLLow = statusWord & 0x1;
	}else {
		partialLHigh = statusWord & 0x1;
		partialLLow = statusWord & 0x2;
	}

	//if the move was started without limit switches auto stop, enable it as soon as out of limit switches
	if (exitingLimSw && (partialLLow < lLow or partialLHigh < lHigh)) {  //falling edges on limit switches readings
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-out of limit switch %s\n", lHigh ? "HIGH" : "LOW");
		controlByteBits_->write(0x20, 0x20);  //enable limit switch auto stop
		exitingLimSw = false;
		if (startingHome) {  //re-launch homing to now perform homing from out of limsw
			startingHome = false;
			lHigh = partialLHigh;
			lLow = partialLLow;
			home(curr_min_velo, curr_home_velo, curr_acc, curr_forw);
		}
	}

	lHigh = partialLHigh;
	lLow = partialLLow;
	setIntegerParam(pC_->motorStatusHighLimit_, lHigh);
	setIntegerParam(pC_->motorStatusLowLimit_, lLow);

	//set status
	statusByte = pC_->memInp_cache[axisNo_][SB];  //read from cache
	regAccess = statusByte & 0x80;
	if(!regAccess) { //should never be one, but just in case...
		error = statusByte & 0x40;
		setIntegerParam(pC_->motorStatusProblem_, error);
		//warning = statusByte & 0x20;
		moveDone = statusByte & 0x10;
		if (moveDone && movePend) {  //movement has just finished
			pC_->poll();
			updateCurrentPosition();
			setDoubleParam(pC_->motorPosition_, currPos);
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-ending position:\t%10.2f %s\n", currPos, (lHigh || lLow) ? (lHigh ? "limit HIGH" : "limit LOW") :"");
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,"-startingHome is %d\n", startingHome);
			if (startingHome) { //not yet out of limit switches, do another movement
				home(curr_min_velo, curr_home_velo, curr_acc, curr_forw);
			}
		}
		movePend = !moveDone;
		*moving = movePend;
		setIntegerParam(pC_->motorStatusDone_, moveDone);
		//loadAngle = statusByte & 0xE;
		ready = statusByte & 0x1;
		setIntegerParam(pC_->motorStatusPowerOn_, ready);
	} else {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,"Warning: registry access - mutex violated!\n");
	}

	//set direction status
	setIntegerParam(pC_->motorStatusDirection_, (lastDir>0) ? 1 : 0 );
	callParamCallbacks();

	return asynSuccess;
}








/**
 * Function to be called by shell command to toggle the configuration methods
 */
BeckController * findBeckControllerByName(const char *name) {
	for(std::vector<BeckController *>::iterator i = _controllers.begin(); i != _controllers.end(); i++ ){
		if (strcmp(name, (*i)->portName) == 0)
			return *i;
	}
	return 0;
}

//parse commands and call each function
extern "C" int BeckConfigController(const char *ctrlName, char *axisRange, const char *cmd, const char *cmdArgs) {
	BeckController *ctrl = findBeckControllerByName(ctrlName);
	if (ctrl == NULL) {
		epicsStdoutPrintf("Cannot find controller %s!\n", ctrlName);
		return 0;
	}

	int axisNumbers[1000];
	int axisListLen=0, i=0;
	const char s[2] = ",";
	char *token;

	// First: parse strings separated by a semicolon
	token = strtok(axisRange, s);
	while( token != NULL )
	{
		// Second: the string may be a range (ex: 3-5) or a single number
		int begin, end;
		int nParsed = sscanf(token, "%d-%d", &begin, &end);

		//If it is a range put in a list all the elements, else only the single number
		if(nParsed>1){
			for (i=0; i<=end-begin; i++) {
				axisNumbers[axisListLen+i] = begin +i;
			}
			axisListLen += end-begin+1;
		} else {
			axisNumbers[axisListLen++] = begin;
		}
		token = strtok(NULL, s);
	}

	//create a beckAxis instance for each axis in list
	int k=0;
	BeckAxis *axis[axisListLen];

	for (i=0; i<axisListLen; i++) {
		BeckAxis *tmp = ctrl->getAxis(axisNumbers[i]);
		if (tmp != NULL) {
			axis[i-k] = tmp;
		}
		else {
			epicsStdoutPrintf("Axis %d notFound ", axisNumbers[i]);
			k++;
		}
	}
	axisListLen -= k;


	//Now parse commands, and apply to list of axis
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
				epicsStdoutPrintf("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}

		}

		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->initCurrents(maxCurr, autoHoldinCurr, highAccCurr, lowAccCurr);
		}
		epicsStdoutPrintf("\n");

	}
	else if (strcmp(cmd, "softReset") ==0 ) {
		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->softReset();
		}
		epicsStdoutPrintf("\n");
	}
	else if (strcmp(cmd, "hardReset") ==0 ) {
		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->hardReset();
		}
		epicsStdoutPrintf("\n");
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
				epicsStdoutPrintf("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}

		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->init((bool) encoder, (bool) watchdog);
		}
		epicsStdoutPrintf("\n");

	}
	else if (strcmp(cmd, "initEncoder") ==0 ) {
			char *pprStr=0;
			char *invertStr=0;

			int nPar = sscanf(cmdArgs, "%m[^,],%m[^,]", &pprStr,
														&invertStr);
			double ppr = 0;
			double invert = 0;

			switch (nPar) {
				case 2: epicsScanDouble(invertStr, &invert);
				case 1: epicsScanDouble(pprStr, &ppr); break;
				default: {
					epicsStdoutPrintf("Wrong number of parameters: %d!\n", nPar);
					return 0;
				}
			}

			epicsStdoutPrintf("Applying to axis: ");
			for (i=0; i<axisListLen; i++){
				epicsStdoutPrintf("%d ", axisNumbers[i]);
				axis[i]->initEncoder((int) ppr, (bool) invert);
			}
			epicsStdoutPrintf("\n");

		}
	else if (strcmp(cmd, "initHomingParams") ==0 ) {
		char *refPositionStr;
		char *NCcontactsStr;
		char *lsDownOneStr;
		char *homeAtStartupStr;
		char *homingSpeedStr;
		char *emergencyAcclStr;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,],%m[^,],%m[^,],%m[^,],%m[^,]", &refPositionStr,
																				&NCcontactsStr,
																				&lsDownOneStr,
																				&homeAtStartupStr,
																				&homingSpeedStr,
																				&emergencyAcclStr);
		double refPosition = 0;
		double NCcontacts = 0;
		double lsDownOne = 0;
		double homeAtStartup = 0;
		double homingSpeed = -1, emergencyAccl = -1;

		switch (nPar) {
			case 6: epicsScanDouble(emergencyAcclStr, &emergencyAccl);
			case 5: epicsScanDouble(homingSpeedStr, &homingSpeed);
			case 4: epicsScanDouble(homeAtStartupStr, &homeAtStartup);
			case 3: epicsScanDouble(lsDownOneStr, &lsDownOne);
			case 2: epicsScanDouble(NCcontactsStr, &NCcontacts);
			case 1: epicsScanDouble(refPositionStr, &refPosition); break;
			default: {
				epicsStdoutPrintf("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}

		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->initHomingParams((int) refPosition, (bool) NCcontacts, (bool) lsDownOne, (int) homeAtStartup, homingSpeed, emergencyAccl);
		}
		epicsStdoutPrintf("\n");
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
				epicsStdoutPrintf("Wrong number of parameters: %d!\n", nPar);
				return 0;
			}
		}

		epicsStdoutPrintf("Applying to axis: ");
		for (i=0; i<axisListLen; i++){
			epicsStdoutPrintf("%d ", axisNumbers[i]);
			axis[i]->initStepResolution((int) microstepPerStep, (int) stepPerRevolution);
		}
		epicsStdoutPrintf("\n");
	}
	else {
		epicsStdoutPrintf("BeckConfigController: Command \"%s\" not found!\n", cmd);
	}
	//ADD: Encoder related fields
	return(asynSuccess);
}

extern "C" int BeckDumpRegs(const char *driverName, const int axis)
{
	asynInt32Client *reg;
	char name[10];
	epicsInt32 val;

	asynInt32Client *statusByte_ = new asynInt32Client(driverName, axis, "SB");
	asynInt32Client *dataIn_ = new asynInt32Client(driverName, axis, "DI");
	asynInt32Client *statusWord_ = new asynInt32Client(driverName, axis, "SW");
	asynInt32Client *controlByte_ = new asynInt32Client(driverName, axis, "CB");
	asynInt32Client *dataOut_ = new asynInt32Client(driverName, axis, "DO");
	asynInt32Client *controlWord_ = new asynInt32Client(driverName, axis, "CW");

	statusByte_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "SB", val);

	dataIn_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "DI", val);

	statusWord_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "SW", val);

	controlByte_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "CB", val);

	dataOut_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "DO", val);

	controlWord_->read(&val);
	epicsStdoutPrintf("%s: 0x%04x\n", "CW", val);

	for (int i=0; i < KL2541_N_REG; i++){
		sprintf(name, "R%02i", i);
		reg = new asynInt32Client(driverName, axis, name);
		reg->read(&val);
		epicsStdoutPrintf("%s: 0x%04x\n", name, val);
	}
    return asynSuccess;
}






/**
 * Code for iocsh registration
 */
static const iocshArg BeckCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateControllerArg1 = {"Driver port name", iocshArgString};
static const iocshArg BeckCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg BeckCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};

static const iocshArg BeckConfigControllerArg0 = {"Controller Name", iocshArgString};
static const iocshArg BeckConfigControllerArg1 = {"Axis Range", iocshArgString};
static const iocshArg BeckConfigControllerArg2 = {"Command", iocshArgString};
static const iocshArg BeckConfigControllerArg3 = {"CommandArgs", iocshArgString};

static const iocshArg BeckDumpRegsArg0 = {"Driver", iocshArgString};
static const iocshArg BeckDumpRegsArg1 = {"Axis", iocshArgInt};

static const iocshArg * const BeckCreateControllerArgs[] = {&BeckCreateControllerArg0,
															&BeckCreateControllerArg1,
															&BeckCreateControllerArg2,
															&BeckCreateControllerArg3};
static const iocshArg * const BeckConfigControllerArgs[] = {&BeckConfigControllerArg0,
															&BeckConfigControllerArg1,
															&BeckConfigControllerArg2,
															&BeckConfigControllerArg3};
static const iocshArg * const BeckDumpRegsArgs[] = {&BeckDumpRegsArg0,
													&BeckDumpRegsArg1};

static const iocshFuncDef BeckCreateControllerDef = {"BeckCreateController", 4, BeckCreateControllerArgs};
static const iocshFuncDef BeckConfigControllerDef = {"BeckConfigController", 4, BeckConfigControllerArgs};
static const iocshFuncDef BeckDumpRegsDef = {"BeckDumpRegs", 2, BeckDumpRegsArgs};

static void BeckCreateControllerCallFunc(const iocshArgBuf *args) {
	BeckCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}
static void BeckConfigControllerCallFunc(const iocshArgBuf *args) {
	BeckConfigController(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}
static void BeckDumpRegsCallFunc(const iocshArgBuf *args) {
	BeckDumpRegs(args[0].sval, args[1].ival);
}

static void BeckControllerRegister(void) {
	iocshRegister(&BeckCreateControllerDef, BeckCreateControllerCallFunc);
	iocshRegister(&BeckConfigControllerDef, BeckConfigControllerCallFunc);
	iocshRegister(&BeckDumpRegsDef, BeckDumpRegsCallFunc);
}
extern "C" {
	epicsExportRegistrar(BeckControllerRegister);
}


//TODO:check negative high values when converting uint-int
