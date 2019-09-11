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
#include <algorithm>

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
#define NO_MASK 0xFFFFFFFF

#include <cmath>

//A vector to store all the controllers, in order to search into it to find controller by its name
static std::vector<BeckController *> _controllers;

//BeckController  :  asynMotorController  :  asynPortDriver
//Represent a number of KL2541 Beckhoff modules
BeckController::BeckController(const char *portName, const char *beckDriverPName, int numAxis, double movingPollPeriod, double idlePollPeriod )
  :  asynMotorController(portName, numAxis, 0,
						 asynUInt32DigitalMask, // Add asynUInt32Digital interface to read single bits
						 asynUInt32DigitalMask, // Add asynUInt32Digital callbacks to read single bits
						 ASYN_CANBLOCK | ASYN_MULTIDEVICE,
						 1, // autoconnect
						 0, 0)	// Default priority and stack size
{
	asynStatus status;
	//static const char *functionName = "BeckController::BeckController";

	//BeckAxis *pAxis; //set but not used not to be eliminated by compiler

	beckDriverPName_ = (char *) mallocMustSucceed(strlen(beckDriverPName)+1, "Malloc failed\n");
	strcpy(beckDriverPName_, beckDriverPName);

	//create each axis of this controller
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"Now create %d axis\n", numAxes_);
	for (int i=0; i<numAxes_; i++){
		new BeckAxis(this, i);
		asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"Axis n: %d successfully created\n", i);
		std::array<epicsInt32, 3> axisRegs;
		memInp_cache.push_back(axisRegs);;
	}

	//arrays to keep values polled from the controller
	//the axis pollers will read this instead of doing modbus I/O -> single big readings instead of small ones = efficiency
	r0_cache = new epicsInt32[numAxes_];
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
	status = r_[0]->read(r0_cache, numAxes_, &nin);
	if (status!=asynSuccess) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Warning: could not initialize r0 cache\n");
	}
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
	//asynPrint(pasynUserSelf, ASYN_TRACE_BECK, "polling...\n");

	size_t nin;
	memInp_->read(memInp_cache.data()->data(), 3*numAxes_, &nin);
	r_[0]->read(r0_cache, numAxes_, &nin);
	r_[1]->read(r1_cache, numAxes_, &nin);

	return asynSuccess;
}

/**
 * Like writeUInt32Digital method of beckDriver, but applied to arrays.
 * Let you write an array of registers, applying a mask on all of them.
 */
asynStatus BeckController::writeUInt32DigitalArray(asynInt32ArrayClient *client, int *value, uint mask, size_t nElements) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%p, %p, %d, %ld)\n", __FUNCTION__, client, value, mask, nElements);

	if (mask!=NO_MASK) { // if NO_MASK the mask would not have any effect, so we avoid doing the useless read
		int rawValue[nElements];
		asynStatus status;
		size_t nIn=0;

		status = client->read(rawValue, nElements, &nIn);
		if (status!=asynSuccess) {
			return status;
		}

		for (size_t i=0; i<nIn; i++) {
			/* Set bits that are set in the value and set in the mask */   // <-- smart guys
			rawValue[i] |=  (value[i] & mask);
			/* Clear bits that are clear in the value and set in the mask */
			rawValue[i]  &= (value[i] | ~mask);
			/* Copy back to value */
			value[i] = rawValue[i];
		}

		nElements = nIn;
	}

	return client->write(value, nElements);
}

/**
 * Like readUInt32Digital method of beckDriver, but applied to arrays.
 * Let you read an array of registers, applying a mask on all of them.
 */
asynStatus BeckController::readUInt32DigitalArray(asynInt32ArrayClient *client, int *value, uint mask, size_t nElements, size_t *nIn) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%p, %p, %d, %ld, %p)\n", __FUNCTION__, client, value, mask, nElements, nIn);
	asynStatus status;

	status = client->read(value, nElements, nIn);
	if (status!=asynSuccess) {
		return status;
	}

	//simply apply a mask over the asynInt32 reading
	for (size_t i=0; i<*nIn; i++) {
		value[i]= value[i] & mask;
	}

	return asynSuccess;

}
/**
 * write the an array of values value on all the registers pointed by a client
 * writes only if necessary: a read is always performed
 * to avoid useless writings to static memory
 */
bool BeckController::writeWithPassword(asynInt32ArrayClient *client, int *value, uint mask, size_t nElem, const char *regName) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%p, %p, %d, %ld, %s)\n", __FUNCTION__, client, value, mask, nElem, regName);

	int password[numAxes_];
	int oldValue[nElem];
	size_t nIn;
	bool toBeUpdated = false;
	asynStatus status;

	status = readUInt32DigitalArray(client, oldValue, mask, nElem, &nIn);
	if (status!=asynSuccess){
		return false;
	}

	for (size_t i=0; i<nIn; i++) {
		if (oldValue[i]!= (int) (value[i] & mask)) {
			toBeUpdated = true;
			asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s \t 0x%04x -> 0x%04x\n", regName, oldValue[i], value[i]);
		}
	}

	if (toBeUpdated) {
		//insert password in all the registers 31 - this enables writing to protected registers
		std::fill_n(password, numAxes_, 0x1235);
		r_[31]->write(password, numAxes_);

		//insert value inside the protected register
		writeUInt32DigitalArray(client, value, mask, nIn);

		//remove password
		std::fill_n(password, numAxes_, 0);
		r_[31]->write(password, numAxes_);
	}

	return toBeUpdated;
}

/**
 * write the same value on all the registers pointed by a client
 */
bool BeckController::writeWithPassword(asynInt32ArrayClient *client, int value, uint mask, size_t nElem, const char *regName) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%p, %d, %d, %ld, %s)\n", __FUNCTION__, client, value, mask, nElem, regName);
	int values[nElem];
	std::fill_n(values, nElem, value);
	return writeWithPassword(client, values, mask, nElem, regName);
}

bool BeckController::axisRangeOk(int *begin, int *end) {
	//check that the range is valid: start<=end and start>=0 and end<=num of axis -1
	if (*begin > *end || *begin >= numAxes_ || *end < 0) return false;
	*begin = (*begin<0) ? 0 : *begin;
	*end = (*end>=numAxes_) ? numAxes_-1 : *end;
	return true;
}

/**
 * To be called by shell command - mandatory
 * Set general parameters
 * encoder = use encoder
 * watchdog = enable watchdog
 * ppr = pulse per revolution
 * invert = if an external encoder is installed opposite the stepper motor (e.g. the encoder shows a negative rotation when the motor rotates in positive direction).
 */
asynStatus BeckController::init(int firstAxis, int lastAxis, bool encoder, bool watchdog, int encoderPpr, bool encoderInvert) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%02d-%02d, %d, %d, %d, %d)\n", __FUNCTION__, firstAxis, lastAxis, encoder, watchdog, encoderPpr, encoderInvert);

	//creating array clients starting at firstAxis
	asynInt32ArrayClient *cb = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "CB");
	asynInt32ArrayClient *r32 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R32");
	asynInt32ArrayClient *r34 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R34");
	asynInt32ArrayClient *r46 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R46");
	asynInt32ArrayClient *r52 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R52");

	size_t axisLen = lastAxis - firstAxis +1;

	//reset precedent errors by putting 0 -> 1 -> 0 to bit 0x40 of control byte
	//and stop all the axis by putting 0 -> 1 -> 0 to bit 0x2 of control byte
	int tobewritten[axisLen];
	std::fill_n(tobewritten, axisLen, 0);
	writeUInt32DigitalArray(cb, tobewritten, 0x42, axisLen);
	std::fill_n(tobewritten, axisLen, 0x42);
	writeUInt32DigitalArray(cb, tobewritten, 0x42, axisLen);
	std::fill_n(tobewritten, axisLen, 0);
	writeUInt32DigitalArray(cb, tobewritten, 0x42, axisLen);

	//in case of encoder microstepping must be disabled
	if (encoder) {
		asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,"-Warning: Using the encoder microstepping is disabled");
		writeWithPassword(r46, 0, NO_MASK, axisLen, "R46 microstep");
	}

	//set encoder variable on all the axis
	for (int i=firstAxis; i<=lastAxis; i++) {
		getAxis(i)->encoderEnabled = encoder;
	}

	//set feature register 1
	epicsInt32 value;
	value = 0x18	//path control mode
		  + 0x2 	//enable autostop
		  + (!encoder<<15) + (!encoder<<11) + (!watchdog<<2) + (encoderInvert << 6);

	writeWithPassword(r32, value, 0x885e, axisLen, "R32 featureReg1");

	//set feature register 2
	value = 0x8 + 0x4;	//enable idle and latching R0 and R1
	writeWithPassword(r52, value, 0xC, axisLen, "R52 featureReg2");

	if (encoder) {
		//set reg 34 = number of increments issued by the encoder connected to the KL2541 during a complete turn (default: 4000).
		encoderPpr = encoderPpr * 4.0;  //this is a quadrature encoder
		writeWithPassword(r34, encoderPpr, NO_MASK, axisLen, "R34 increments per revolution");
	}

	return asynSuccess;
}

/**
 * To be called by shell command
 * Set the step resolution of the controller
 */
asynStatus BeckController::initStepResolution(int firstAxis, int lastAxis, int microstepPerStep, int stepPerRevolution){
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%02d-%02d, %d, %d)\n", __FUNCTION__, firstAxis, lastAxis, microstepPerStep, stepPerRevolution);

	//creating array clients starting at firstAxis
	asynInt32ArrayClient *r46 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R46");
	asynInt32ArrayClient *r33 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R33");

	size_t axisLen = lastAxis - firstAxis +1;
	int tobewritten[axisLen];

	if (microstepPerStep>0) {
		if (microstepPerStep > 64) {
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-WARNING: Maximum microstep resolution is 64, setting 64!\n");
			microstepPerStep = 64;
		}
		if (microstepPerStep < 1) {
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-WARNING: Minimum microstep resolution is 1, setting 1!\n");
			microstepPerStep = 1;
		}

		//calculate value to be written in HW, as of beckhoff datasheet
		microstepPerStep = round(log2(microstepPerStep));
		//save the actual value as variable on each axis
		for (int i=firstAxis; i<=lastAxis; i++){
			BeckAxis *a = getAxis(i);
			if (a->encoderEnabled and microstepPerStep!=0){
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02d WARNING: Cannot use microstepping with encoder - reverting to 1 microstepPerStep\n", i);
				tobewritten[i] = 0; //means 1 microstep per step (fullstep)
			} else {
				tobewritten[i]=microstepPerStep;
			}
			a->microstepPerStep = pow(2.0, tobewritten[i]);

		}

		writeWithPassword(r46, tobewritten, NO_MASK, axisLen, "R46 microstep");
	}

	if (stepPerRevolution>0) {
		writeWithPassword(r33, stepPerRevolution, NO_MASK, axisLen, "R33 stepPerRevolution");
	}

	return asynSuccess;
}

/**
 * To be called by shell command
 * Set coil currents of the motor
 */
asynStatus BeckController::initCurrents(int firstAxis, int lastAxis, double maxAmp, double autoHoldinCurr, double highAccCurr, double lowAccCurr) {
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%02d-%02d, %.2f, %.2f, %.2f, %.2f)\n", __FUNCTION__, firstAxis, lastAxis, maxAmp, autoHoldinCurr, highAccCurr, lowAccCurr);

	//creating array clients starting at firstAxis
	asynInt32ArrayClient *r35 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R35");
	asynInt32ArrayClient *r36 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R36");
	asynInt32ArrayClient *r42 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R42");
	asynInt32ArrayClient *r43 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R43");
	asynInt32ArrayClient *r44 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R44");

	size_t axisLen = lastAxis - firstAxis +1;
	epicsInt32 termType[numAxes_];
	double fullScaleCurr[axisLen];
	double setMaxAmp[axisLen];
	int percent[axisLen];
	size_t nIn;

	//read controller code and convert ampere to %
	r_[8]->read(termType, numAxes_, &nIn);

	for (size_t i=0; i<axisLen; i++) {
		switch (termType[firstAxis+i]) {
			case 2531: fullScaleCurr[i] = 1.5; break;
			case 2541: fullScaleCurr[i] = 5.0; break;
			default: {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02ld Error: Cannot recognize controller type %d\n", firstAxis+i, termType[firstAxis+i]);
				return asynError;
			}
		}
		asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%02ld Terminal: Beckhoff KL%d  -  Max available ampere: %.2lf\n", firstAxis+i, termType[firstAxis+i], fullScaleCurr[i]);

		//R35: Maximum coil current A (in % to fullScale of device)
		//R36: Maximum coil current B (in % to fullScale of device)
		if (maxAmp>=0){
			if (maxAmp>fullScaleCurr[i]) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02ld Warning: Cannot set max current higher than full scale, reverting to %.2lfA\n", firstAxis+i, fullScaleCurr[i]);
			}
			percent[i] = round( std::min(maxAmp, fullScaleCurr[i]) / fullScaleCurr[i] *100 ); //enforce here fullScaleCurr as higher limit
			setMaxAmp[i] = ((double) percent[i])/100.0*fullScaleCurr[i];
		}

	}

	//update values
	if (maxAmp>0) {
		writeWithPassword(r35, percent, NO_MASK, axisLen, "R35 max coilA curr");
		writeWithPassword(r36, percent, NO_MASK, axisLen, "R36 max coilB curr");
	}

	//R44: Coil current, v = 0 (automatic) (in % to maxAmp)
	if (autoHoldinCurr>=0) {
		for (size_t i=0; i<axisLen; i++) {
			if (autoHoldinCurr>setMaxAmp[i]) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02ld Warning: Cannot set holding current higher than maximum coil current, reverting to %.2lfA\n", firstAxis+i, setMaxAmp[i]);
			}
			percent[i] = round( std::min(setMaxAmp[i], autoHoldinCurr) / setMaxAmp[i] *100 );
		}
		writeWithPassword(r44, percent, NO_MASK, axisLen, "R44 auto holding current");
	}

	//R42: Coil current, a > ath (in % to maxAmp)
	if (highAccCurr>=0) {
		for (size_t i=0; i<axisLen; i++) {
			if (highAccCurr>setMaxAmp[i]) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02ld Warning: Cannot set over acceleration current higher than maximum coil current, reverting to %.2lfA\n", firstAxis+i, setMaxAmp[i]);
			}
			percent[i] = round( std::min(setMaxAmp[i], highAccCurr) / setMaxAmp[i] *100 );
		}
		writeWithPassword(r42, percent, NO_MASK, axisLen, "R42 holding current a>a_th");
	}

	//R43: Coil current, a <= ath (in % to maxAmp)
	if (lowAccCurr>=0) {
		for (size_t i=0; i<axisLen; i++) {
			if (lowAccCurr>setMaxAmp[i]) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-%02ld Warning: Cannot set sub acceleration current higher than maximum coil current, reverting to %.2lfA\n\n", firstAxis+i, setMaxAmp[i]);
			}
			percent[i] = round( std::min(setMaxAmp[i], lowAccCurr) / setMaxAmp[i] *100 );
		}
		writeWithPassword(r43, percent, NO_MASK, axisLen, "R43 holding current a<=a_th");
	}

	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"Currents written to the controller!\n");
	return asynSuccess;
}

/**
 * To be called by shell command
 * Set parameters for the homing procedure
 */
asynStatus BeckController::initHomingParams(int firstAxis, int lastAxis, int refPosition, bool NCcontacts, bool lsDownOne, int homeAtStartup, double homingSpeed, double emergencyAccl){
	asynPrint(pasynUserSelf, ASYN_TRACE_BECK,"-%s(%02d-%02d, %d, %d, %d, %d, %.2f, %.2f)\n", __FUNCTION__, firstAxis, lastAxis, refPosition, NCcontacts, lsDownOne, homeAtStartup, homingSpeed, emergencyAccl);

	//creating array clients starting at firstAxis
	asynInt32ArrayClient *r50 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R50");
	asynInt32ArrayClient *r52 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R52");
	asynInt32ArrayClient *r53 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R53");
	asynInt32ArrayClient *r54 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R54");
	asynInt32ArrayClient *r55 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R55");
	asynInt32ArrayClient *r56 = new asynInt32ArrayClient(beckDriverPName_, firstAxis, "R56");

	size_t axisLen = lastAxis - firstAxis +1;
	epicsUInt32 featureReg;

	writeWithPassword(r55, refPosition & 0xFFFF, NO_MASK, axisLen, "R55 reference position (low word)");
	writeWithPassword(r56, (refPosition>>16) & 0xFFFF, NO_MASK, axisLen, "R56 reference position (high word)");

	featureReg = (NCcontacts<<15) + (NCcontacts<<14);
	if (writeWithPassword(r52, featureReg, 0xC000, axisLen, "R52 featureReg2")){
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"-WARNING: changing type of contacts usually requires a reboot or a softReset of the Beckhoff module!\n");
	}

	if (homingSpeed>=0){
		writeWithPassword(r53, (int) homingSpeed, NO_MASK, axisLen, "R53 homing speed");
		writeWithPassword(r54, (int) homingSpeed, NO_MASK, axisLen, "R54 homing speed");
	}

	if (emergencyAccl>=0){
		writeWithPassword(r50, (int) emergencyAccl, NO_MASK, axisLen, "R50 emergency acceleration");
	}

	for (int i=firstAxis; i<=lastAxis; i++){
		if (homeAtStartup!=0) {
			getAxis(i)->home(100, 500, 1000, (homeAtStartup>0) ? 1 : 0);
		}

		getAxis(i)->limitSwitchDownIsInputOne = lsDownOne;
	}

	return asynSuccess;
}


//function called from iocsh to create a controller and save it into the _controllers vector
extern "C" int BeckCreateController(const char *portName, const char *beckDriverPName, int movingPollPeriod, int idlePollPeriod ) {
	int maxAddr = getBeckMaxAddr(beckDriverPName);
	if (maxAddr>0) {
		BeckController *ctrl = new BeckController(portName, beckDriverPName, maxAddr, movingPollPeriod/1000., idlePollPeriod/1000.);
		epicsStdoutPrintf("OK: Controller %p\n", ctrl);
		_controllers.push_back(ctrl);
		return(asynSuccess);
	}
	epicsStdoutPrintf("ERROR: No axis available, skipping controller creation.\n");
	return(asynError);
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
	encoderEnabled = false;
	curr_min_velo = 0;  //mSteps
	curr_max_velo = 0;  //mSteps
	curr_home_velo = 0; //mSteps
	curr_acc = 0;       //mSteps
	curr_forw = -1;     // illegal value to trigger first writing in any case
	startingHome = false;
	exitingLimSw = false;
	microstepPerStep = 64;
	lHigh = false;
	lLow = false;
	currPos = 0; //mSteps
	lastDir = 0;

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
	
	pLow = pC_->r0_cache[axisNo_];
	pHigh = pC_->r1_cache[axisNo_];

	currPos = (pLow + (pHigh<<16)) / (3.0*encoderEnabled+1.0); //when encoder is enabled divide by 4.0
	if (movePend) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK, "%d high: %6d   --  low: %6d  --  tot: %10.2f %1d\n", axisNo_, pHigh, pLow, currPos, encoderEnabled);
	}
	return asynSuccess;
}

//convenient function to set acceleration and velocity of the motor
asynStatus BeckAxis::setAcclVelo(double min_velocity_step, double max_velocity_step, double acceleration_step) {
	
	// Convert from record motor convention (STEPS) to beckhoff module convention (MICRO STEPS)
	auto min_velocity_mstep = microstepPerStep * min_velocity_step;
	auto max_velocity_mstep = microstepPerStep * max_velocity_step;
	auto acceleration_mstep = microstepPerStep * acceleration_step;
	
	//info at http://infosys.beckhoff.com/italiano.php?content=../content/1040/bk9000/html/bt_bk9000_title.htm&id=259
	if (min_velocity_mstep!=curr_min_velo) {
		curr_min_velo = min_velocity_mstep;
		min_velocity_mstep = (int) (min_velocity_mstep * 0.016384);  //vel=mstep/sec/16Mhz*262144
		r_[38]->write(min_velocity_mstep);
	}
	if (max_velocity_mstep!=curr_max_velo) {
		curr_max_velo = max_velocity_mstep;
		max_velocity_mstep = (int) (max_velocity_mstep * 0.016384);
		r_[39]->write(max_velocity_mstep);
	}
	if (acceleration_mstep!=curr_acc) {
		curr_acc = acceleration_mstep;
		acceleration_mstep = (int) (acceleration_mstep * 1.073742/1000);  //accl = mstep/s^2*2^38/(16Mhz)^2
		r_[40]->write(acceleration_mstep);
		r_[58]->write(acceleration_mstep);
	}
	return asynSuccess;
}

/**
 * To be called by shell command
 * Restore the BEckhoff module to its factory settings
 */
asynStatus BeckAxis::hardReset() {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"Attention - Restoring factory setting of axis %d!\n", axisNo_);
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
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"Attention - Restoring saved setting of axis %d!\n", axisNo_);
	r_[31]->write(0x1235);
	r_[7]->write(0x8000);
	r_[31]->write(0);
	return asynSuccess;
}

//Method to execute movement
//Parameters received are in steps (not mSteps)
asynStatus BeckAxis::move(double position_step, int relative, double min_velocity, double max_velocity, double acceleration)
{
	//TODO check for overflow in relative mode
	//epicsStdoutPrintf("-%s(%.2f, %i, %.2f, %.2f, %.2f)\n", __FUNCTION__, position, relative, min_velocity, max_velocity, acceleration);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-%02d %s(%.2f, %i, %.2f, %.2f, %.2f)\n", axisNo_, __FUNCTION__, position_step, relative, min_velocity, max_velocity, acceleration);

	//This to prevent to put movePend to 1 if cannot move
	if (movePend) return asynSuccess;

	setAcclVelo(min_velocity, max_velocity, acceleration);
	exitingLimSw = false;

	int newPos = (relative ? currPos : 0 ) + position_step * microstepPerStep;
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

	int newr2 = (newPos*(3*encoderEnabled+1)) & 0xFFFF;	//when encoder is enabled multiply by 4.0
	int newr3 = ((newPos*(3*encoderEnabled+1))>>16) & 0xFFFF;
	r_[2]->write(newr2);
	r_[3]->write(newr3);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-R2: %d R3: %d \n", newr2, newr3);
	
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
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-starting position:\t%10.2f\n", currPos);
	int curr_controByte;
	controlByte_->read(&curr_controByte);  //read from cache
	if (curr_controByte & 0x4) {  //we need a rising edge on bit 2 of controlByte -- if it is currently at 1, first zero it, then put to 1
		controlByte_->write(0x1);
	}
	controlByte_->write(goCmd);  //the movement should now start
	epicsThreadSleep(0.1); //wait at least 100ms before polling to let the controller update moveDone bit
	return asynSuccess;
}

//Method to execute the homing
//Parameters received are in steps (not mSteps)
asynStatus BeckAxis::home(double min_velocity, double home_velocity, double acceleration, int forward){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-%02d %s(%.2f, %.2f, %.2f, %d)\n", axisNo_, __FUNCTION__, min_velocity, home_velocity, acceleration, forward);
	//epicsStdoutPrintf("-%s(%.2f, %.2f, %.2f, %d)\n", __FUNCTION__, min_velocity, home_velocity, acceleration, forward);
	startingHome = false;

	//set runtime parameters
	setAcclVelo(min_velocity, curr_max_velo/microstepPerStep, acceleration);
	auto home_velocity_mstep = microstepPerStep * home_velocity;

	//set homing velocity and direction
	if (home_velocity_mstep!=curr_home_velo or forward!=curr_forw){
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-%s: updating velocity and direction\n", __FUNCTION__);
		r_[31]->write(0x1235);  //enable write to protected regs
		if (home_velocity_mstep!=curr_home_velo) {
			curr_home_velo = home_velocity_mstep;
			home_velocity_mstep = (int) home_velocity_mstep * 0.016384;  //vel=mstep/sec/16Mhz*262144
			r_[53]->write(home_velocity_mstep);
			r_[54]->write(home_velocity_mstep);
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
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-%s: limits switch, moving out...\n", __FUNCTION__);
		lastDir = lLow ? -1 : 1;
		move(-lastDir*OUTOFSWITCH_STEPS, 1, curr_min_velo/microstepPerStep, curr_max_velo/microstepPerStep, curr_acc/microstepPerStep);  //exit limit switches
		startingHome = true;
		return asynSuccess;
	}

	//start homing
	dataOut_->write(0);
	controlByte_->write(1);
	r_[7]->write(0x520);

	movePend=true;
	controlByte_->write(0x25);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"Homing started!! ----------------------------------------------------\n");

	//update lastDir, it is the contrary of the homing direction, as the homing ends moving away from limit switch
	lastDir = forward ? -1 : 1;
	epicsThreadSleep(0.050); //wait at least 50ms before polling to let the controller update moveDone bit
	return asynSuccess;
}

//Method to stop motor movement
asynStatus BeckAxis::stop(double acceleration){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-%02d %s(%.2f) -moving: %d\n", axisNo_, __FUNCTION__, acceleration, movePend);
	//epicsStdoutPrintf("- %02d %s ( accl: %lf ) \n", axisNo_, __FUNCTION__,  acceleration);

	controlByteBits_->write(0x2, 0x6); // CB.1=1 CB.2=0
	controlByteBits_->write(0, 0x2);   // CB.1=0

	return asynSuccess;
}

asynStatus BeckAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"- %02d %s ( minvel: %lf maxvel: %lf accl %lf ) \n", axisNo_, __FUNCTION__,  minVelocity, maxVelocity, acceleration);
	//epicsStdoutPrintf("- %02d %s ( minvel: %lf maxvel: %lf accl %lf ) \n", axisNo_, __FUNCTION__,  minVelocity, maxVelocity, acceleration);

	setAcclVelo(minVelocity, maxVelocity, acceleration);

	epicsInt32 val;

	//dataout must be zeroed to restart a new movement with same speed as before (so it can see changement)
	dataOut_->read(&val);
	if (val!=0 && not movePend) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK, "-reset dataOut to 0\n");
		dataOut_->write(0x0);
	}
	//a cycle of low-high or simply a rising edge on bit 0x20
	controlByte_->read(&val);
	if (val!=1  && not movePend) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-reset controlByte to 1\n");
		controlByte_->write(0x1);
	}
	controlByte_->write(0x21);

	epicsInt32 velocity = (int) (maxVelocity * microstepPerStep / 3.812951);  //conversion for beckhoff
	//limit to 15 bit + sign register
	if (velocity > 32767) {
		velocity = 32767;
	}
	else if (velocity < -32768) {
		velocity = -32768;
	}

	//start movement
	movePend = true;
	dataOut_->write(velocity);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-starting position:\t%10.2f velocity %d\n", currPos, velocity);
	epicsThreadSleep(0.050); //wait at least 50ms before polling to let the controller update moveDone bit
	return asynSuccess;
}

asynStatus BeckAxis::setPosition(double position){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"- %02d %s ( pos: %lf ) \n", axisNo_, __FUNCTION__,  position);
	return asynSuccess;
}

asynStatus BeckAxis::setEncoderPosition(double position){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"- %02d %s ( pos: %lf ) \n", axisNo_, __FUNCTION__,  position);
	return asynSuccess;
}

asynStatus BeckAxis::setEncoderRatio(double ratio){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"- %02d %s ( ratio: %lf ) \n", axisNo_, __FUNCTION__,  ratio);
	return asynSuccess;
}

asynStatus BeckAxis::doMoveToHome(){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"- %02d %s ( ) \n", axisNo_, __FUNCTION__);
	return asynSuccess;
}

//Poller to update motor status on the record
asynStatus BeckAxis::poll(bool *moving) {
	//epicsStdoutPrintf("- %02d poll -- homing: %d  -- exitingLimSw: %d\n", axisNo_, startingHome, exitingLimSw);
	epicsInt32 statusByte, statusWord;
	bool regAccess, error, ready, moveDone, justDone; //warning
	bool partialLHigh, partialLLow;
	//epicsInt32 loadAngle;

	//update position
	updateCurrentPosition();
	setDoubleParam(pC_->motorPosition_, currPos/microstepPerStep);  // tell motor record value in step (currPos in mStep)

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
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-out of limit switch %s\n", lHigh ? "HIGH" : "LOW");
		controlByteBits_->write(0x20, 0x20);  //enable limit switch auto stop
		exitingLimSw = false;
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
		justDone = moveDone && movePend;
		movePend = !moveDone;
		if (justDone) {  //movement has just finished
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_BECK,"-ending position:\t%10.2f %s %s\n", currPos, (lHigh || lLow) ? (lHigh ? "limit HIGH" : "limit LOW") :"", startingHome ? "homing unfinished":"");
			if (startingHome) { //not yet out of limit switches, do another movement
				home(curr_min_velo/microstepPerStep, curr_home_velo/microstepPerStep, curr_acc/microstepPerStep, curr_forw);
				moveDone = movePend; //has been be changed by home()
			}
		}
		*moving = movePend;
		setIntegerParam(pC_->motorStatusDone_, moveDone);
		setIntegerParam(pC_->motorStatusMoving_, movePend);
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
extern "C" int BeckConfigController(const char *ctrlName, char *axisRangeStr, const char *cmd, const char *cmdArgs) {
	BeckController *ctrl = findBeckControllerByName(ctrlName);
	if (ctrl == NULL) {
		epicsStdoutPrintf("-ERROR: Cannot find controller %s!\n", ctrlName);
		return asynError;
	}

	int axisNumbers[100][2];
	int axisListLen=0;
	const char s[2] = ",";
	char *token;

	// First: parse strings separated by a semicolon
	token = strtok(axisRangeStr, s);
	while( token != NULL )
	{
		// Second: the string may be a range (ex: 3-5) or a single number
		int begin, end;
		int nParsed = sscanf(token, "%d-%d", &begin, &end);

		axisNumbers[axisListLen][0] = begin;
		if (nParsed<2) {
			end = begin;
		}
		axisNumbers[axisListLen][1] = end;
		axisListLen++;
		token = strtok(NULL, s);
	}

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
				epicsStdoutPrintf("-ERROR: Wrong number of parameters: %d!\n", nPar);
				return asynError;
			}

		}

		for (int i=0; i<axisListLen; i++){
			if (ctrl->axisRangeOk(&(axisNumbers[i][0]), &(axisNumbers[i][1]))) {
				epicsStdoutPrintf("-Applying to axis range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
				ctrl->initCurrents(axisNumbers[i][0], axisNumbers[i][1], maxCurr, autoHoldinCurr, highAccCurr, lowAccCurr);
			} else {
				epicsStdoutPrintf("-ERROR: Invalid range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
			}
		}

	}
	else if (strcmp(cmd, "softReset") ==0 ) {
		for (int i=0; i<axisListLen; i++){
			for (int j=axisNumbers[i][0]; j<=axisNumbers[i][1]; j++){
				epicsStdoutPrintf("-%02d Soft reset... \n", j);
				ctrl->getAxis(j)->softReset();
			}

		}
	}
	else if (strcmp(cmd, "hardReset") ==0 ) {
		for (int i=0; i<axisListLen; i++){
			for (int j=axisNumbers[i][0]; j<=axisNumbers[i][1]; j++){
				epicsStdoutPrintf("-%02d Hard reset... \n", j);
				ctrl->getAxis(j)->hardReset();
			}

		}
	}
	else if (strcmp(cmd, "init") ==0 ) {
		char *encoderStr=0;
		char *watchdogStr=0;
		char *encoderPprStr=0;
		char *encoderInvertStr=0;

		int nPar = sscanf(cmdArgs, "%m[^,],%m[^,],%m[^,],%m[^,]", &encoderStr,
																  &watchdogStr,
																  &encoderPprStr,
																  &encoderInvertStr);
		double encoder = 0;
		double watchdog = 0;
		double encoderPpr = 400;
		double encoderInvert = 0;

		switch (nPar) {
			case 4: epicsScanDouble(encoderInvertStr, &encoderInvert);
			case 3: epicsScanDouble(encoderPprStr, &encoderPpr);
			case 2: epicsScanDouble(watchdogStr, &watchdog);
			case 1: epicsScanDouble(encoderStr, &encoder); break;
			default: {
				epicsStdoutPrintf("-ERROR: Wrong number of parameters: %d!\n", nPar);
				return asynError;
			}
		}

		if (nPar<3 and encoder!=0) {
			epicsStdoutPrintf("-ERROR: Pulse per revolution parameter is mandatory when using encoder!\n");
			return asynError;
		}

		for (int i=0; i<axisListLen; i++){
			if (ctrl->axisRangeOk(&(axisNumbers[i][0]), &(axisNumbers[i][1]))) {
				epicsStdoutPrintf("-Applying to axis range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
				ctrl->init(axisNumbers[i][0], axisNumbers[i][1], (bool) encoder, (bool) watchdog, (int) encoderPpr, (bool) encoderInvert);
			} else {
				epicsStdoutPrintf("-ERROR: Invalid range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
			}
		}

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
				epicsStdoutPrintf("-ERROR: Wrong number of parameters: %d!\n", nPar);
				return asynError;
			}
		}

		for (int i=0; i<axisListLen; i++){
			if (ctrl->axisRangeOk(&(axisNumbers[i][0]), &(axisNumbers[i][1]))) {
				epicsStdoutPrintf("-Applying to axis range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
				ctrl->initHomingParams(axisNumbers[i][0], axisNumbers[i][1], (int) refPosition, (bool) NCcontacts, (bool) lsDownOne, (int) homeAtStartup, homingSpeed, emergencyAccl);
			} else {
				epicsStdoutPrintf("-ERROR: Invalid range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
			}
		}

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
				epicsStdoutPrintf("-ERROR: Wrong number of parameters: %d!\n", nPar);
				return asynError;
			}
		}

		for (int i=0; i<axisListLen; i++){
			if (ctrl->axisRangeOk(&(axisNumbers[i][0]), &(axisNumbers[i][1]))) {
				epicsStdoutPrintf("-Applying to axis range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
				ctrl->initStepResolution(axisNumbers[i][0], axisNumbers[i][1], (int) microstepPerStep, (int) stepPerRevolution);
			} else {
				epicsStdoutPrintf("-ERROR: Invalid range %02d -> %02d \n", axisNumbers[i][0], axisNumbers[i][1]);
			}
		}

	}
	else {
		epicsStdoutPrintf("-ERROR: BeckConfigController: Command \"%s\" not found!\n", cmd);
		return asynError;
	}
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
