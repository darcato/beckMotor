/*
 * BeckPortDriver.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: davide
 */

#include "BeckPortDriver.h"
#include <epicsExport.h>
#include <iocsh.h>
#include <string.h>
#include <dbAccess.h>
#include <epicsThread.h>
//#include <unistd.h> //for usleep

#define KL2541_N_REG 64
#define MAX_TRY 2


static std::vector<BeckPortDriver *> _drivers;


int getBeckMaxAddr(const char *portName) {
	for(std::vector<BeckPortDriver *>::iterator i=_drivers.begin(); i != _drivers.end(); i++) {
		if(strcmp((*i)->portName, portName) == 0) {
			printf("getBeckMaxAddr -- OK: found: %d controllers!\n",(*i)->maxAddr);
			return (*i)->maxAddr;
		}
	}
	printf("getBeckMaxAddr -- Error: Cannot find the underlying port!\n");
	return -1;
}


BeckPortDriver::BeckPortDriver(const char *portName, int nAxis, const char *inModbusPort, const char *outModbusPort)
	: asynPortDriver( portName, nAxis, KL2541_N_REG+6, asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, asynInt32Mask | asynUInt32DigitalMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
	  triggerReadIn_(inModbusPort, 0, "MODBUS_READ"),
	  newDataIn_(inModbusPort, 0, "MODBUS_DATA")
{
	char name[10];
	int tmp;
	createParam("R00", asynParamInt32, &roffset);
	printf("Create parameter %s with reason %d\n", "R00", roffset);
	for (int i=1; i < KL2541_N_REG; i++){
		sprintf(name, "R%02i", i);
		createParam(name, asynParamInt32, &tmp);
		printf("Create parameter %s with reason %d\n", name, tmp);
	}

	createParam("SB", asynParamInt32, &statusByteIndx_);
	createParam("DI", asynParamInt32, &dataInIndx_);
	createParam("SW", asynParamInt32, &statusWordIndx_);
	createParam("CB", asynParamInt32, &controlByteIndx_);
	createParam("DO", asynParamInt32, &dataOutIndx_);
	createParam("CW", asynParamInt32, &controlWordIndx_);


	for(int i=0; i<nAxis; i++){
		statusByte_.push_back(new asynInt32Client(inModbusPort, i*3+0, "MODBUS_DATA"));
		dataIn_.push_back(new asynInt32Client(inModbusPort, i*3+1, "MODBUS_DATA"));
		statusWord_.push_back(new asynInt32Client(inModbusPort, i*3+2, "MODBUS_DATA"));
		controlByte_.push_back(new asynInt32Client(outModbusPort, i*3+0, "MODBUS_DATA"));
		dataOut_.push_back(new asynInt32Client(outModbusPort, i*3+1, "MODBUS_DATA"));
		controlWord_.push_back(new asynInt32Client(outModbusPort, i*3+2, "MODBUS_DATA"));

		controlByteInitialized_.push_back(false);
		dataOutInitialized_.push_back(false);
		controlWordInitialized_.push_back(false);

		controlByteValue_.push_back(0);
		dataOutValue_.push_back(0);
		controlWordValue_.push_back(0);
	}
}

asynStatus BeckPortDriver::readInt32(asynUser *pasynUser, epicsInt32 *value) {
	//printf("readInt32 with user %p and reason %d\n", pasynUser, pasynUser->reason);
	epicsInt32 axis;
	getAddress(pasynUser, &axis);

	if (pasynUser->reason < KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		//printf("Read Register %d\n", pasynUser->reason-roffset);
		return readReg(pasynUser->reason-roffset, axis, value);

	} else if (pasynUser->reason == statusByteIndx_){
		return readProc(statusByte_[axis], 1, value);
		//printf("Read Status Byte - 0x%x\n", *value);

	} else if (pasynUser->reason == dataInIndx_){
		return readProc(dataIn_[axis], 1, value);
		//printf("Read DataIn - 0x%x\n", *value);

	} else if (pasynUser->reason == statusWordIndx_){
		return readProc(statusWord_[axis], 1, value);
		//printf("Read Status Word - 0x%x\n", *value);

	} else if (pasynUser->reason == controlByteIndx_){
		if (controlByteInitialized_[axis]) {
			*value = controlByteValue_[axis];
			return asynSuccess;
		} else {
			asynStatus status;
			status = controlByte_[axis]->read(value);
			if (status == asynSuccess) {
				controlByteInitialized_[axis] = true;
				printf("controlByte initialized with value 0x%04x\n", *value);
			}
			return status;
		}

	} else if (pasynUser->reason == dataOutIndx_){
		if (dataOutInitialized_[axis]) {
			*value = dataOutValue_[axis];
			return asynSuccess;
		} else {
			asynStatus status;
			status = dataOut_[axis]->read(value);
			if (status == asynSuccess) {
				dataOutInitialized_[axis] = true;
				printf("dataOut initialized with value 0x%04x\n", *value);
			}
			return status;
		}

	} else if (pasynUser->reason == controlWordIndx_){
		if (controlWordInitialized_[axis]) {
			*value = controlWordValue_[axis];
			return asynSuccess;
		} else {
			asynStatus status;
			status = controlWord_[axis]->read(value);
			if (status == asynSuccess) {
				controlWordInitialized_[axis] = true;
				printf("controlWord initialized with value 0x%04x\n", *value);
			}
			return status;
		}

	} else {
		printf("Error: Wrong reason!\n");
		return asynError;
	}
}

asynStatus BeckPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	epicsInt32 axis;
	getAddress(pasynUser, &axis);
	//printf("writeInt32 with value 0x%04x %i\n", value, axis);

	if (pasynUser->reason <= KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		//printf("Write Register %d - 0x%x\n", pasynUser->reason-roffset, value);
		return writeReg(pasynUser->reason-roffset, axis, value);

	} else if (pasynUser->reason == controlByteIndx_){
		//printf("Write Control Byte - 0x%x\n", value);
		controlByteValue_[axis] = value;
		controlByteInitialized_[axis] = true;
		return writeProc(controlByte_[axis], value);

	} else if (pasynUser->reason == dataOutIndx_){
		//printf("Write DataOut - 0x%x\n", value);
		dataOutValue_[axis] = value;
		dataOutInitialized_[axis] = true;
		return writeProc(dataOut_[axis], value);

	} else if (pasynUser->reason == controlWordIndx_){
		//printf("Write Control Word - 0x%x\n", value);
		controlWordValue_[axis] = value;
		controlWordInitialized_[axis] = true;
		return writeProc(controlWord_[axis], value);

	} else {
		printf("Error: Wrong reason!\n");
		return asynError;
	}
}


asynStatus BeckPortDriver::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask) {
	//printf("readUInt32Digital with user %p, reason %d and mask 0x%04x\n", pasynUser, pasynUser->reason, mask);

	epicsInt32 rawValue;
	asynStatus status;

	status = readInt32(pasynUser, &rawValue);
	if (status!=asynSuccess) {
		return status;
	}

	*value = rawValue & mask;
	//printf ("Read 0x%04x with mask 0x%04x becomes 0x%04x\n", rawValue, mask, *value);
	return asynSuccess;
}

asynStatus BeckPortDriver::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask) {
	printf("writeUInt32Digital with user %p, reason %d, value %d, and mask 0x%04x\n", pasynUser, pasynUser->reason, value, mask);

	epicsInt32 rawValue;
	asynStatus status;

	status = readInt32(pasynUser, &rawValue);
	if (status!=asynSuccess) {
		return status;
	}

	/* Set bits that are set in the value and set in the mask */
	rawValue |=  (value & mask);
	/* Clear bits that are clear in the value and set in the mask */
	rawValue  &= (value | ~mask);

	return writeInt32(pasynUser, rawValue);
}

asynStatus BeckPortDriver::writeReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 value) {
	//printf("writeReg %d of axis %d with value 0x%x\n", regN, axis, value);
	epicsInt32 readbackValue, k=0;
	epicsInt32 writeCmd = regN + 0xC0;
	do {
		writeProc(controlByte_[axis], 0x80);
		writeProc(dataOut_[axis], value);
		writeProc(controlByte_[axis], writeCmd);
		if (interruptAccept) {
			readReg(regN, axis, &readbackValue);
		} else {
			readbackValue = value;
			writeProc(controlByte_[axis], 0x80);
		}
		if (readbackValue == value)
			return asynSuccess;
	} while (k++ <= MAX_TRY);

	printf("Axis %d: Cannot correctly write [%d=0x%x] in register %d - Value in hardware: %d\n", axis, value, value, regN, readbackValue);
	return asynError;
}

asynStatus BeckPortDriver::readReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 *value){
	//printf("readReg %d of axis %d\n", regN, axis);
	asynStatus status;
	epicsInt32 readCmd = regN + 0x80;
	status = writeProc(controlByte_[axis], readCmd);
	if (status!=asynSuccess) {
		return status;
	}
	return readProc(dataIn_[axis], 1, value);
}

asynStatus BeckPortDriver::writeProc(asynInt32Client *modbusReg, epicsInt32 value) {
	//printf("writeProc with value 0x%x\n", value);
	return modbusReg->write(value);
}

asynStatus BeckPortDriver::readProc(asynInt32Client *modbusReg, bool in, epicsInt32 *value) {
	//printf("readProc... %s \n", in ? "input" : "output");
	if (!in) return asynError;

	if (interruptAccept) {
		triggerReadIn_.write(0);
		newDataIn_.readWait(value);
	} else {
		printf("Warning, interruptions not yet allowed... skipping reading!\n");
		return asynError;
	}
	return modbusReg->read(value);
}



extern "C" int BeckCreateDriver(const char *portName, const int numAxis, const char *inModbusPName, const char *outModbusPName)
{
	BeckPortDriver *drv = new BeckPortDriver(portName, numAxis, inModbusPName, outModbusPName);
    printf("Driver %p created!\n", drv);
	_drivers.push_back(drv);
    return asynSuccess;
}


/**
 * Code for iocsh registration
 */
static const iocshArg BeckCreateDriverArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateDriverArg1 = {"Number of consecutive controllers", iocshArgInt};
static const iocshArg BeckCreateDriverArg2 = {"Modbus INPUT port name", iocshArgString};
static const iocshArg BeckCreateDriverArg3 = {"Modbus OUTPUT port name", iocshArgString};

static const iocshArg * const BeckCreateDriverArgs[] = {&BeckCreateDriverArg0,
                                                        &BeckCreateDriverArg1,
                                                        &BeckCreateDriverArg2,
                                                        &BeckCreateDriverArg3};

static const iocshFuncDef BeckCreateDriverDef = {"BeckCreateDriver", 4, BeckCreateDriverArgs};

static void BeckCreateDriverCallFunc(const iocshArgBuf *args) {
	BeckCreateDriver(args[0].sval, args[1].ival, args[2].sval, args[3].sval);
}

static void BeckPortRegister(void) {
	iocshRegister(&BeckCreateDriverDef, BeckCreateDriverCallFunc);
}
extern "C" {
	epicsExportRegistrar(BeckPortRegister);
}
