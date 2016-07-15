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

#define KL2541_N_REG 64
#define MAX_TRY 2


static std::vector<BeckPortDriver *> _drivers;


int getBeckMaxAddr(const char *name) {
	for(std::vector<BeckPortDriver *>::iterator i=_drivers.begin(); i != _drivers.end(); i++) {
		if(strcmp((*i)->portName, name) == 0) {
			return (*i)->maxAddr;
		}
	}
	printf("getBeckMaxAddr -- Error: Cannot find the underlying port!\n");
	return -1;
}


BeckPortDriver::BeckPortDriver(const char *portName, int nAxis, const char *inModbusPort, const char *outModbusPort)
	: asynPortDriver( portName, nAxis, KL2541_N_REG+6, asynInt32Mask, asynInt32Mask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
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
	}
}


asynStatus BeckPortDriver::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
	printf("readInt32 with reason %d\n", pasynUser->reason);
	epicsInt32 axis;
	getAddress(pasynUser, &axis);

	if (pasynUser->reason <= KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		return readReg(pasynUser->reason-roffset, axis, value);
	}

	if (pasynUser->reason == statusByteIndx_){
		readProc(statusByte_[axis], 1, value);

	} else if (pasynUser->reason == dataInIndx_){
		readProc(dataIn_[axis], 1, value);

	} else if (pasynUser->reason == statusWordIndx_){
		readProc(statusWord_[axis], 1, value);

	} else if (pasynUser->reason == controlByteIndx_){
		readProc(controlByte_[axis], 0, value);

	} else if (pasynUser->reason == dataOutIndx_){
		readProc(dataOut_[axis], 0, value);

	} else if (pasynUser->reason == controlWordIndx_){
		readProc(controlWord_[axis], 0, value);

	} else {
		printf("Error: Unknown reason!\n");
		return asynError;
	}

	return asynSuccess;

}

asynStatus BeckPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	printf("writeInt32\n");
	epicsInt32 axis;
	getAddress(pasynUser, &axis);

	if (pasynUser->reason <= KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		return writeReg(pasynUser->reason-roffset, axis, value);
	}

	if (pasynUser->reason == statusByteIndx_){
		writeProc(statusByte_[axis], value);

	} else if (pasynUser->reason == dataInIndx_){
		writeProc(dataIn_[axis], value);

	} else if (pasynUser->reason == statusWordIndx_){
		writeProc(statusWord_[axis], value);

	} else if (pasynUser->reason == controlByteIndx_){
		writeProc(controlByte_[axis], value);

	} else if (pasynUser->reason == dataOutIndx_){
		writeProc(dataOut_[axis], value);

	} else if (pasynUser->reason == controlWordIndx_){
		writeProc(controlWord_[axis], value);

	} else {
		printf("Error: Unknown reason!\n");
		return asynError;
	}
	return asynSuccess;
}

asynStatus BeckPortDriver::writeReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 value) {
	printf("writeReg %d of axis %d with value %d\n", regN, axis, value);
	epicsInt32 readbackValue, k=0;
	epicsInt32 writeCmd = regN + 0xC0;
	do {
		writeProc(controlByte_[axis], 0x80);
		writeProc(dataOut_[axis], value);
		writeProc(controlByte_[axis], writeCmd);
		readReg(regN, axis, &readbackValue);
	} while ( (readbackValue != value) && (k++ <= MAX_TRY) );

	if ( readbackValue == value ) {
		return asynSuccess;
	}
	else {
		printf("Cannot correctly write %d in register %d of axis n: %d\n", value, regN, axis);
		return asynError;
	}
}

asynStatus BeckPortDriver::readReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 *value){
	printf("readReg %d of axis %d\n", regN, axis);
	epicsInt32 readCmd = regN + 0x80;
	epicsInt32 readBackValue, k=0;
	do {
		writeProc(controlByte_[axis], readCmd);
		readProc(controlByte_[axis], 0, &readBackValue);
	} while ( (readCmd != readBackValue) && (k++ <= MAX_TRY));

	if ( readCmd == readBackValue ) {
		return readProc(dataIn_[axis], 1, value);
	} else {
		printf("Cannot correctly read register %d of axis n: %d\n", regN, axis);
		return asynError;
	}
}

asynStatus BeckPortDriver::writeProc(asynInt32Client *modbusReg, epicsInt32 value) {
	printf("writeProc with value %d\n", value);
	return modbusReg->write(value);
}

asynStatus BeckPortDriver::readProc(asynInt32Client *modbusReg, bool in, epicsInt32 *value) {
	printf("readProc... ");
	if (in) {
		triggerReadIn_.write(0);
		newDataIn_.readWait(value);
	}
	printf("done!\n");
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
