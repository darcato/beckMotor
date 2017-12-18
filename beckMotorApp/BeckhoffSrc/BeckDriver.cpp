/**
 *  BeckPortDriver.cpp
 *  An epics port to access beckhoff process data registers and internal ones
 *  directly. Implementing asynInt32 and asynUInt32 interfaces.
 *
 *  Created on: July 14, 2016
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

#include "BeckDriver.h"
#include <epicsExport.h>
#include <iocsh.h>
#include <string.h>
#include <dbAccess.h>
#include <epicsThread.h>
#include <chrono>

//#include <unistd.h> //for usleep

#define KL2541_N_REG 64
#define CB 0
#define DO 1
#define CW 2

//a vector to store a reference to each driver created
static std::vector<BeckPortDriver *> _drivers;

//function to retrieve the number of axis given a controller name
int getBeckMaxAddr(const char *portName) {
	for(std::vector<BeckPortDriver *>::iterator i=_drivers.begin(); i != _drivers.end(); i++) {
		if(strcmp((*i)->portName, portName) == 0) {
			epicsStdoutPrintf("getBeckMaxAddr -- OK: found: %d controllers!\n",(*i)->maxAddr);
			return (*i)->maxAddr;
		}
	}
	epicsStdoutPrintf("getBeckMaxAddr -- Error: Cannot find the underlying port!\n");
	return -1;
}

//A driver to facilitate communication with beckhoff module
//enables r/w indifferently in internal or modbus registers, hiding the difference
//the addressing is done via the reason: SB, DI, SW, CB, DO, CW for the modbus regs and R00->R63 for the internal ones
//implements asynInt32 and asynUInt32Digital
BeckPortDriver::BeckPortDriver(const char *portName, int startAddr, int nAxis, const char *inModbusPort, const char *outModbusPort)
	: asynPortDriver( portName, nAxis, KL2541_N_REG+8, asynInt32Mask | asynUInt32DigitalMask | asynInt32ArrayMask | asynDrvUserMask, asynInt32Mask | asynUInt32DigitalMask | asynInt32ArrayMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0)//,
	//  triggerReadIn_(inModbusPort, 0, "MODBUS_READ"),
	//  newDataIn_(inModbusPort, 0, "MODBUS_DATA")
{
	nAxis_ = nAxis;     //how many KL2541 are stacked together
	nRegs_ = 3*nAxis_;  //each KL2541 has 3 registers in input, and 3 in output

	//create a parameter (representing the reason) for each internal register
	char name[10];
	int tmp;
	createParam("R00", asynParamInt32, &roffset);
	epicsStdoutPrintf("Creating %d parameters with reason offset of %d\n", KL2541_N_REG, roffset);
	for (int i=1; i < KL2541_N_REG; i++){
		sprintf(name, "R%02i", i);
		createParam(name, asynParamInt32, &tmp);
	}

	//create a parameter for the modbus registers -- ORDER IS IMPORTANT  --if you add one, change KL2541_N_REG+8 with updated num above
	createParam("SB", asynParamInt32, &statusByteReas_);
	createParam("DI", asynParamInt32, &dataInReas_);
	createParam("SW", asynParamInt32, &statusWordReas_);
	createParam("CB", asynParamInt32, &controlByteReas_);
	createParam("DO", asynParamInt32, &dataOutReas_);
	createParam("CW", asynParamInt32, &controlWordReas_);
	createParam("MI", asynParamInt32, &memoryInReas_);
	createParam("MO", asynParamInt32, &memoryOutReas_);

	//for each port, connect a client to read or write all the memory
	inRegs_ = new asynInt32ArrayClient(inModbusPort, startAddr, "MODBUS_DATA");
	outRegs_ = new asynInt32ArrayClient(outModbusPort, startAddr+2048, "MODBUS_DATA");

	//for each axis connect to the modbus registers
	for(int i=0; i<nAxis; i++){
		epicsUInt16 inAddr = startAddr + i*3;
		epicsUInt16 outAddr = startAddr + 2048 + i*3;   //output registers are shifted by 0x800 = 2048
		statusByte_.push_back(new asynInt32Client(inModbusPort, inAddr+0, "MODBUS_DATA"));
		dataIn_.push_back(new asynInt32Client(inModbusPort, inAddr+1, "MODBUS_DATA"));
		statusWord_.push_back(new asynInt32Client(inModbusPort, inAddr+2, "MODBUS_DATA"));
		controlByte_.push_back(new asynInt32Client(outModbusPort, outAddr+0, "MODBUS_DATA"));
		dataOut_.push_back(new asynInt32Client(outModbusPort, outAddr+1, "MODBUS_DATA"));
		controlWord_.push_back(new asynInt32Client(outModbusPort, outAddr+2, "MODBUS_DATA"));

		std::array<epicsInt32, 3> axisOutRegs;
		cache_.push_back(axisOutRegs);
	}

	//get initial values of output values to sync cache at the beginning
	//then their values will be updated only on writings and a cache will be kept
	size_t allnIn;  //how many actually read
	//size_t nAllRegs = 3*nAxis;  //There are 3 output registers for each axis (see KL2541 docs) - nAxis_ is the num of axis
	//epicsInt32 allRegs[nRegs_][3] = {0};  //an array to receive the reading of all the output registers
	asynStatus status = outRegs_->read(cache_.data()->data(), nRegs_, &allnIn);  //will read all the initial values (not updated with changes from this program)
	epicsStdoutPrintf("read\n");
	if (status != asynSuccess or allnIn!=nRegs_) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "ERROR: Cannot initialize registers\n");
	}

}

//readInt32 implementation
asynStatus BeckPortDriver::readInt32(asynUser *pasynUser, epicsInt32 *value) {
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "readInt32 with user %p and reason %d\n", pasynUser, pasynUser->reason);
	epicsInt32 axis;
	getAddress(pasynUser, &axis);

	//if an internal register
	if (pasynUser->reason < KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Register %d\n", pasynUser->reason-roffset);
		return readReg(pasynUser->reason-roffset, axis, value);

	//else check which modbus register
	} else if (pasynUser->reason == statusByteReas_){
		//asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Status Byte - 0x%x\n", *value);
		return statusByte_[axis]->read(value);

	} else if (pasynUser->reason == dataInReas_){
		//asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read DataIn - 0x%x\n", *value);
		return dataIn_[axis]->read(value);

	} else if (pasynUser->reason == statusWordReas_){
		//asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Status Word - 0x%x\n", *value);
		return statusWord_[axis]->read(value);

	//for the output modbus register I return a saved value because they can be read only one time
	//this means this driver must be the only one accessing those registers
	} else if (pasynUser->reason == controlByteReas_){
		*value = cache_[axis][CB];
		return asynSuccess;

	} else if (pasynUser->reason == dataOutReas_){
		*value = cache_[axis][DO];
		return asynSuccess;

	} else if (pasynUser->reason == controlWordReas_){
		*value = cache_[axis][CW];
		return asynSuccess;

	//cannot recognize reason
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error: Wrong reason!\n");
		return asynError;
	}
}

//writeInt32 implementation
asynStatus BeckPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	epicsInt32 axis;
	getAddress(pasynUser, &axis);
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "writeInt32 with value 0x%04x %i\n", value, axis);

	//if an internal register
	if (pasynUser->reason <= KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Write Register %d - 0x%x\n", pasynUser->reason-roffset, value);
		return writeReg(pasynUser->reason-roffset, axis, value);

	//if a modbus register
	} else if (pasynUser->reason == controlByteReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Write Control Byte - 0x%x\n", value);
		cache_[axis][CB] = value;
		return controlByte_[axis]->write(value);

	} else if (pasynUser->reason == dataOutReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Write DataOut - 0x%x\n", value);
		cache_[axis][DO] = value;
		return dataOut_[axis]->write(value);

	} else if (pasynUser->reason == controlWordReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Write Control Word - 0x%x\n", value);
		cache_[axis][CW] = value;
		return controlWord_[axis]->write(value);

	//cannot write to input registers, sorry.
	} else {
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Error: Wrong reason!\n");
		return asynError;
	}
}

//readUInt32Digital implementation
asynStatus BeckPortDriver::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask) {
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "readUInt32Digital with user %p, reason %d and mask 0x%04x\n", pasynUser, pasynUser->reason, mask);

	epicsInt32 rawValue;
	asynStatus status;

	status = readInt32(pasynUser, &rawValue);
	if (status!=asynSuccess) {
		return status;
	}

	//simply apply a mask over the asynInt32 reading
	*value = rawValue & mask;
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read 0x%04x with mask 0x%04x becomes 0x%04x\n", rawValue, mask, *value);
	return asynSuccess;
}

//writeUInt32Digital implementation
asynStatus BeckPortDriver::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask) {
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "writeUInt32Digital with user %p, reason %d, value %d, and mask 0x%04x\n", pasynUser, pasynUser->reason, value, mask);

	epicsInt32 rawValue;
	asynStatus status;

	status = readInt32(pasynUser, &rawValue);
	if (status!=asynSuccess) {
		return status;
	}

	/* Set bits that are set in the value and set in the mask */   // <-- smart guys
	rawValue |=  (value & mask);
	/* Clear bits that are clear in the value and set in the mask */
	rawValue  &= (value | ~mask);

	return writeInt32(pasynUser, rawValue);
}

asynStatus BeckPortDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn) {
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "readInt32Array with user %p and reason %d\n", pasynUser, pasynUser->reason);
	epicsInt32 axisFrom;
	getAddress(pasynUser, &axisFrom);
	asynStatus status;

	printf("BeckDriver - read Int32Array\n");

	//if an internal register
	if (pasynUser->reason < KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Registers %d\n", pasynUser->reason-roffset);
		return readRegArray(pasynUser->reason-roffset, axisFrom, nElements, value, nIn);

	//modbus input registers
	//read all of them (efficient) and return only the interesting ones
	} else if (pasynUser->reason == statusByteReas_ or pasynUser->reason == dataInReas_ or pasynUser->reason == statusWordReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Input Modbus Register\n");
		size_t allnIn;  //how many actually read
		epicsInt32 allRegs[nRegs_];  //an array to receive the reading of all the input registers
		size_t whichReg = pasynUser->reason - statusByteReas_;  //0=statusByte, 1=DataIn, 2=statusWord
		status = inRegs_->read(allRegs, nRegs_, &allnIn);
		if (status != asynSuccess) {
			return status;
		}
		*nIn=0; //will accumulate here the num of elements in value
		for (size_t i=axisFrom+whichReg; *nIn<nElements && i<allnIn; i+=3) {
			value[(*nIn)++]=allRegs[i];
		}
		return asynSuccess;

	//for the output modbus register I return a saved value because they can be read only one time
	//this means this driver must be the only one accessing those registers
	} else if (pasynUser->reason == controlByteReas_ or pasynUser->reason == dataOutReas_ or pasynUser->reason == controlWordReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Output Modbus Register\n");
		*nIn=0; //will accumulate here the num of elements in value
		for (size_t axis=axisFrom; axis-axisFrom<nElements && axis<nAxis_; axis++) {
			value[(*nIn)++] = cache_[axis][pasynUser->reason-controlByteReas_];
		}
		return asynSuccess;

	//reding the whole memory in input
	} else if (pasynUser->reason == memoryInReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Memory Input\n");
		size_t allnIn;  //how many actually read
		epicsInt32 allRegs[nRegs_];  //an array to receive the reading of all the input registers
		status = inRegs_->read(allRegs, nRegs_, &allnIn);
		if (status != asynSuccess) {
			return status;
		}
		*nIn=0; //will accumulate here the num of elements in value
		for (size_t i=3*axisFrom; *nIn<nElements && i<allnIn; i++) {
			value[(*nIn)++]=allRegs[i];
		}
		return asynSuccess;

	//reding the whole memory in output (cache)
	} else if (pasynUser->reason == memoryOutReas_){
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Read Memory Output (cache)\n");
		for ((*nIn)=0; *nIn<nElements && axisFrom+*nIn/3<nAxis_ && *nIn<nRegs_; (*nIn)++) {
			value[*nIn] = cache_[axisFrom+(*nIn)/3][(*nIn)%3];  //return cache
		}
		return asynSuccess;

	//cannot recognize reason
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error: Wrong reason!\n");
		return asynError;
	}
}
asynStatus BeckPortDriver::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements) {
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "writeInt32Array with user %p and reason %d\n", pasynUser, pasynUser->reason);
	epicsInt32 axisFrom;
	getAddress(pasynUser, &axisFrom);

	printf("BeckDriver - write Int32Array\n");

	//if an internal register
	if (pasynUser->reason <= KL2541_N_REG+roffset && pasynUser->reason>=roffset) {
		return writeRegArray(pasynUser->reason-roffset, axisFrom, nElements, value);

	//if a modbus register
	} else if (pasynUser->reason == controlByteReas_ or pasynUser->reason == dataOutReas_ or pasynUser->reason == controlWordReas_){
		for (size_t axis=axisFrom; axis<axisFrom+nElements && axis<nAxis_; axis++) {
			for (int reg=0; reg<3; reg++){
				if (reg==pasynUser->reason-controlByteReas_) { //replace the old values with new ones
					cache_[axis][reg] = value[axis-axisFrom];  //only for interested axis and only on selected reg
				}
			}
		}
		return outRegs_->write(cache_.data()->data(), nRegs_);

	//writing the whole memory in output
	} else if (pasynUser->reason == memoryOutReas_){
		for (size_t i=0; i<nElements && axisFrom+i/3<nAxis_ && i<nRegs_; i++) {
			cache_[axisFrom+i/3][i%3] = value[i];  //update cache
		}
		return outRegs_->write(cache_.data()->data(), nRegs_);

	//cannot write to input registers, sorry.
	} else {
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "Error: Wrong reason!\n");
		return asynError;
	}
}

/**
 * PRIVATE
 */

//WRITE a beckhoff internal register
//regN = the internal register to write
//axisFrom -> axisAmount = The range of axis where to write the register
//value = the value to be written
asynStatus BeckPortDriver::writeReg(size_t regN, size_t axis, epicsInt32 value) {
	//epicsStdoutPrintf("writeReg %d of axis %d with value 0x%x\n", regN, axis, value);
	epicsInt32 writeCmd = regN + 0xC0;
	asynStatus status = asynSuccess;

	status = controlByte_[axis]->write(writeCmd);  //tell controllor I want to write regN register
	if (status!=asynSuccess) {
		return status;
	}
	status = dataOut_[axis]->write(value);	      //set the value of the register
	if (status!=asynSuccess) {
		return status;
	}
	status = controlByte_[axis]->write(0x80);      //stop writing mode (I cannot directly reset dataOut, or it will reset the regN value,
	if (status!=asynSuccess) {                     //I cannot directly reset controlByte, or it will start moving with velocity dataOut)
		return status;
	}
	status = dataOut_[axis]->write(cache_[axis][DO]);  //reset to previous dataOut
	if (status!=asynSuccess) {
		return status;
	}
	status = controlByte_[axis]->write(cache_[axis][CB]);  //reset to previous controlByte
	if (status!=asynSuccess) {
		return status;
	}

	//NOTE: this function must leave the controller in process data mode
	return asynSuccess;
}

//READ a beckhoff internal register
//regN = the internal register to read
//axisFrom -> axisAmount = The range of axis where to write the register
//value = place where to put the return value
asynStatus BeckPortDriver::readReg(size_t regN, size_t axis, epicsInt32 *value ){
	//epicsStdoutPrintf("readReg %d of axis %d\n", regN, axis);
	asynStatus status = asynSuccess;
	epicsInt32 readCmd = regN + 0x80;
	status = controlByte_[axis]->write(readCmd);
	if (status!=asynSuccess) {
		return status;
	}
	status = dataIn_[axis]->read(value);
	if (status!=asynSuccess) {
		return status;
	}
	status = controlByte_[axis]->write(cache_[axis][CB]);
	if (status!=asynSuccess) {
		return status;
	}
	//NOTE: this function must leave the controller in process data mode
	return asynSuccess;
}

asynStatus BeckPortDriver::writeRegArray(size_t regN, size_t axisFrom, size_t axisAmount, epicsInt32 *value) {
	epicsInt32 writeCmd = regN + 0xC0;
	asynStatus status = asynSuccess;

	epicsInt32 writingVals[nAxis_][3];  //output values used to write the desired inner registers
	for (size_t axis=0; axis<nAxis_; axis++) {
		if (axis>=axisFrom && axis-axisFrom<axisAmount){  //if interested axes
			writingVals[axis][CB] = writeCmd;  //controlByte
			writingVals[axis][DO] = value[axis-axisFrom];  //dataOut
		} else {  //leave them as cached
			writingVals[axis][CB] = cache_[axis][CB];  //controlByte
			writingVals[axis][DO] = cache_[axis][DO];  //dataOut
		}
		writingVals[axis][CW] = cache_[axis][CW];  //controlWord
	}
	status = outRegs_->write(writingVals[0], nRegs_);
	if (status!=asynSuccess) {
		return status;
	}
	status = outRegs_->write(cache_.data()->data(), nRegs_); //restore process data mode
	if (status!=asynSuccess) {
		return status;
	}
	//NOTE: this function must leave the controller in process data mode
	return asynSuccess;
}

asynStatus BeckPortDriver::readRegArray(size_t regN, size_t axisFrom, size_t axisAmount, epicsInt32 *value, size_t *nIn) {
	asynStatus status = asynSuccess;
	epicsInt32 readCmd = regN + 0x80;
	size_t readnIn;

	epicsInt32 writingVals[nAxis_][3];  //output values used to write the desired inner registers
	epicsInt32 readValues[nAxis_][3];
	for (size_t axis=0; axis<nAxis_; axis++) {
		if (axis>=axisFrom && axis-axisFrom<axisAmount){  //if interested axes
			writingVals[axis][CB] = readCmd;  //controlByte
		} else {
			writingVals[axis][CB] = cache_[axis][CB];  //controlByte
		}
		writingVals[axis][DO] = cache_[axis][DO];  //dataOut
		writingVals[axis][CW] = cache_[axis][CW];  //controlWord
	}

	status = outRegs_->write(writingVals[0], nRegs_);
	if (status!=asynSuccess) {
		return status;
	}
	status = inRegs_->read(readValues[0], nRegs_, &readnIn);
	if (status!=asynSuccess) {
		return status;
	}
	status = outRegs_->write(cache_.data()->data(), nRegs_); //restore process data mode
	if (status!=asynSuccess) {
		return status;
	}

	//return the values read from dataIn
	for ((*nIn)=0; axisFrom+(*nIn)<nAxis_ && (*nIn)<axisAmount && 3*axisFrom+3*(*nIn)+1<readnIn; (*nIn)++) {  //nIn := axis
		value[*nIn] = readValues[3*axisFrom+3*(*nIn)][1];  //regN values are found in read dataIn (offset 1)
	}
	//NOTE: this function must leave the controller in process data mode
	return asynSuccess;
}


extern "C" int BeckCreateDriver(const char *portName, const int startAddr, const int numAxis, const char *inModbusPName, const char *outModbusPName)
{
	if (startAddr<0 || startAddr+3*numAxis>255) return asynError;  //beckhoff memory limits
	if (numAxis>41) return asynError; //modbus limits on a single operation

	BeckPortDriver *drv = new BeckPortDriver(portName, startAddr, numAxis, inModbusPName, outModbusPName);
	epicsStdoutPrintf("Driver %p created!\n", drv);
	_drivers.push_back(drv);
    return asynSuccess;
}


/**
 * Code for iocsh registration
 */
static const iocshArg BeckCreateDriverArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateDriverArg1 = {"Start Address", iocshArgInt};
static const iocshArg BeckCreateDriverArg2 = {"Number of axes", iocshArgInt};
static const iocshArg BeckCreateDriverArg3 = {"Modbus INPUT port name", iocshArgString};
static const iocshArg BeckCreateDriverArg4 = {"Modbus OUTPUT port name", iocshArgString};

static const iocshArg * const BeckCreateDriverArgs[] = {&BeckCreateDriverArg0,
                                                        &BeckCreateDriverArg1,
                                                        &BeckCreateDriverArg2,
														&BeckCreateDriverArg3,
                                                        &BeckCreateDriverArg4};

static const iocshFuncDef BeckCreateDriverDef = {"BeckCreateDriver", 5, BeckCreateDriverArgs};

static void BeckCreateDriverCallFunc(const iocshArgBuf *args) {
	BeckCreateDriver(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].sval);
}

static void BeckDriverRegister(void) {
	iocshRegister(&BeckCreateDriverDef, BeckCreateDriverCallFunc);
}
extern "C" {
	epicsExportRegistrar(BeckDriverRegister);
}
