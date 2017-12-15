/**
 *  BeckPortDriver.h
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

#include <asynPortDriver.h>
#include <asynPortClient.h>
#include <asynUInt32Digital.h>
#include <vector>
#include <array>
#include <epicsEvent.h>

int getBeckMaxAddr(const char *portName);

//a classs to trigger a modbus read and wait for new data before returning
//use write to trigger and readWait to read new data
class asynInt32ClientSyncIO : public asynInt32Client {
private:
	epicsEvent event_;
	epicsInt32 value_;
	asynStatus status_;
	static void newvaluecb(void *userPvt, asynUser *pasynUser, epicsInt32 data) {
		asynInt32ClientSyncIO *obj=static_cast<asynInt32ClientSyncIO *>(userPvt);
		obj->value_ = data;
		obj->status_ = asynSuccess;  //TODO: propagate status
		obj->event_.signal();
	}

public:
	asynInt32ClientSyncIO(const char *portName, int addr, const char *drvInfo, double timeout=DEFAULT_TIMEOUT)
	    : asynInt32Client(portName, addr, drvInfo, timeout),
		  event_(epicsEventEmpty),
		  value_(0),
		  status_(asynSuccess)
	{
		registerInterruptUser(asynInt32ClientSyncIO::newvaluecb);
	}
	asynStatus readWait(epicsInt32 *ret){
		event_.wait();
		*ret = value_;
		return status_;
	}


};


class BeckPortDriver : public asynPortDriver {
protected:
	int statusByteIndx_;
	int dataInIndx_;
	int statusWordIndx_;
	int controlByteIndx_;
	int dataOutIndx_;
	int controlWordIndx_;
	int memoryInIndx_;
	int memoryOutIndx_;

	int roffset;

	std::vector<asynInt32Client *> statusByte_;
	std::vector<asynInt32Client *> dataIn_;
	std::vector<asynInt32Client *> statusWord_;
	std::vector<asynInt32Client *> controlByte_;
	std::vector<asynInt32Client *> dataOut_;
	std::vector<asynInt32Client *> controlWord_;

	asynInt32ArrayClient *inRegs_;
	asynInt32ArrayClient *outRegs_;

	//std::vector< std::array<epicsUInt32, 3> > cache;
	std::vector<epicsUInt32> controlByteValue_;
	std::vector<epicsUInt32> dataOutValue_;
	std::vector<epicsUInt32> controlWordValue_;

public:
	BeckPortDriver(const char *portName, int startAddr, int nAxes, const char *inModbusPort, const char *outModbusPort);
	asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);
	asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
	asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn);
	asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);

private:
	asynStatus writeReg(size_t regN, size_t axis, epicsInt32 value);
	asynStatus readReg(size_t regN, size_t axis, epicsInt32 *value);
	asynStatus writeRegArray(size_t regN, size_t axisFrom, size_t axisAmount, epicsInt32 *value);
	asynStatus readRegArray(size_t regN, size_t axisFrom, size_t axisAmount, epicsInt32 *value, size_t *nIn);
	asynStatus initializeOutputValues();
};

