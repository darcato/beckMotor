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

#define KL2541_N_REG 64
#define SB 0
#define DI 1
#define SW 2
#define CB 0
#define DO 1
#define CW 2


int getBeckMaxAddr(const char *portName);

class BeckPortDriver : public asynPortDriver {
protected:
	size_t nAxis_;
	size_t nRegs_;

	int statusByteReas_;
	int dataInReas_;
	int statusWordReas_;
	int controlByteReas_;
	int dataOutReas_;
	int controlWordReas_;
	int memoryInReas_;
	int memoryOutReas_;

	int roffset;

	std::vector<asynInt32Client *> statusByte_;
	std::vector<asynInt32Client *> dataIn_;
	std::vector<asynInt32Client *> statusWord_;
	std::vector<asynInt32Client *> controlByte_;
	std::vector<asynInt32Client *> dataOut_;
	std::vector<asynInt32Client *> controlWord_;

	asynInt32ArrayClient *inRegs_;
	asynInt32ArrayClient *outRegs_;

	std::vector< std::array<epicsInt32, 3> > cache_;

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
};

