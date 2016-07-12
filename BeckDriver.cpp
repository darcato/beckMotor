/*
FILENAME... BeckDriver.cpp
USAGE...    Motor driver support for the Beckhoff KL2541 controller.

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

BeckController::BeckController(const char *portName, const int numAxis, const char *inModbusPName, const char *outModbusPName, double movingPollPeriod, double idlePollPeriod )
  :  asynMotorController(portName, numAxis, 0,
                         asynUInt32DigitalMask, // Add asynUInt32Digital interface to read single bits
                         asynUInt32DigitalMask, // Add asynUInt32Digital callbacks to read single bits
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  BeckAxis *pAxis; //set but not used not to be eliminated by compiler

  inModbusPName_ = (char *) mallocMustSucceed(strlen(inModbusPName)+1, "Malloc failed\n");
  outModbusPName_ = (char *) mallocMustSucceed(strlen(outModbusPName)+1, "Malloc failed\n");

  strcpy(inModbusPName_, inModbusPName);
  strcpy(outModbusPName_, outModbusPName);

  printf("Now create axis\n");
  int i = 0;
  for (i=0; i<numAxis; i++){
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
  fprintf(fp, "Beckoff motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, 1, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

extern "C" int BeckCreateController(const char *portName, const int numAxis, const char *inModbusPName, const char *outModbusPName, int movingPollPeriod, int idlePollPeriod )
{
	BeckController *ctrl = new BeckController(portName, numAxis, inModbusPName, outModbusPName, movingPollPeriod/1000., idlePollPeriod/1000.);
    printf("Controller %p\n", ctrl);
	_controllers.push_back(ctrl);
    return(asynSuccess);
}



/************************************************/

BeckAxis::BeckAxis(BeckController *pC, int axis) :
		asynMotorAxis(pC, axis),
		pC_(pC),
		statusByte_(pC_->inModbusPName_, 3*axis+0, "MODBUS_DATA"),
		dataIn_(pC_->inModbusPName_, 3*axis+1, "MODBUS_DATA"),
		statusWord_(pC_->inModbusPName_, 3*axis+2, "MODBUS_DATA"),
		controlByte_(pC_->outModbusPName_, 3*axis+0, "MODBUS_DATA"),
		dataOut_(pC_->outModbusPName_, 3*axis+1, "MODBUS_DATA"),
		controlWord_(pC_->outModbusPName_, 3*axis+2, "MODBUS_DATA"),
		triggerRead_(pC_->inModbusPName_, 0, "MODBUS_READ")
{
  asynStatus status;
  static const char *functionName = "BeckAxis::BeckAxis";

  /* Connect to bits */
  status = pasynUInt32DigitalSyncIO->connect(pC_->inModbusPName_, 3*axis+0, &statusByteBits_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }

  status = pasynUInt32DigitalSyncIO->connect(pC_->inModbusPName_, 3*axis+2, &statusWordBits_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }

  status = pasynUInt32DigitalSyncIO->connect(pC_->outModbusPName_, 3*axis+0, &controlByteBits_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }

  status = pasynUInt32DigitalSyncIO->connect(pC_->outModbusPName_, 3*axis+2, &controlWordBits_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
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

  controlByte_.write(0x21);

}

void BeckAxis::report(FILE *fp, int level)
{
    fprintf(fp, "Axis %d status: %s\n", axisNo_, movePend ? "Moving" : "Idle");
    asynMotorAxis::report(fp, level);
}

asynStatus BeckAxis::setAcclVelo(double min_velocity, double max_velocity, double acceleration) {
	modbusMutex.lock();
	controlByte_.write(0x80); //prevent undesired writings
	usleep(1);

	dataOut_.write((int) min_velocity);
	controlByte_.write(0xE6);
	controlByte_.write(0xA6);
	usleep(1);

	dataOut_.write((int) max_velocity);
	controlByte_.write(0xE7);
	controlByte_.write(0xA7);
	usleep(1);

	dataOut_.write((int) acceleration);
	controlByte_.write(0xE8);
	controlByte_.write(0xA8);
	usleep(1);

	modbusMutex.unlock();
	return asynSuccess;
}

asynStatus BeckAxis::setCoilCurrent(double maxCurr, double autoHoldinCurr, double manHoldingCurr, double highAccCurr, double lowAccCurr) {
	int termType = 0;
	double fullScaleCurr;
	int setMaxCurrent;
	int percent = -1;

	//read controller code and convert ampere to %
	modbusMutex.lock();
	controlByte_.write(0x88);
	triggerRead_.write(0);
	statusByte_.readWait(&termType);
	statusWord_.readWait(&termType);
	dataIn_.readWait(&termType);

	switch (termType) {
		case 2531: fullScaleCurr = 1.5; break;
		case 2541: fullScaleCurr = 5.0;	break;
		default: {
			printf("Error: Cannot recognize controller type %d", termType);
			return asynError;
		}
	}

	if (maxCurr>=0) {
		percent = round( maxCurr / fullScaleCurr *100 );
		dataOut_.write(percent);
		controlByte_.write(0xE3);
	}
	controlByte_.write(0xA3);
	triggerRead_.write(0);
	statusByte_.readWait(&setMaxCurrent);
	statusWord_.readWait(&setMaxCurrent);
	dataIn_.readWait(&setMaxCurrent);
	if (maxCurr>=0 && setMaxCurrent!=percent){
		printf("Error: Failed to write maxCurrent\n");
		return asynError;
	}
	if (autoHoldinCurr>=0) {
		if (autoHoldinCurr>setMaxCurrent) {
			printf("Error: Cannot set holding current higher than maximum coil current!\n");
		}
		percent = round( autoHoldinCurr / setMaxCurrent *100 );
		dataOut_.write(percent);
		controlByte_.write(0xE3);
		controlByte_.write(0xA3);
		usleep(PLC_LOOP_US);
	}

	printf("Setto corrente in axis %d!\n", axisNo_);
	return asynSuccess;
}

asynStatus BeckAxis::resetController() {
	printf("Resetto controller axis %d!\n", axisNo_);
	return asynSuccess;
}


asynStatus BeckAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
    //TODO set velocity & acceleration //DONE
    //TODO check for overflow in relative mode
    //TODO check first execution //DONE
    //TODO understand the useless write of 0x82 to controlByte //DONE
    //TODO verify stall or unable to end movement

    //TODO may be useless to read position from HW  //DONE
    //TODO understand what to do if relative or not  //DONE
    //TODO understand if I need to check for end of movement  //DONE

    //TODO register callback to wait end of movement  //DONE

	//This to prevent to put movePend to 1 if cannot move
	if (movePend) return asynSuccess;

	setAcclVelo(min_velocity, max_velocity, acceleration);

	modbusMutex.lock();

    printf("-%s(%.2f, %i, %.2f, %.2f, %.2f)\n", __FUNCTION__, position, relative, min_velocity, max_velocity, acceleration);

    //stop motor
    controlByte_.write(0x21);

    //set new position
    int newPos =  (relative ? currPos : 0 ) + position;

    if (!movePend)
    	printf("NewPos is %d\n", newPos);

    dataOut_.write(newPos & 0xFFFF);
    controlByte_.write(0xC2);

    //As long as 0xC2 is written in control byte, everything you write in dataOut is written to R7
    //so we stop this behaviour by writing the reading command 0x87 in controlbyte
    controlByte_.write(0x82);
    usleep(1);

    dataOut_.write((newPos>>16) & 0xFFFF);
    controlByte_.write(0xC3);
    controlByte_.write(0x83);
    usleep(1);

    //put zeroes in data as required
    dataOut_.write(0x0);

    movePend=true;
    controlByte_.write(0x25);

    //printf("Movement complete: new position is 0x%x\n\n", newPos);
    modbusMutex.unlock();
    return asynSuccess;
}

asynStatus BeckAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
   /* int outVal = 0x520;
    pasynInt32SyncIO->write(dataOut_, outVal, 500);
    outVal = 0xC7;
    pasynInt32SyncIO->write(controlByte_, outVal, 500);
    outVal = 0;
    pasynInt32SyncIO->write(dataOut_, outVal, 500);
    outVal = 0x5;
    pasynInt32SyncIO->write(controlByte_, outVal, 500);

    endMove();*/

    return asynSuccess;
}
asynStatus BeckAxis::poll(bool *moving)
{
	if (interruptAccept){
	    epicsInt32 statusByte=0;
	    epicsInt32 dataIn=0;
	    epicsInt32 statusWord=0;
	    bool lhigh, llow;
	    bool regAccess, error, warning, idle, ready;
	    epicsInt32 loadAngle;

	    modbusMutex.lock();
	    controlByte_.write(0x80);
	    //usleep(PLC_LOOP_US);
	    triggerRead_.write(0);
	    statusByte_.readWait(&statusByte); //to be overwritten
	    statusWord_.readWait(&statusWord); //to be overwritten
	    dataIn_.readWait(&dataIn);
	    currPos = dataIn;

	    controlByte_.write(0x81);
	    //usleep(PLC_LOOP_US);
	    triggerRead_.write(0);
	    statusByte_.readWait(&statusByte); //to be overwritten
	    statusWord_.readWait(&statusWord);
	    dataIn_.readWait(&dataIn);
	    currPos += dataIn<<16;

	    setDoubleParam(pC_->motorPosition_, currPos);

	    lhigh = statusWord & 0x1;
	   	llow = statusWord & 0x2;
	   	setIntegerParam(pC_->motorStatusHighLimit_, lhigh);
	   	setIntegerParam(pC_->motorStatusLowLimit_, llow);

		moveDone = movePend ? ((statusWord & 0x8) || lhigh || llow) : true;
		if (moveDone) {
			controlByte_.write(0x21);
			movePend = false;
		}
		else{
			controlByte_.write(0x25);
		}
		usleep(1);
		triggerRead_.write(0);
	    statusByte_.readWait(&statusByte);
	    dataIn_.readWait(&dataIn);
	    statusWord_.readWait(&statusWord);
	    modbusMutex.unlock();

	    regAccess = statusByte & 0x80;
		error = statusByte & 0x40;
		setIntegerParam(pC_->motorStatusProblem_, error);
		warning = statusByte & 0x20;
		idle = statusByte & 0x10;
		//printf("statusByte 0x%x\t !idle is %d\n",statusByte,!idle);
		loadAngle = statusByte & 0xE;
		ready = statusByte & 0x1;
		setIntegerParam(pC_->motorStatusPowerOn_, ready);

		*moving = moveDone ? false : true;
		setIntegerParam(pC_->motorStatusDone_, moveDone);

		//printf("Moving: %d\t MoveDone: %d\t SwL: %d\t SwH: %d\n", movePend, moveDone, llow, lhigh);
		callParamCallbacks();
	}
	else {
		printf("Interruptions not yet allowed..\n");
	}

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
    printf("%s ( %s, %d, %s, %s)\n", "--BeckConfigController", ctrlName, axisRange, cmd, cmdArgs);
    BeckController *ctrl = findBeckControllerByName(ctrlName);
    if (ctrl == NULL) {
    	epicsStdoutPrintf("Cannot find controller %s!\n", ctrlName);
    	return 0;
    }
    BeckAxis *axis = ctrl->getAxis(axisRange);
    if (axis == NULL) {
    	epicsStdoutPrintf("Cannot find axis %d!\n", axisRange);
    }

    if (strcmp(cmd, "setCoilCurrents") == 0) {
    	char *maxCurrStr=0;
    	char *autoHoldinCurrStr=0;
    	char *manHoldingCurrStr=0;
    	char *highAccCurrStr=0;
		char *lowAccCurrStr=0;

    	int nPar = sscanf(cmdArgs, "%m[^,],%m[^,],%m[^,],%m[^,],%m[^,]", &maxCurrStr,
    			                                          &autoHoldinCurrStr,
		                                                  &manHoldingCurrStr,
		                                                  &highAccCurrStr,
		                                                  &lowAccCurrStr);
    	printf("setCoilCurrents %d -- %s -- %s -- %s -- %s -- %s\n",nPar,
    																maxCurrStr,
																	autoHoldinCurrStr,
																	manHoldingCurrStr,
																	highAccCurrStr,
																	lowAccCurrStr);

    	double maxCurr=-1;
    	double autoHoldinCurr=-1;
    	double manHoldingCurr=-1;
    	double highAccCurr=-1;
		double lowAccCurr=-1;

		switch (nPar) {
			case 5: epicsScanDouble(lowAccCurrStr, &lowAccCurr);
			case 4: epicsScanDouble(highAccCurrStr, &highAccCurr);
			case 3:	epicsScanDouble(manHoldingCurrStr, &manHoldingCurr);
			case 2: epicsScanDouble(autoHoldinCurrStr, &autoHoldinCurr);
			case 1: epicsScanDouble(maxCurrStr, &maxCurr);
			default: {
				printf ("Wrong number of parameters!\n");
				return 0;
			}

		}

    	axis->setCoilCurrent(maxCurr, autoHoldinCurr, manHoldingCurr, highAccCurr, lowAccCurr);
    }
    else if (strcmp(cmd, "resetController") ==0 ) {
    	axis->resetController();
    }
    else {
    	epicsStdoutPrintf("Command not found!\n");
    }
    //ADD: set/getMicrostep - Encoder related fields -
    return(asynSuccess);
}








/**
 * Code for iocsh registration
 */
static const iocshArg BeckCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateControllerArg1 = {"Number of consecutive controllers", iocshArgInt};
static const iocshArg BeckCreateControllerArg2 = {"Modbus INPUT port name", iocshArgString};
static const iocshArg BeckCreateControllerArg3 = {"Modbus OUTPUT port name", iocshArgString};
static const iocshArg BeckCreateControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg BeckCreateControllerArg5 = {"Idle poll period (ms)", iocshArgInt};

static const iocshArg BeckConfigControllerArg0 = {"Controller Name", iocshArgString};
static const iocshArg BeckConfigControllerArg1 = {"Axis Number", iocshArgInt};
static const iocshArg BeckConfigControllerArg2 = {"Command", iocshArgString};
static const iocshArg BeckConfigControllerArg3 = {"CommandArgs", iocshArgString};

static const iocshArg * const BeckCreateControllerArgs[] = {&BeckCreateControllerArg0,
                                                            &BeckCreateControllerArg1,
                                                            &BeckCreateControllerArg2,
                                                            &BeckCreateControllerArg3,
                                                            &BeckCreateControllerArg4,
                                                            &BeckCreateControllerArg5};
static const iocshArg * const BeckConfigControllerArgs[] = {&BeckConfigControllerArg0,
                                                            &BeckConfigControllerArg1,
                                                            &BeckConfigControllerArg2,
                                                            &BeckConfigControllerArg3};

static const iocshFuncDef BeckCreateControllerDef = {"BeckCreateController", 6, BeckCreateControllerArgs};
static const iocshFuncDef BeckConfigControllerDef = {"BeckConfigController", 4, BeckConfigControllerArgs};

static void BeckCreateControllerCallFunc(const iocshArgBuf *args) {
	BeckCreateController(args[0].sval, args[1].ival, args[2].sval, args[3].sval, args[4].ival, args[5].ival);
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
