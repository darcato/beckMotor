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
//#include <math.h>

#include <iocsh.h>
//#include <epicsThread.h>

#include <asynInt32SyncIO.h>
#include <asynUInt32Digital.h>
#include <asynUInt32DigitalSyncIO.h>

//#include <dbAccessDefs.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include <cantProceed.h>
#include "BeckDriver.h"


#define PLC_LOOP_US 1
#define MODBUS_IO_US 1

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
    BeckController *pBeckController //set but not used not to be eliminated by compiler
    = new BeckController(portName, numAxis, inModbusPName, outModbusPName, movingPollPeriod/1000., idlePollPeriod/1000.);
    pBeckController = NULL;
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

/*
   Connect to trigger read, to be called to do modbusIO
  status = pasynInt32SyncIO->connect(pC_->inModbusPName_, 0, &triggerRead_, "MODBUS_READ");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }

   Connect to Input registers
  status = pasynInt32SyncIO->connect(pC_->inModbusPName_, 3*axis+0, &statusByte_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }
  status = pasynInt32SyncIO->connect(pC_->inModbusPName_, 3*axis+1, &dataIn_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }
  status = pasynInt32SyncIO->connect(pC_->inModbusPName_, 3*axis+2, &statusWord_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }

   Connect to Output registers
  status = pasynInt32SyncIO->connect(pC_->outModbusPName_, 3*axis+0, &controlByte_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }
  status = pasynInt32SyncIO->connect(pC_->outModbusPName_, 3*axis+1, &dataOut_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }
  status = pasynInt32SyncIO->connect(pC_->outModbusPName_, 3*axis+2, &controlWord_, "MODBUS_DATA");
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Beckhoff controller\n", functionName);
  }
*/

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

}

void BeckAxis::report(FILE *fp, int level)
{
    fprintf(fp, "Axis report\n");
    asynMotorAxis::report(fp, level);
}

//private method to read position from KL2541
asynStatus BeckAxis::readPosition(double *position) {
    epicsInt32 outVal = 0x80;

    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    epicsInt32 pLow = 0;

    controlByte_.write(outVal);
    //usleep(PLC_LOOP_US);
    //pasynInt32SyncIO->write(triggerRead_, outVal, 500);
    //dataIn_.resetEvent();
    triggerRead_.write(outVal);
    usleep(MODBUS_IO_US);
    //pasynInt32SyncIO->read(dataIn_, &pLow, 500);
    dataIn_.readWait(&pLow);


    outVal=0x81;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    controlByte_.write(outVal);
    //usleep(PLC_LOOP_US);
    //pasynInt32SyncIO->write(triggerRead_, outVal, 500);
    triggerRead_.write(outVal);
    usleep(MODBUS_IO_US);
    epicsInt32 pHigh = 0;
    //pasynInt32SyncIO->read(dataIn_, &pHigh, 500);
    dataIn_.read(&pHigh);

    *position = pLow+(pHigh<<16);
    //printf("Read Position is: 0x %04x %04x\n", pHigh, pLow);
    return asynSuccess;
}

asynStatus BeckAxis::endMove() {
    //wait for travel completion
    epicsInt32 end=0;
    asynStatus status;
    epicsUInt16 cmd = 0;
    int k=0;
    while(!end) {
        //status = pasynInt32SyncIO->write(triggerRead_, cmd, 500);
    	status = triggerRead_.write(cmd);
    	printf("asynStatus is: %d ", status);
        status = statusWord_.read(&end);
        printf(", %d\n", status);
        if (!end) {
            usleep(1e5);
        }
        k++;
    }

    printf("Movement completed after %d loops\n", k);

    //end movement
    cmd = 0x21; //TODO need to unset only bit 3
    //pasynInt32SyncIO->write(controlByte_, cmd, 500);
    controlByte_.write(cmd);

    return asynSuccess;
}

asynStatus BeckAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
    //TODO set velocity & acceleration
    //TODO check for overflow in relative mode
    //TODO check first execution //DONE
    //TODO understand the useless write of 0x82 to controlByte
    //TODO verify stall or unable to end movement

    //TODO may be useless to read position from HW
    //TODO understand what to do if relative or not
    //TODO understand if I need to check for end of movement

    //TODO register callback to wait end of movement

    printf("\nENTERING MOVE\n");
    //asynStatus status;

    //stop motor
    epicsUInt16 outVal=0x21;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    controlByte_.write(outVal);

    double currPos = 0;
    if (relative) {
        readPosition(&currPos);
    }

    //set new position
    int newPos = (long) currPos + (long) position;
    int newPosLow= newPos & 0xFFFF;
    //printf("newPosLow is 0x%x\n", newPosLow);
    //pasynInt32SyncIO->write(dataOut_, newPosLow, 500);
    dataOut_.write(newPosLow);
    outVal=0xC2;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    controlByte_.write(outVal);

    //inexplicably useful call
    //outVal=0x82;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    usleep(PLC_LOOP_US);

    int newPosHigh = (newPos>>16) & 0xFFFF;
    //printf("newPosHigh is 0x%x\n", newPosHigh);
    //pasynInt32SyncIO->write(dataOut_, newPosHigh, 500);
    dataOut_.write(newPosHigh);
    outVal=0xC3;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    controlByte_.write(outVal);

    //put zeroes in data out
    outVal=0x0;
    //pasynInt32SyncIO->write(dataOut_, outVal, 500);
    dataOut_.write(outVal);

    //start movement
    outVal=0x25;
    //pasynInt32SyncIO->write(controlByte_, outVal, 500);
    controlByte_.write(outVal);

    endMove();

    printf("Movement complete: new position is 0x%x\n\n", newPos);
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
    //int addr;
    //pasynManager->getAddr(pasynUser_, &addr);
    //printf("address %i\n", addr);

/*
    epicsInt32 sB;
    epicsInt32 dI;
    epicsInt32 sW;
    epicsInt32 cB;
    epicsInt32 dO;
    epicsInt32 cW;

    epicsInt32 mval=0;
*/
//    pasynInt32SyncIO->write(triggerRead_, mval, 500);
/*
    pasynInt32SyncIO->read(statusByte_, &sB, 500);
    pasynInt32SyncIO->read(dataIn_, &dI, 500);
    pasynInt32SyncIO->read(statusWord_, &sW, 500);
    pasynInt32SyncIO->read(controlByte_, &cB, 500);
    pasynInt32SyncIO->read(dataOut_, &dO, 500);
    pasynInt32SyncIO->read(controlByte_, &cW, 500);
*/

//    printf("StatusByte:  0x%x\tDataIn:  0x%x\tStatusWord:  0x%x\n", sB, dI, sW);
//    printf("ControlByte: 0x%x\tDataOut: 0x%x\tControlWord: 0x%x\n", cB, dO, cW);

//    move(0x100,1,0,0,0);

    int statusWord = 0;
    int statusByte = 0;
    int controlByte = 0;
    //pasynInt32SyncIO->write(triggerRead_, statusWord, 500);
    triggerRead_.write(statusWord);
    usleep(PLC_LOOP_US);
    //pasynInt32SyncIO->read(statusWord_, &statusWord, 500);
    statusWord_.read(&statusWord);
    statusByte_.read(&statusByte);

    //get the end of a movement
    bool endMov = statusWord & 0x8;
    setIntegerParam(pC_->motorStatusDone_, endMov);

    //get limit switch status
    bool lhigh = statusWord & 0x1;
    bool llow = statusWord & 0x2;
    setIntegerParam(pC_->motorStatusHighLimit_, lhigh);
    setIntegerParam(pC_->motorStatusLowLimit_, llow);

    //get moving flag (only in process data mode [SB7==0], else don't change)
    bool regAccess = statusByte & 0x80;
    bool error;
    bool warning;
    bool idle;
    int loadAngle;
    bool ready;
    if ( !regAccess ) {
        error = statusByte & 0x40;
        warning = statusByte & 0x20;
        idle = statusByte & 0x10;
        *moving = !idle;
        loadAngle = statusByte & 0xE;
        ready = statusByte & 0x1;
    }

    //get motor postion
    double position;
    //readPosition(&position);
    //setDoubleParam(pC_->motorPosition_, (int) position);

    printf("statusByte: 0x%x\tmov: %d\tend %d\t l+ %d\tl- %d\tpos: 0x%x\n", statusByte, *moving, endMov, lhigh, llow, (int) position);

    printf("--Going to write 0x21\n");
    controlByte = 0x21;
    epicsUInt32 rbit;
    //pasynInt32SyncIO->write(controlByte_, controlByte, 500);
    controlByte_.write(controlByte);
    sleep(2);
    //pasynInt32SyncIO->read(controlByte_, &controlByte, 500);
    controlByte_.read(&controlByte);
    usleep(1e3);
    printf("-ControlByte is 0x%x\n", controlByte);
    pasynUInt32DigitalSyncIO->read(controlByteBits_, &rbit, 0x20, 500);
    usleep(1e3);
    printf("-Bit 0x20 of controlByte is: %d\n", rbit);
    printf("-Now zero bit 0x20\n");
    rbit=0x0;
    pasynUInt32DigitalSyncIO->write(controlByteBits_, rbit, 0x20, 500);
    usleep(1e3);
    pasynUInt32DigitalSyncIO->read(controlByteBits_, &rbit, 0x20, 500);
    printf("--Bit 0x20 of controlByte is: %d\n\n", rbit);




    return asynSuccess;
}
























/** Code for iocsh registration */
static const iocshArg BeckCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg BeckCreateControllerArg1 = {"Number of consecutive controllers", iocshArgInt};
static const iocshArg BeckCreateControllerArg2 = {"Modbus INPUT port name", iocshArgString};
static const iocshArg BeckCreateControllerArg3 = {"Modbus OUTPUT port name", iocshArgString};
static const iocshArg BeckCreateControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg BeckCreateControllerArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const BeckCreateControllerArgs[] = {&BeckCreateControllerArg0,
                                                             &BeckCreateControllerArg1,
                                                             &BeckCreateControllerArg2,
                                                             &BeckCreateControllerArg3,
                                                             &BeckCreateControllerArg4,
                                                             &BeckCreateControllerArg5};
static const iocshFuncDef BeckCreateControllerDef = {"beckhoffCtrlCreate", 6, BeckCreateControllerArgs};
static void BeckCreateContollerCallFunc(const iocshArgBuf *args)
{
  BeckCreateController(args[0].sval, args[1].ival, args[2].sval, args[3].sval, args[4].ival, args[5].ival);
}

static void BeckRegister(void)
{
  iocshRegister(&BeckCreateControllerDef, BeckCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(BeckRegister);
}
