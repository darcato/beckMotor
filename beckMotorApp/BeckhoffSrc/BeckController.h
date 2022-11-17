/**
 *  BeckDriver.h
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

#ifndef MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_
#define MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <asynPortClient.h>
#include "BeckDriver.h"
#include <vector>
#include <array>

/**
 * -------------------------------------------------
 * AXIS
 * -------------------------------------------------
 */
class epicsShareClass BeckAxis : public asynMotorAxis {
public:
  /* These are the methods we override from the base class */
  BeckAxis(class BeckController *pC, int axis);

  void report(FILE *fp, int level);

  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  //asynStatus setHighLimit(double highLimit);
  //asynStatus setLowLimit(double lowLimit);
  //asynStatus setPGain(double pGain);
  //asynStatus setIGain(double iGain);
  //asynStatus setDGain(double dGain);
  //asynStatus setClosedLoop(bool closedLoop);
  asynStatus setEncoderRatio(double ratio);
  asynStatus doMoveToHome();

  asynStatus hardReset();
  asynStatus softReset();

private:
  asynStatus updateCurrentPosition();
  asynStatus setAcclVelo(double min_velocity, double max_velocity, double acceleration);
  void setAutoStop(bool autoStop);

  BeckController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  //asynClients representing internal registers
  std::vector<asynInt32Client *> r_;
  //asynClients representing internal registers - bit access
  std::vector<asynUInt32DigitalClient *> ru_;

  //asynClients representing modbus registers
  asynInt32Client *statusByte_;
  asynInt32Client *dataIn_;
  asynInt32Client *statusWord_;
  asynInt32Client *controlByte_;
  asynInt32Client *dataOut_;
  asynInt32Client *controlWord_;

  //asynClients representing modbus registers
  asynUInt32DigitalClient *statusByteBits_;
  asynUInt32DigitalClient *dataInBits_;
  asynUInt32DigitalClient *statusWordBits_;
  asynUInt32DigitalClient *controlByteBits_;
  asynUInt32DigitalClient *dataOutBits_;
  asynUInt32DigitalClient *controlWordBits_;

  //a flag to indicate end of movement
  bool movePend;
  double currPos, lastDir;
  bool lHigh, lLow;
  bool limitSwitchDownIsInputOne, encoderEnabled;
  epicsInt32 microstepPerStep;
  double curr_min_velo, curr_max_velo, curr_home_velo, curr_acc;
  int curr_forw;
  bool exitingLimSw, startingHome;  //exitingLimSw = move has started, as soon as out of lim switches remember to enable lim switch auto stopping
                                    //startingHome = exiting limit switches, when out relaunch homing
  bool autoStop;
  int autoStopVal;

friend class BeckController;
};



/**
 * -------------------------------------------------
 * CONTROLLER
 * -------------------------------------------------
 */
class epicsShareClass BeckController : public asynMotorController {

//asynClients representing internal registers
  std::vector<asynInt32ArrayClient *> r_;

  //asynClients representing modbus registers
  asynInt32ArrayClient *statusByte_;
  asynInt32ArrayClient *dataIn_;
  asynInt32ArrayClient *statusWord_;
  asynInt32ArrayClient *controlByte_;
  asynInt32ArrayClient *dataOut_;
  asynInt32ArrayClient *controlWord_;
  asynInt32ArrayClient *memInp_;
  asynInt32ArrayClient *memOut_;

  //these arrays are populated by the poll of the controller, and read by axis pollers
  epicsInt32 *r0_cache, *r1_cache;
  std::vector< std::array<epicsInt32, 3> > memInp_cache;

protected:
  char *beckDriverPName_;

public:
  BeckController(const char *portName, const char *beckDriverPName, int numAxis, double movingPollPeriod, double idlePollPeriod );

  void report(FILE *fp, int level);
  BeckAxis* getAxis(asynUser *pasynUser);
  BeckAxis* getAxis(int axisNo);
  asynStatus poll();

  asynStatus init(int firstAxis, int lastAxis, bool encoder, bool watchdog, int encoderPpr, bool encoderInvert, bool autoStop);
  asynStatus initCurrents(int firstAxis, int lastAxis, double maxCurr, double autoHoldinCurr, double highAccCurr, double lowAccCurr);
  asynStatus initHomingParams(int firstAxis, int lastAxis, int refPosition, bool NCcontacts, bool lsDownOne, int homeAtStartup, double homingSpeed, double emergencyAccl);
  asynStatus initStepResolution(int firstAxis, int lastAxis, int microstepPerStep, int stepPerRevolution);
  asynStatus readUInt32DigitalArray(asynInt32ArrayClient *client, int *value, uint mask, size_t nElements, size_t *nIn);
  asynStatus writeUInt32DigitalArray(asynInt32ArrayClient *client, int *value, uint mask, size_t nElements);
  bool writeWithPassword(asynInt32ArrayClient *client, int value, uint mask, int nElem, const char *regName);
  bool writeWithPassword(asynInt32ArrayClient *client, int *value, uint mask, int nElem, const char *regName);
  void writeWithPasswordDirect(asynInt32ArrayClient *client, int *value, uint mask, int nElem, const char *regName);
  bool axisRangeOk(int *begin, int *end);

  friend class BeckAxis;
  friend BeckController * findBeckControllerByName(const char *name);
};


#endif /* MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_ */
