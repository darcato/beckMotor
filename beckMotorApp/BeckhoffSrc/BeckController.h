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
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
//  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards); //this may become goUp goDown
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving); //pool and read infos from beckhoff
//  asynStatus setClosedLoop(bool closedLoop);
  asynStatus initCurrents(double maxCurr, double autoHoldinCurr, double highAccCurr, double lowAccCurr);
  asynStatus initHomingParams(int refPosition, bool NCcontacts, bool lsDownOne, int homeAtStartup, double homingSpeed, double emergencyAccl);
  asynStatus initStepResolution(int microstepPerStep, int stepPerRevolution);
  asynStatus hardReset();
  asynStatus softReset();
  asynStatus init(bool encoder, bool watchdog);

private:
  asynStatus updateCurrentPosition();
//  asynStatus directMove(int position, int goCmd);
//  asynStatus exitLimSw(bool usePos, int newPos);
  asynStatus setAcclVelo(double min_velocity, double max_velocity, double acceleration);

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
  bool limitSwitchDownIsInputOne;
  epicsInt32 microstepPerStep;
  double curr_min_velo, curr_max_velo, curr_home_velo, curr_acc;
  int curr_forw;
  int lastr2, lastr3;
  bool exitingLimSw, startingHome;  //exitingLimSw = move has started, as soon as out of lim switches remember to enable lim switch auto stopping
                                    //startingHome = exiting limit switches, when out relaunch homing

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
  epicsInt32 *r1_cache;
  std::vector< std::array<epicsInt32, 3> > memInp_cache;

  bool pHighAlreadyRead = false;

protected:
  char *beckDriverPName_;

public:
  BeckController(const char *portName, const char *beckDriverPName, double movingPollPeriod, double idlePollPeriod );

  void report(FILE *fp, int level);
  BeckAxis* getAxis(asynUser *pasynUser);
  BeckAxis* getAxis(int axisNo);
  asynStatus poll();

  friend class BeckAxis;
  friend BeckController * findBeckControllerByName(const char *name);
};


#endif /* MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_ */
