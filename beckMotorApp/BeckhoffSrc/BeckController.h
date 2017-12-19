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
  asynStatus initHomingParams(int refPosition, bool NCcontacts, bool lsDownOne, int homeAtStartup, double speedToHome, double speedFromHome, double emergencyAccl);
  asynStatus initStepResolution(int microstepPerStep, int stepPerRevolution);
  asynStatus hardReset();
  asynStatus softReset();
  asynStatus init(bool encoder, bool watchdog);

private:
  asynStatus updateCurrentPosition();
  asynStatus directMove(int position, int goCmd);
  asynStatus exitLimSw(bool usePos, int newPos);

  BeckController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  //asynUsers representing modbus registers
  asynUser *statusByte_;
  asynUser *dataIn_;
  asynUser *statusWord_;
  asynUser *controlByte_;
  asynUser *dataOut_;
  asynUser *controlWord_;

  //asynUsers representing internal registers
  asynUser *r0_;
  asynUser *r1_;
  asynUser *r2_;
  asynUser *r3_;
  asynUser *r7_;
  asynUser *r8_;
  asynUser *r31_;
  asynUser *r32_;
  asynUser *r33_;
  asynUser *r35_;
  asynUser *r36_;
  asynUser *r38_;
  asynUser *r39_;
  asynUser *r40_;
  asynUser *r42_;
  asynUser *r43_;
  asynUser *r44_;
  asynUser *r46_;
  asynUser *r50_;
  asynUser *r52_;
  asynUser *r53_;
  asynUser *r54_;
  asynUser *r55_;
  asynUser *r56_;
  asynUser *r58_;

  //a flag to indicate end of movement
  bool movePend;
  double currPos, lastDir;
  bool lHigh, lLow;
  epicsUInt32 lLowRepetitions, lHighRepetitions, topRepetitions;
  bool limitSwitchDownIsInputOne;
  epicsInt32 microstepPerStep;
  double curr_min_velo, curr_max_velo, curr_acc;
  int lastr2, lastr3;
  bool exitingLimSw, homing;
  //util methods
  asynStatus setAcclVelo(double min_velocity, double max_velocity, double acceleration);

friend class BeckController;
};



/**
 * -------------------------------------------------
 * CONTROLLER
 * -------------------------------------------------
 */
class epicsShareClass BeckController : public asynMotorController {

  //asynUser *inputRegs_;
  //asynUser *outputRegs_;
  asynUser *statusByte_;
  asynUser *statusWord_;
  asynUser *r0_;
  asynUser *r1_;

  //these arrays are populated by the poll of the controller, and read by axis pollers
  epicsInt32 *r0_cache;
  epicsInt32 *r1_cache;
  epicsInt32 *statusByte_cache;
  epicsInt32 *statusWord_cache;


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
