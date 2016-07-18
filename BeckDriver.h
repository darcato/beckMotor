/*
 * BeckDriver.h
 *
 *  Created on: Jun 17, 2016
 *      Author: davide.marcato@lnl.infn.it
 */

#ifndef MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_
#define MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <asynPortClient.h>
#include "BeckPortDriver.h"

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
//  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving); //pool and read infos from beckhoff
//  asynStatus setPosition(double position);
//  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setCoilCurrent(double maxCurr, double autoHoldinCurr, double highAccCurr, double lowAccCurr);
  asynStatus resetController();

private:
  BeckController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  //asynUsers representing the 6 registers of the KL2541
  asynUser *statusByte_;
  asynUser *dataIn_;
  asynUser *statusWord_;
  asynUser *controlByte_;
  asynUser *dataOut_;
  asynUser *controlWord_;
  asynUser *r0_;
  asynUser *r1_;
  asynUser *r2_;
  asynUser *r3_;
  asynUser *r7_;
  asynUser *r8_;
  asynUser *r31_;
  asynUser *r35_;
  asynUser *r36_;
  asynUser *r38_;
  asynUser *r39_;
  asynUser *r40_;
  asynUser *r42_;
  asynUser *r43_;
  asynUser *r44_;

  //a flag to indicate end of movement
  bool moveDone, movePend;
  double currPos;
  double accl, velo, minVelo;

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
protected:
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  char *beckDriverPName_;


public:
  BeckController(const char *portName, const char *beckDriverPName, double movingPollPeriod, double idlePollPeriod );

  void report(FILE *fp, int level);
  BeckAxis* getAxis(asynUser *pasynUser);
  BeckAxis* getAxis(int axisNo);

  friend class BeckAxis;
  friend BeckController * findBeckControllerByName(const char *name);
};




#endif /* MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_ */
