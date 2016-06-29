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

private:
  BeckController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  asynUser *statusByte_;
  asynUser *dataIn_;
  asynUser *statusWord_;
  asynUser *controlByte_;
  asynUser *dataOut_;
  asynUser *controlWord_;
  asynUser *triggerRead_;

  //util methods
  asynStatus readPosition(double *position);
//  asynStatus sendAccelAndVelocity(double accel, double velocity);

friend class BeckController;
};


class epicsShareClass BeckController : public asynMotorController {
protected:
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  char *inModbusPName_;
  char *outModbusPName_;


public:
  BeckController(const char *portName, const int numAxis, const char *inModbusPName, const char *outModbusPName, double movingPollPeriod, double idlePollPeriod );

  void report(FILE *fp, int level);
  BeckAxis* getAxis(asynUser *pasynUser);
  BeckAxis* getAxis(int axisNo);

  friend class BeckAxis;
};




#endif /* MOTORAPP_BECKHOFFSRC_BECKDRIVER_H_ */
