
#include <asynPortDriver.h>
#include <asynPortClient.h>
#include <vector>
#include <epicsEvent.h>


/**
 * -------------------------------------------------
 * INT32CLIENT
 * -------------------------------------------------
 */
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

	int roffset;

	std::vector<asynInt32Client *> statusByte_;
	std::vector<asynInt32Client *> dataIn_;
	std::vector<asynInt32Client *> statusWord_;
	std::vector<asynInt32Client *> controlByte_;
	std::vector<asynInt32Client *> dataOut_;
	std::vector<asynInt32Client *> controlWord_;

	//user to trigger an immediate reading of all the input modbus port, with each reagister updated
	asynInt32Client triggerReadIn_;
	//call readWait on this to wait for new data to be read by modbusIO
	asynInt32ClientSyncIO newDataIn_;

public:
	BeckPortDriver(const char *portName, int nCtrl, const char *inModbusPort, const char *outModbusPort);
	asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

private:
	asynStatus writeReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 value);
	asynStatus readReg(epicsInt32 regN, epicsInt32 axis, epicsInt32 *value);
	asynStatus writeProc(asynInt32Client *modbusReg, epicsInt32 value);
	asynStatus readProc(asynInt32Client *modbusReg, bool in, epicsInt32 *value);
};

