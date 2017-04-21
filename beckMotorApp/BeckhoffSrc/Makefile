# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

#will create a support called beckmotor
LIBRARY_IOC += beckMotor

#will create a dbd called beckmotor
DBD += beckMotor.dbd

#to be added to beckMotor.dbd
beckMotor_DBD += devBeckMotor.dbd


# Beckhoff KL2541 driver support.
beckMotor_SRCS += BeckDriver.cpp
beckMotor_SRCS += BeckPortDriver.cpp

beckMotor_LIBS += motor
beckMotor_LIBS += asyn
beckMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
