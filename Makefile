# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devBeckMotor.dbd

LIBRARY_IOC = Beck

#SRCS += BeckRegister.cc

# Advanced Control Systems driver support.
#SRCS += devMCB4B.c drvMCB4B.c
SRCS += BeckDriver.cpp

Beck_LIBS += motor asyn
Beck_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

