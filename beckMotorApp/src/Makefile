TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += beckMotor

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install beckMotor.dbd into <top>/dbd
DBD += beckMotor.dbd

# specify all source files to be compiled and added to the library
#beckMotor_SRCS += xxx

beckMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

