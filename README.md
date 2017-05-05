# beckMotor
--------------------
## Epics motor record device support for Beckhoff KL2541 stepper motor driver

_Developers_: Damiano Bortolato - Davide Marcato

Laboratori Nazionali di Legnaro - INFN

## Introduction

This the implementation of a motor record device support, via API model 3, for the Beckhoff KL2541. It is a stepper motor driver with up to 5A/ph current, limit switches and encoder support.
One or more modules can be physically attached to the beckhoff KL9100, which has two switched ethernet ports and exposes the module's registers to a modbus TCP port.
So this driver will use the epics modbus module as the lower communication channel. The KL2541 has two kinds of register, the "process communication" ones, which are just 
6 modbus registers (3 read + 3 write) and the "register communication" ones, which are 64 internal register with motor parameters and can be accesed through a series of 
read/writes to the modbus registers.
So we implemented an asynPortDriver port (called "motorDriver") to mask the various kinds of registers, and then implemented the motor record support 
(called "motorController", sorry!) reading and writing to the motorDriver port.

### Supported features
Absolute and relative movement, automatically stop at limit switches and prevent wrong direction, homing, static setup at startup.
**Not implemented**: encoder reading.

### Distribution
This device support is distributed as a separate module, which uses the epics motor module, in order to keep the two separated. This was discussed in some tech-talks threads as a good 
practice for the future of the motor record development.
However it could be easily integrated in the main motor record repository.
The whole program is released under the GPL license.

### Required modules
1. asyn (tested with R4-30, should work with not-too-old releases)
2. modbus (R2.8+)
3. motor (tested with R6-9)

### To-do
- add support for encoders
- switch from current modbus reading method (read the whole port and wait for interrupt) to new reading method with absolute addressing (each read is an actual modbusIO call) available in modbus 2.9

## How to install and use:
1. Download (via git clone or via zip file) and place it (extracted) inside a directory that we will call $(SUPPORT). This is usually the place where you keep all the epics modules.

1. Edit the configure/RELEASE file of the beckMotor with the correct paths of the required modules.

1. Compile: `make distclean; make` on top directory (beckMotor).

1. If you havent't already done, you should create your application, see the epics guide. Then add to your application's **configure/RELEASE** asyn, modbus, motor and this device support (beckMotor):
```
ASYN=$(SUPPORT)/asyn4-29
MODBUS=$(SUPPORT)/modbus-R2-8
MOTOR=$(SUPPORT)/motorR6-9
BECKMOTOR=$(SUPPORT)/beckMotor
```


1. Include in your **src makefile** modbus module (1.8+ required), asyn and motor dbd and libs. The required are:
```
yourIocName_DBD += asyn.dbd
yourIocName_DBD += drvAsynIPPort.dbd
yourIocName_DBD += modbusSupport.dbd
yourIocName_DBD += motorRecord.dbd
yourIocName_DBD += motorSupport.dbd
yourIocName_DBD += beckMotor.dbd
```
```
yourIocName_LIBS += asyn
yourIocName_LIBS += modbus
yourIocName_LIBS += motor
yourIocName_LIBS += beckMotor
```

1. Now let's move to **st.cmd**. Create a ip port:
```
#drvAsynIPPortConfigure(portName, hostInfo, priority, noAutoConnect, noProcessEos)
drvAsynIPPortConfigure("EK9100_3", "172.16.17.3:502", 0, 0, 1)
```

1. Create a modbus interpose port from modbus module
```
#modbusInterposeConfig(portName, linkType, timeoutMsec, writeDelayMsec)
modbusInterposeConfig("EK9100_3", 0, 2000, 0)
```

1. Create 2 modbus ports, one for the input registers and one for the output ones
    - Its width must be at least 6*n where n is the number of consecutive bechoff kl2541 modules
    - The poll time should be 0 for the input port (disable polling, triggering readings when needed), and positive for the output one (update initial value at startup)
    
        ```
        #drvModbusAsynConfigure(portName,   tcpPortName, slaveAddr, funct, startAddr, length, dataType, pollMsec, plcType);
        drvModbusAsynConfigure("inpRegs",  "EK9100_3",   1,         3,     0x0,       48,     0,        0,       "Beckhoff");
        drvModbusAsynConfigure("outRegs",  "EK9100_3",   1,         6,     0x800,     48,     0,        50,      "Beckhoff");
        ```
1. Create a driver port from the beckhoff support
    - This is the lower layer of the driver which translates read/write with particular reasons to a sequence of instruction to mascherate the readings of internal registers and modbus ones. The supported reasons are:
        - SB - r/w statusByte or 1st register of input port
        - DI - r/w dataIn or 2nd register of input port
        - SW - r/w statusWord or 3rd register of input port
        - CB - r/w controlByte or 1st register of output port
        - DO - r/w dataOut or 2nd register of output port
        - CW - r/w controlWord or 3rd register of output port
        - from R00 to R63 -r/w corresponding internal register
    - This port implements asynInt32 and asynUInt32Digital interfaces and can be used directly in records for r/w operation. However it may broke upper motor support due to concurrency. If you can try to never write with this port.
        - The syntax is:
        
        ```
        #BeckCreateDriver("portName",numberOfBeckModules, "inpModbusPort", "outmodbusPort")
        BeckCreateDriver("motorDriver",2, "inpRegs", "outRegs")`
        ```
        - where the numberOfBeckModules is the number of consecutive kl2541 to control
    
1. Create a controller port, from the beckhoff support
    - This is a port of type asynMotor and offers support to motor record.
    
    ```
    #BeckCreateController("portName", "driverPortName", movingPollms, idlePollms) 
    BeckCreateController("motorController", "motorDriver", 10, 100) `
    ```
    - where the last two values refers to the ms between a poll of the status of the module when moving and when still
    
1. **After ioc init has completed**, call the configuration commands. The general syntax is:
    
    ```
    #BeckConfigController(controller, axisRange, cmd, cmdArgs);
    ```
    - controller is the controller port name
    - axisRange indicates the axis to whom to apply this command; syntax is 3,5 for enumerations and 3-5 for ranges, or mix of them.
    - cmd is a string with the command to be executed
    - cmdArgs is a string with the args to be passed to cmd. You can write only the initial ones, stopping when you want. A whitespace means "do not modify".
    - The available cmd are:
        - #### INIT
            
            ```
            #BeckConfigController(controller, axisRange, init, "encoder, watchdog");
            BeckConfigController("motorController", "0-1", init, "0, 0");
            ```
            - encoder: [0/1] if an encoder is to be used. Up to now encoder is not supported, write 0.
            - whatchdog: [0/1] if a watchdog is to be used. 
        
        - #### INIT CURRENTS
            
            ```
            #BeckConfigController(controller, axisRange, initCurrents, "maxCurr, autoHoldinCurr, highAccCurr, lowAccCurrStr");
            BeckConfigController("motorController", "0-1", initCurrents, "1, 0.2");
            ```
            - maxCurr: the maximum ampere of your motor
            - autoHoldinCurr: ampere when still
            - highAccCurr: ampere after acc threshold
            - lowAccCurrStr: ampere before acc threshold
        
        - #### INIT HOMING PARAMS
            
            ```
            #BeckConfigController(controller, axisRange, initHomingParams, "refPosition, NCcontacts, lsDownOne, homeAtStartup, speedToHome, speedFromHome, emergencyAccl");
            BeckConfigController("motorController", "0-1", initHomingParams, "0, 0, 0, 0, 100, 100, 2047");
            ```
            - refPosition: the value of the position to set when homing complete
            - NCcontacts: [0/1] if the contacts are normally closed or not
            - lsDownOne: [0/1] if the input one refers to the low limit switch (as for motor internal countings) or vice versa
            - homeAtStartup: [-1, 0, 1] if to home or not at startup, in the direction indicated by the sign of the value
            - speedToHome: speed going towards the limit switch in homing procedure
            - speedFromHome: speed moving out of limit switch in homing procedure
            - emergencyAccl: the acceleration to stop motor when a limit switch is reached in homing procedure
        
        - #### INIT STEP RESOLUTION
            
            ```
            #BeckConfigController(controller, axisRange, initStepResolution, "microstepPerStep, stepPerRevolution");
            BeckConfigController("motorController", "0-1", initStepResolution, "64, 200");
            ```
            - microstepPerStep: how many microstep per step to set
            - stepPerRevolution: how many step per full revolution

1. Implement a database using controller port name for the motor records (as DTYP) and driver port name for others.
    - motor record example:

        ```
        #the main record for the motor
        record (motor, "Motor01") {
            field (DESC, "main record of the motor")
            field (DTYP, "asynMotor")
            field (OUT, "@asyn("motorController", $(NCHAN), $(TMOUT))")
            field (VMAX, "51200")         # max settable velo
            field (VELO, 12800)           # mstep/s
            field (VBAS, "100")           # starting velocity before accl
            field (ACCL, 0.3)             # accl time from vbas to velo
            field (BACC, 0.5)             # backlash acceleration
            field (HVEL, 5000)            # homing velocity
            field (S, 1)                  # speed: revolutions per second
            field (PINI, "1")
            field (DHLM, "2000000000")    # max position accepted high
            field (DLLM, "-2000000000")   # max position accepted low
            field (SREV, "12800")         # step per revolution
            field (SYNC, 1)               # at startup sync hw readback with val
        }
        ```
    - direct register access example:
        
        ```
        #run to reset hardware errors
        record (bo, "motorErrorReset") {
            field (DESC, "run to reset hardware errors")
            field (DTYP, "asynUInt32Digital")
            field (OUT, "@asynMask("motorDriver", $(NCHAN), 0x40, $(TMOUT))CB")
        }
        
        #a binary word with the status of the motor controller
        record (ai, "motorStatus") {
            field (DESC, "word with the status of the motor")
            field (DTYP, "asynInt32")
            field (INP, "@asyn("motorDriver", $(NCHAN), $(TMOUT))SW")
            field (SCAN, "1 second")
        }
        
        #read internal register 50: emergency acceleration
        record (ai, "readEmergencyAcceleration") {
            field (DESC, "read R50: emergency accl")
            field (DTYP, "asynInt32")
            field (INP, "@asyn("motorDriver", $(NCHAN), $(TMOUT))R50")
        }
        ```