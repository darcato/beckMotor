# beckMotor
--------------------
## Epics motor record device support for Beckhoff KL2541 stepper motor driver

_Developers_: Damiano Bortolato - Davide Marcato

Laboratori Nazionali di Legnaro - INFN

## Introduction

This is the implementation of a motor record device support, via API model 3, for the Beckhoff KL2541. It is a stepper motor driver with up to 5A/ph current, limit switches and encoder support.
One or more modules can be physically attached to the beckhoff BK9100, which has two switched ethernet ports and exposes the module's registers to a modbus TCP port.
So this driver will use the epics modbus module as the lower communication channel. The KL2541 has two kinds of register, the "process communication" ones, which are just 
6 modbus registers (3 read + 3 write) and the "register communication" ones, which are 64 internal registers with motor parameters and can be accesed through a series of 
read/writes to the modbus registers.
So we implemented an asynPortDriver port (called "motorDriver") to mask the various kinds of registers, and then implemented the motor record support 
(called "motorController", sorry!) reading and writing to the motorDriver port.

### Supported features
Absolute and relative movement, automatically stop at limit switches and prevent wrong direction, homing, static setup at startup.
**Not implemented**: encoder reading, velocity controlled movement.

**WARNING:** remember to correctly configure limit switches (NC/NO contacts; which limit switch, low or high in movement coordinates, is connected to the first input) before starting operations, to avoid damages. Modifying the NC/NO settings requires a restart of the beckhoff modules.

### Distribution
This device support is distributed as a separate module, which uses the epics motor module, in order to keep the two separated. This was discussed in some tech-talks threads as a good 
practice for the future of the motor record development.
However it could be easily integrated in the main motor record repository.
The whole program is released under the GPL license.

### Required modules
0. epics base (tested with 3.14.12.5)
1. asyn (R4.33+)
2. modbus (R2.9+)
3. motor (tested with R6-9)

### To-do
- add support for encoders.
- velocity controlled movements.

## How to install and use:
1. Download (via git clone or via zip file) and place it (extracted) inside a directory that we will call $(SUPPORT). This is usually the place where you keep all the epics modules.

1. Edit the configure/RELEASE file of the beckMotor with the correct paths of the required modules.

1. Compile: `make distclean; make` on top directory (beckMotor).

1. If you havent't already done, you should create your application, see the epics guide. Then add to your application's **configure/RELEASE** asyn, modbus, motor and this device support (beckMotor):
```
ASYN=$(SUPPORT)/asyn4-33
MODBUS=$(SUPPORT)/modbus-R2-9
MOTOR=$(SUPPORT)/motorR6-9 (
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
#drvAsynIPPortConfigure(portName,  hostInfo,          priority, noAutoConnect, noProcessEos)
drvAsynIPPortConfigure("EK9100_3", "172.16.17.3:502", 0,        0,             1)
asynSetOption("EK9100_3",0, "disconnectOnReadTimeout", "Y")  #available since asyn 4-32
```

1. Create a modbus interpose port from modbus module
```
#modbusInterposeConfig(portName,  linkType, timeoutMsec, writeDelayMsec)
modbusInterposeConfig("EK9100_3", 0,        2000,        0)
```

1. Create 2 modbus ports, one for the input registers and one for the output ones
    - Its width must be at least 3*n where n is the number of consecutive bechoff kl2541 modules
    - The poll time should be 0 for the input port (disable polling, triggering readings when needed), and positive for the output one (update initial value at startup)
    - The startAddr must always be -1 to use absolute addressing
    - The two modbus functions used are: 3 for input registers, 16 for output ones
    
        ```
        #drvModbusAsynConfigure(portName,   tcpPortName, slaveAddr, funct, startAddr, length, dataType, pollMsec, plcType);
        drvModbusAsynConfigure("inpRegs",  "EK9100_3",   1,         3,     -1,        24,     0,        0,       "Beckhoff");
        drvModbusAsynConfigure("outRegs",  "EK9100_3",   1,         16,    -1,        24,     0,        50,      "Beckhoff");
        ```
1. Create a driver port from the beckhoff support
    - This is the lower layer of the driver which translates read/write with particular reasons to a sequence of instruction to mask the readings of internal registers and modbus ones. The supported reasons are:
        
        | reason     | meaning           | type           | access            | description       |
        | --------   | -------           | -------        | -------           | -------           |
        | SB         | statusByte        | asynInt32 / asynUInt32Digital / asynInt32Array   | R   | 1st register of input port  |
        | DI         | dataIn            | asynInt32 / asynUInt32Digital / asynInt32Array   | R   | 2nd register of input port  |
        | SW         | statusWord        | asynInt32 / asynUInt32Digital / asynInt32Array   | R   | 3rd register of input port  |
        | CB         | controlByte       | asynInt32 / asynUInt32Digital / asynInt32Array   | R/W | 1st register of ouput port  |
        | DO         | dataOut           | asynInt32 / asynUInt32Digital / asynInt32Array   | R/W | 2nd register of ouput port  |
        | CW         | controlWord       | asynInt32 / asynUInt32Digital / asynInt32Array   | R/W | 3rd register of ouput port  |
        | MI         | memoryIn          | asynInt32Array   | R   | all the input registers of n modules   |
        | MO         | memoryOut         | asynInt32Array   | R/W | all the output registers of n modules  |
        | R00 -> R63 | internal register | asynInt32 / asynUInt32Digital / asynInt32Array   | R/W | corresponding internal register |
    - This port implements asynInt32 and asynUInt32Digital interfaces and can be used directly in records for r/w operation. 
    Remember that some registers are write protected to reduce the stress on non-volatile memory: you first need to write to 0x1235 to register 31.
        - The syntax to create the port is:
        
        ```
        #BeckCreateDriver("portName",   startAddress,  numberOfBeckModules, "inpModbusPort", "outmodbusPort")
        BeckCreateDriver("motorDriver", 0,             8,                   "inpRegs",       "outRegs")
        ```
        - where the numberOfBeckModules is the number of consecutive kl2541 to control and the startAddress is the starting address of the first module
    
1. Create a controller port, from the beckhoff support
    - This is a port of type asynMotor and offers support to motor record.
    
    ```
    #BeckCreateController("portName",       "driverPortName", movingPollms, idlePollms) 
    BeckCreateController("motorController", "motorDriver",    100,          500) 
    ```
    - where the last two values refers to the ms between a poll of the status of the module when moving and when still. Please note that the polling is at least 20ms (for each controller, regardless of the number of axis) so avoid using too low values here which could saturate the available network resources. 
    
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
            #BeckConfigController(controller,       axisRange, init, "encoder, watchdog");
            BeckConfigController("motorController", "0-1",     init, "0,       0");
            ```
            - encoder: [0/1] if an encoder is to be used. Up to now encoder is not supported, write 0.
            - whatchdog: [0/1] if a watchdog is to be used. 
        
        - #### INIT CURRENTS
            
            ```
            #BeckConfigController(controller,       axisRange, initCurrents, "maxCurr, autoHoldinCurr, highAccCurr, lowAccCurrStr");
            BeckConfigController("motorController", "0-1",     initCurrents, "1,       0.2");
            ```
            - maxCurr: the maximum ampere of your motor
            - autoHoldinCurr: ampere when still
            - highAccCurr: ampere after acc threshold
            - lowAccCurrStr: ampere before acc threshold
        
        - #### INIT HOMING PARAMS
            
            ```
            #BeckConfigController(controller,       axisRange, initHomingParams, "refPosition, NCcontacts, lsDownOne, homeAtStartup, homingSpeed, speedFromHome, emergencyAccl");
            BeckConfigController("motorController", "0-1",  initHomingParams,    "0,           0,          0,         0,             500,           1500");
            ```
            - refPosition: the value of the position to set when homing complete
            - NCcontacts: [0/1] if the contacts are normally closed or not
            - lsDownOne: [0/1] if the input one refers to the low limit switch (as for motor internal countings) or vice versa
            - homeAtStartup: [-1, 0, 1] if to home or not at startup, in the direction indicated by the sign of the value
            - homingSpeed: speed going towards the limit switch in homing procedure
            - emergencyAccl: the acceleration to stop motor when a limit switch is reached in homing procedure
        
        - #### INIT STEP RESOLUTION
            
            ```
            #BeckConfigController(controller,       axisRange, initStepResolution, "microstepPerStep, stepPerRevolution");
            BeckConfigController("motorController", "0-1",     initStepResolution, "64,               200");
            ```
            - microstepPerStep: how many microstep per step to set (max 64)
            - stepPerRevolution: how many full step per full revolution

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
