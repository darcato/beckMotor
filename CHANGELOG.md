# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.1.2] - 2018-05-18
### Fixed
- Fixed compiler errors with older compilers
- Fixed wrong error print at startup with encoders and microstep set to 1

## [2.1.1] - 2018-04-20
### Fixed
- Fixed crash when beckhoff not connected

## [2.1.0] - 2018-04-20
### Added
- Support for encoders
- Support for jog movements (moveVelocity) - the motor moves with a certain velocity in one direction until stopped
- Added documentation for motor record usage

### Changed
- Performace improvements on the config commands when used on axis ranges.

## [2.0.1] - 2018-04-03
### Fixed
- Using new asynPortDriver constructor for asyn >= 4.32. This removes compiling warnings/errors depending on your EPICS base version.
- Removed compile warnings for unused variables
- Fixed wrong position after homing procedure in some cases

## [2.0.0] - 2018-01-25
### Changed
- HUGE performance inprovements, especially when using multiple axes. The poll method has been completely rewritten to minimize network access. Now a single modbus I/O is executed per each poll (in the controller poll) to update the position and status of all the axes, requiring just about 15-20ms, instead of about 70-100ms per axis. This enables much faster movement startup.
- beckDriver now implements asynInt32Array to read/write values from multiple modules at the same time.
- now using modbus absolute addressing to have syncronous readings.
### Fixed
- fixed wrong record value at startup
- fixed multiple errors printings at startup
- fixed moving reporting to motor record to set polling period
- other bug fixes

## [1.1.0] - 2017-12-15
### Added
- Implemented STOP function
- Small efficiency improvements

## [1.0.0] - 2017-05-24
### Added
- First release
- Position control movements
- Limit switches control
- Homing
- Startup commands for configuration
- Separate module from epics motor

