CANopen_OS
==========

CanFestival CANOpenShell modified to deliver and receive commands via CANopen OS interface

Assumes CanFestival dictionary utils installed and in execution path.

I made this modification to understand better the use of semaphores for synchronization between
the CanFestival read thread and blocking SDO read/writes.

I tested it on ELMO Whistle drives but it should work on any device implementing the CANopen OS
interface.



