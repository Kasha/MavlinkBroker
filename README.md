MavlinkBroker
========================

MavlinkBroker is a companion app, running on machines such as NVIDIA TX2 docked to the drone. Receiving/Sending unique and Natural language commands from other companion apps such as a "location detector by pictures" (When GPS connection is lost or not compatible with real location) and communicating/translating with Ardupilot/PX4. 

This process connects an external MAVLink UART device to send and receive data with companion apps sensors' data correction

Building
========

```
$ cd MavlinkBroker/
$ make
```

