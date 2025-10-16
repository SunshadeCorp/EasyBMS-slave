# EasyBMS

Simple Wifi based BMS for battery modules from EVs

This battery management system is a combination of an openhardware board, the IC LTC6804-1 and a WLAN-enabled controller ESP32-C6 and a bit of opensource code. 

12-cell passive BMS per board with a WLAN interface suitable for lithium-ion NMC battery modules.

Cost per BMS board <20€

Balancing across modules possible through a master unit

Two operating modes

    Single - the cells of one battery module are balanced standalone with only one BMS board
    Multi - the cells of several battery modules are balanced via several, distributed BMS boards and coordinated via head unit
    
Balancing with relatively low current ~40mA, which is sufficient for EV battery modules.
All BMS data is provided to MQTT

## Build

- `git clone https://github.com/SunshadeCorp/EasyBMS-slave.git`
- `cd EasyBMS-slave`
- `git submodule init`
- `git submodule update`
- `cd include`
- `cp config.h.example config.h`
- edit `config.h` to your setup
- build with Visual Studio Code and PlatformIO
