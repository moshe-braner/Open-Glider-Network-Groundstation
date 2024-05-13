# OGNbase


New in version MB136:  Supports the new (March 2024) radio protocol.  Can get exact time from NTP - GNSS not needed.  Added bit error correction to increase reporting range and frequency.  Allow setting the web server port - can then access it remotely.  Detect and use OLED display on either I2C port.


### Groundstation for Open Glider Network with ESP32

The aim of this project is to create a simple base station for the OGN network. The SoftRF project was used as the base. Thanks to Linar Yusupov for this work!

A TTGO T-Beam (SoftRF Prime Edition MkII) is used as hardware, which has an ESP32 processor, Wifi, GNSS module and a radio chip.  Since the T-Beam uses very low power, it can be operated very easily on a battery with a small solar panel.

The T-Beam does not need another computer (Raspberry) - it sends the APRS messages directly into the Open Glider network.
Only WiFi access and the station callsign have to be configured, the position is determined via GPS.

OGNbase also offers the "relay" mode of operation, where the "remote" station is located at a high point for good reception of signals from aircraft, and the "base" station is located where there is internet access.  The two stations communicate via radio. 

There are also a few drawbacks relative to the traditional OGN receivers. Only one protocol can be decoded at the same time.  At the moment the second protocol has no function.  Only messages in the FLARM protocol is decoded, messages in the OGNTP protocol are ignored. 

Both source code and compiled binaries are available here.  For full documentation see the [documentation file](https://github.com/moshe-braner/Open-Glider-Network-Groundstation/blob/main/ognbase/documentation/documentation.txt).

Note: version MB120+ is compatible with the newer version v1.2 of the T-Beam, with the AXP2101 - as well as with the older version v1.1 with the AXP192.  Firmware update via WiFi sometimes fails - hangs within async_tcp and then auto-reboots - keep trying.

