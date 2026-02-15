# Release notes

### Note

Besides the binary here, download the files in the "data" folder: config.json (template), index.html, style.css.


## revision MB148

### Minor improvements

* GPIO output to shunt solar panel to keep battery voltage below 4.0V
* in 2-station mode report to APRS the RSSI of hearing from remote station
* in 2-station mode base listens to traffic directly one time slot every 8 sec
* report if TTGO version is running on a T-Beam
* show messages in Serial even if OLED not installed
* show "mobile" operation mode on OLED
* in mobile mode refresh web page after obtaining a GNSS fix
* change NZ freq to 869.20
* prevent crash if radio chip is not found
* improved sleep behavior in 2-station operation
* report remote background noise by channel to base even without traffic
* if APRS port is zero, test operation without connecting to APRS


## revision MB147

### Major improvements

Changes to help inaccessible remote station survive through winter


## revision MB146

### Minor improvements

Fixed and improved mobile mode


## revision MB145

### Major improvements

Fixed bugs in 2-station traffic timestamp and in single-station sleep


## revision MB144

### Major improvements

Cleaned up and improved the 2-station mode code

### Minor improvements

* Improved the station status reporting to OGN


## revision MB142

### Major improvements

Improved web interface to show more relevant status and statistics when using NTP time (requires new index.html & style.css)

### Minor improvements

Fixed bug that prevented use of "base sleep follows remote" when using NTP time


## revision MB141

### Major improvements

Corrected the processing of some data fields that were not being passed correctly in the 2-station mode

### Minor improvements

* Report to OGN the SNR (and BEC) of the original signal received by the remote station
* Removed the "ignore no-track" setting - no-track is never reported, stealth is always reported


## revision MB140

### Major improvements

* Support the LilyGo T3S3 board
* Re-organize time-relay to be symmetric in both directions
 
### Minor improvements

Report remote station stats even when it is not sending time to base station


## revision MB139

### Minor improvements

Measure radio noise level and use it for calculating SNR and to collect noise statistics


## revision MB138

### Major improvements
* Fixed bugs that prevented decryption of FLARM packets in Europe
* Fixed: no charging of the battery in T-Beam v1.2


## revision MB137

### Minor improvements

Save settings from web page into config.json in "pretty" human-readable format


## revision MB136

### Major improvements
* Support the new (March 2024) radio protocol.  Still supports the old protocol too.
* GNSS no longer needed.  Base station can get accurate time from NTP, and optionally send it to the remote station.
* Correction of 1-4 bit burst errors, to increase reporting range and frequency.

### Minor improvements
* Turn GNSS module off if not needed.
* Improved handling of packets air-relayed by SoftRF.
* Can set a custom private port for (remotely) accessing the web interface.
* Detect and use OLED display on either I2C port.
* Report the radio chip type on the web page and in APRS status messages.


## revision MB128

### Major improvements

Support T-Beam v0.7 (as well as v1.0-v1.2).


## revision MB120

### Major improvements

Support T-Beam v1.2 with AXP2101 (as well as the older v1.1 with AXP192).


## revision MB111

### Major improvements

Fixed: WiFi not starting when no config file loaded.


## revision MB110

### Major improvements

Handling of packets air-relayed by SoftRF.


## revision MB108

### Major improvements

Further improved handling of intermittent WiFi.


## revision MB107

### Major improvements

Better handling of WiFi dropouts.  Debug log facility.  Minor bug fixes.


## revision MB106

### Major improvements

Nothing major.  Better formatting of web pages.  Removed some debug output.


## revision MB105

### Major improvements

Show on upload or status page whether the config.json file was found and parsed OK.


## revision MB104

### Major improvements

More transparency in config file upload: show list of existing files, allow downloading the config.json file.


## revision MB103

### Major improvements

Corrected the login to OGN APRS, which finally gets the messages to be passed to clients in the right format.


## revision MB102

### Major improvements

Corrected the format of the messages sent to OGN APRS.

