/*
   ognbase(.ino) firmware
   Copyright (C) 2020 Manuel Roesel

   Author: Manuel Roesel, manuel.roesel@ros-it.ch

   Web: https://github.com/roema/Open-Glider-Network-Groundstation

   Credits:
     Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
     AVR/Arduino nRF905 Library/Driverr is developed by Zak Kemble, contact@zakkemble.co.uk
     flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
     Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
     "Aircraft" and MAVLink Libraries are developed by Andy Little
     TinyGPS++ and PString Libraries are developed by Mikal Hart
     Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
     TrueRandom Library is developed by Peter Knight
     IBM LMIC and Semtech Basic MAC frameworks for Arduino are maintained by Matthijs Kooijman
     ESP8266FtpServer is developed by David Paiva
     Lib_crc is developed by Lammert Bies
     OGN library is developed by Pawel Jalocha
     NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
     ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
     Arduino Core for ESP32 is developed by Hristo Gochkov
     ESP32 BT SPP library is developed by Evandro Copercini
     Adafruit BMP085 library is developed by Limor Fried and Ladyada
     Adafruit BMP280 library is developed by Kevin Townsend
     Adafruit MPL3115A2 library is developed by Limor Fried and Kevin Townsend
     U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
     NeoPixelBus library is developed by Michael Miller
     jQuery library is developed by JS Foundation
     EGM96 data is developed by XCSoar team
     BCM2835 C library is developed by Mike McCauley
     SimpleNetwork library is developed by Dario Longobardi
     ArduinoJson library is developed by Benoit Blanchon
     Flashrom library is part of the flashrom.org project
     Arduino Core for TI CC13X0 and CC13X2 is developed by Robert Wessels
     EasyLink library is developed by Robert Wessels and Tony Cave
     Dump978 library is developed by Oliver Jowett
     FEC library is developed by Phil Karn
     AXP202X library is developed by Lewis He
     Arduino Core for STM32 is developed by Frederic Pillon
     TFT library is developed by Bodmer
     STM32duino Low Power and RTC libraries are developed by Wi6Labs
     Basic MAC library is developed by Michael Kuyper
     LowPowerLab SPIFlash library is maintained by Felix Rusu
     Arduino core for ASR650x is developed by Aaron Lee (HelTec Automation)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "OTA.h"
#include "Time.h"

#include "GNSS.h"
#include "RF.h"

#include "EEPROM.h"
#include "Battery.h"
//#include "NMEA.h"
#include "SoC.h"
#include "WiFi.h"
#include "Web.h"

#include "TTN.h"
#include "Traffic.h"

#include "APRS.h"
#include "RSM.h"
#include "PNET.h"
#include "MONIT.h"
#include "OLED.h"
#include "Log.h"
#include "global.h"
#include "version.h"
#include "config.h"
#include "SPIFFS.h"

#include <rom/rtc.h>

#include <TimeLib.h>

//one of the following needs to be defined in build_opt.h (not here):

#if !defined(TBEAM) && !defined(TTGO) && !defined(T3S3)
#error No board defined
#endif

#if defined(TBEAM) && defined(TTGO)
#error Multiple boards defined
#endif
#if defined(TBEAM) && defined(T3S3)
#error Multiple boards defined
#endif
#if defined(TTGO) && defined(T3S3)
#error Multiple boards defined
#endif

#if defined(ENABLE_AHRS)
#include "AHRS.h"
#endif /* ENABLE_AHRS */

#if LOGGER_IS_ENABLED
#include "Log.h"
#endif /* LOGGER_IS_ENABLED */

#define DEBUG 0
#define DEBUG_TIMING 0

#define seconds() (millis()/1000)

#define isTimeToExport() (millis() > ExportTimeMarker + 1000)

#define APRS_EXPORT_AIRCRAFT 6
#define TimeToExportOGN() (seconds() >= ExportTimeOGN + APRS_EXPORT_AIRCRAFT)

#define APRS_REGISTER_REC 689
#define TimeToRegisterOGN() (seconds() >= ExportTimeRegisterOGN + APRS_REGISTER_REC)

#define APRS_KEEPALIVE_TIME 239
#define TimeToKeepAliveOGN() (seconds() >= ExportTimeKeepAliveOGN + APRS_KEEPALIVE_TIME)

#define APRS_CHECK_KEEPALIVE_TIME 5   // <<< was 21
#define TimeToCheckKeepAliveOGN() (seconds() >= ExportTimeCheckKeepAliveOGN + APRS_CHECK_KEEPALIVE_TIME)

#define APRS_CHECK_WIFI_TIME 133
#define TimeToCheckWifi() (seconds() >= ExportTimeCheckWifi + APRS_CHECK_WIFI_TIME)

#define APRS_STATUS_REC 233
#define TimeToStatusOGN() (seconds() >= ExportTimeStatusOGN + APRS_STATUS_REC)

#define APRS_PROTO_SWITCH 2
#define TimeToswitchProto() (seconds() >= ExportTimeSwitch + APRS_PROTO_SWITCH)

#define TIME_TO_REFRESH_WEB 23
#define TimeToRefreshWeb() (seconds() >= ExportTimeWebRefresh + TIME_TO_REFRESH_WEB)

#define TimeToSleep() (ogn_rxidle > 0 && seconds() >= ExportTimeSleep + ogn_rxidle)

//testing
#define TIME_TO_DIS_WIFI  333
#define TimeToDisWifi() (seconds() >= ExportTimeDisWifi + TIME_TO_DIS_WIFI)

//testing
#define TimeToDisableOled() (seconds() >= ExportTimeOledDisable + oled_disable)

//time reregister if failed
#define TIME_TO_REREG 29
#define TimeToReRegisterOGN() (seconds() >= ExportTimeReRegister + TIME_TO_REREG)

/*Testing FANET service messages*/
#define TIME_TO_EXPORT_FANET_SERVICE 40 /*every 40 sec 10 for testing*/
#define TimeToExportFanetService() (seconds() >= ExportTimeFanetService + TIME_TO_EXPORT_FANET_SERVICE)

ufo_t ThisAircraft;
bool groundstation = false;
int ground_registred = 0;
bool fanet_transmitter = false;
int proto_in_use = 0;

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
};

RTC_DATA_ATTR uint32_t deep_sleep_counter = 0;

unsigned long setup_done_ms = 0;
unsigned long LEDTimeMarker = 0;
unsigned long ExportTimeMarker = 0;

unsigned long ExportTimeOGN = 0;
unsigned long ExportTimeRegisterOGN = 0;
unsigned long ExportTimeKeepAliveOGN = 0;
unsigned long ExportTimeStatusOGN = 0;
unsigned long ExportTimeSwitch = 0;
unsigned long ExportTimeSleep = 0;
unsigned long ExportTimeDisWifi = 0;
unsigned long ExportTimeWebRefresh = 0;
unsigned long ExportTimeFanetService = 0;
unsigned long ExportTimeCheckKeepAliveOGN = 0;
unsigned long ExportTimeCheckWifi = 0;
unsigned long ExportTimeOledDisable = 0;
unsigned long ExportTimeReRegister = 0;

/*set ground position only once*/
bool position_is_set = false;
/*
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}
*/

void setup()
{
  rst_info *resetInfo;
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order
  Battery_setup();
  resetInfo = (rst_info *) SoC->getResetInfoPtr();

  //Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  Serial.begin(SERIAL_OUT_BR);    // this may be USBSerial if ESP32S3 & not CDC
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    float volts = Battery_voltage();
    bool low_bat = (volts < 3.7 && volts > 2.0);
    // 3.7 is intentionally higher than the 3.65V triggering sleep start
    if ((deep_sleep_counter > 0 && deep_sleep_counter < 24) || low_bat) {
      if (low_bat) {
        Serial.println(F("----> battery low, keep sleeping"));
      } else {
        Serial.print(F("----> ")); Serial.print(deep_sleep_counter); Serial.println(F(" more hours to sleep"));
      }
      delay(600);
#ifdef TBEAM
      GNSS_sleep();
      delay(300);
      turn_GNSS_off();   // because GNSS_sleep() didn't always work
      turn_LED_off();
      delay(300);
#endif
      if (deep_sleep_counter > 0)
          --deep_sleep_counter;
      esp_sleep_enable_timer_wakeup(3600*1000000ULL);
//      esp_sleep_enable_ext0_wakeup(GPIO_NUM_26,1);           // this is IO0 on the LORA module
      esp_deep_sleep_start();
   } else {
      Serial.println(F("====> woke up from deep sleep"));
   }
  }
  deep_sleep_counter = 0;

  Serial.println();
  Serial.print(F(SOFTRF_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("By Manuel Roesel and Moshe Braner"));
  Serial.println();
  ESP32_post_setup();   // report detected hardware
  Serial.println();
  Serial.flush();

  if (resetInfo) {
    Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
    }
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(SoC->getFreeHeap());
  Serial.println(SoC->getResetInfo()); Serial.println("");

  EEPROM_setup();

  if (OLED_probe())
    OLED_setup();
  else
    Serial.println("OLED not found");

  WiFi_setup();

  SoC->Button_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;

  hw_info.rf = RF_setup();

  if (hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
      hw_info.revision == 2                      &&
      RF_SX12XX_RST_is_connected)
  {
    hw_info.revision = 5;
  }

  delay(100);

#if defined(TBEAM)
  if (ogn_gnsstime)
    hw_info.gnss = GNSS_setup();
  else
    turn_GNSS_off();
#endif
  
  ThisAircraft.aircraft_type = settings->aircraft_type;
  Traffic_setup();

  SoC->swSer_enableRx(false);

  OTA_setup();
  delay(2000);

  Web_setup(&ThisAircraft);

  Time_setup();

  SoC->WDT_setup();

  if(private_network || remotelogs_enable){
    aes_init();
  }

  OpenDebugLog();

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
      DebugLogWrite("woke up from deep sleep");
}

void pre_shutdown()
{
  SoC->WDT_fini();
  SoC->swSer_enableRx(false);
  Web_fini();
  WiFi_fini();
  RF_Shutdown();
  SoC->Button_fini();
}

void shutdown(const char *msg)
{
  OLED_enable();
  Serial.println(msg);
  OLED_write(msg, 0, 27, true);
  pre_shutdown();
  delay(1500);
  OLED_disable();

  SoC_fini();
}

void loop()
{

//Serial.println("RF_loop...");

  // Do common RF stuff first
  RF_loop();

//Serial.println("AP check...");

  if((WiFi.getMode() == WIFI_AP) && !ognrelay_enable){
    char buf[32];
    OLED_write("Setup mode..", 0, 9, true);
    snprintf (buf, sizeof(buf), "SSID: %s", host_name.c_str());
    OLED_write(buf, 0, 18, false);
    snprintf (buf, sizeof(buf), "ip: %s", "192.168.1.1");
    OLED_write(buf, 0, 27, false);
    snprintf (buf, sizeof(buf), "reboot in %d seconds", 600 - seconds());
    OLED_write(buf, 0, 36, false);
    snprintf (buf, sizeof(buf), "Version: %s ", _VERSION);
    OLED_write(buf, 0, 45, false);    
    delay(1000);
    if(seconds() > 600){
      DebugLogWrite("reset after 600 sec in AP mode");
      delay(700);
      pre_shutdown();
      SoC->reset();
    }
  } else {

    ground();
  }

//Serial.println("WiFi_loop...");

  // Handle DNS
  WiFi_loop();

//Serial.println("Web_loop...");

  // Handle Web
  /*MANU add timer to refresh values*/
  if(TimeToRefreshWeb()){
    Web_loop();
    ExportTimeWebRefresh = seconds();
  }

  yield();

//Serial.println("OTA_loop...");

  // Handle OTA update.
  OTA_loop();

//Serial.println("SoC_loop...");

  SoC->loop();

//Serial.println("Battery_loop...");

  Battery_loop();

//Serial.println("Button_loop...");

  SoC->Button_loop();

  yield();
}

void ground()
{

   const char *disp;
   bool success;
   String msg;
   char buf[32];

//Serial.println("position...");

  if (! position_is_set && ! ogn_mobile && ogn_lat != 0 && ogn_lon != 0) {
    ThisAircraft.latitude = ogn_lat;
    ThisAircraft.longitude = ogn_lon;
    ThisAircraft.altitude = ogn_alt;
    ThisAircraft.course = 0;
    ThisAircraft.speed = 0;
    ThisAircraft.hdop = 0;
    ThisAircraft.geoid_separation = ogn_geoid_separation;

#if defined(TBEAM)
    if (! ogn_gnsstime) {
      GNSS_sleep();
      DebugLogWrite("gnss sleep, position from config");
    }
#endif

    Serial.println("Position set from config file");

    msg = "found position data LAT: ";
    msg += ogn_lat;
    msg += " LON: ";
    msg += ogn_lon;
    msg += " ALT: ";
    msg += ogn_alt;
    Logger_send_udp(&msg);

    position_is_set = true;

  }

#if defined(TBEAM)

//Serial.println("GNSS_loop...");

  GNSS_loop();

  if (!position_is_set || ogn_gnsstime) {

    if ((!position_is_set || ogn_mobile) && isValidFix()) {
    
      ThisAircraft.latitude = gnss.location.lat();
      ThisAircraft.longitude = gnss.location.lng();
      ThisAircraft.altitude = gnss.altitude.meters();
      //ThisAircraft.course = gnss.course.deg();
      //ThisAircraft.speed = gnss.speed.knots();
      ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
      ThisAircraft.geoid_separation = gnss.separation.meters();
#if !defined(EXCLUDE_EGM96)
      if (ThisAircraft.geoid_separation == 0) {   // faulty GNSS modules
          ThisAircraft.geoid_separation = LookupSeparation(ThisAircraft.latitude, ThisAircraft.longitude);
          // assume the altitude reported by such GNSS modules is ellipsoid
          // so correct it to MSL
          ThisAircraft.altitude -= ThisAircraft.geoid_separation;
      }
#endif

      ogn_lat = gnss.location.lat();
      ogn_lon = gnss.location.lng();
      ogn_alt = gnss.altitude.meters();
      ogn_geoid_separation = ThisAircraft.geoid_separation;

      if (!position_is_set) {
          Serial.println("Position set from GNSS");
          msg = "GPS fix LAT: ";
          msg += gnss.location.lat();
          msg += " LON: ";
          msg += gnss.location.lng();
          msg += " ALT: ";
          msg += gnss.altitude.meters();
          Logger_send_udp(&msg);    
          position_is_set = true;    
      }

      if (! ogn_gnsstime && ! ogn_mobile) {
        GNSS_sleep();
        DebugLogWrite("gnss sleep, position from gnss");
      }

    }

  if(!position_is_set){
    Serial.println(F("still no position..."));
    OLED_write("no position data", 0, 18, true);
    delay(1200);
    if (ogn_gnsstime && !isValidFix()) {
        OLED_write("waiting for GPS fix", 0, 18, true);
        delay(1200);
    }
    OLED_info();
  }

  }

#else

  if(!position_is_set){
    Serial.println(F(">>> no position data found"));
    OLED_write("no position data found", 0, 18, true);
    delay(2000);
    OLED_info();
  }

#endif

    yield();

//Serial.println("ground...");

   if (position_is_set && !groundstation) {

    float lon = ThisAircraft.longitude;
    if (lon < 0)   lon += 360.0;
    ogn_timezone = (int)(lon / 15.0 + 0.5);   // 0...24
    if (ogn_timezone > 12)  ogn_timezone -= 24;   // -11...12

    groundstation = true;
    setup_done_ms = millis();

    Serial.print("good morning, startup esp32 groundstation at ms=");
    Serial.println(millis());

    msg = "good morning, startup esp32 groundstation ";
    msg += "version ";
    msg += String(_VERSION);
    msg += " after ";
    msg += String(SoC->getResetInfo());
    Logger_send_udp(&msg);

    ExportTimeSleep = seconds();
  }

//Serial.println("Time_loop...");

  /* time-relay stuff */
  Time_loop();

  yield();

//Serial.println("RF_Receive...");

  success = RF_Receive();

  if (success && position_is_set) {
      // Logger_send_udp(&msg);
//Serial.println("Parse...");
      uint32_t rx = traffic_packets_recvd;
      ParseData();
      if (traffic_packets_recvd > rx)   /* ignore time-sync packets */
        ExportTimeSleep = seconds();    /* actual traffic postpones sleep */
  }

  yield();

//Serial.println("Traffic_loop...");
  Traffic_loop();  /* if relay station, this is where relaying happens */

  //only as basestation (or standalone)

  if (!ognrelay_enable) {

    LogDate();  // once

    if (TimeToRegisterOGN() || ground_registred == 0) {  
      bool time_ok;
      if (ognrelay_time || ognreverse_time)
          time_ok = time_synched;
      else if(ogn_gnsstime)
          time_ok = isValidGNSStime();
      else
          time_ok = NTP_synched;
      if ((OurTime > 1000000) && (ThisAircraft.second != 0)
             && groundstation && (millis() > setup_done_ms + 30000)
             && position_is_set && time_ok && (WiFi.getMode() == WIFI_STA)) {
        Serial.println(F("Registering OGN..."));
        OLED_write("Registering OGN...", 0, 18, true);
        ground_registred = OGN_APRS_Register(&ThisAircraft);
        if (ground_registred == 1)  OLED_write("Registered OGN OK", 0, 27, false);
        msg = "current UTC time ";
        msg += OurTime;
        Logger_send_udp(&msg);
        Serial.println(msg.c_str());
        ExportTimeRegisterOGN = seconds();
        if (SoC->getFreeHeap() < 30000) {
            Serial.print(F("Free heap size: "));
            Serial.println(SoC->getFreeHeap());
            DebugLogWrite("free heap < 30000");
        }
      }
    }

    if(ground_registred == -1) {
      Serial.println(F("server registration failed!"));
      OLED_write("server reg failed!", 0, 18, true);
      OLED_write("please check json file!", 0, 27, false);
      snprintf (buf, sizeof(buf), "%s : %d", ogn_server.c_str(), ogn_port);
      OLED_write(buf, 0, 36, false);
      ground_registred = -2; 
      ExportTimeReRegister = seconds();
    }
  
    if(ground_registred == -2) {
      if (ExportTimeReRegister != 0)
          os_runstep();               // <<< this progresses the LMIC radio stuff?
      if (TimeToReRegisterOGN()) {
Serial.println("ground_registred == -2,  check wifi...");
          if (OGN_APRS_check_Wifi()) {
              ground_registred = 0;
              ExportTimeReRegister = 0;
          } else {
              Serial.println(F("REregistration: wifi error"));
              ExportTimeReRegister = seconds();  // try again 30 seconds from now
          }
      }
//      ExportTimeReRegister = seconds();      /* >>> doesn't this prevent next line? <<< */
//      while(TimeToReRegisterOGN()){          /* >>> is this safe? <<< */
//        os_runstep();
//      }
//      ground_registred = 0;
    }

    yield();

    if (TimeToExportOGN())
    {
      if (ground_registred == 1) {
        if(new_protocol_enable && testmode_enable){
          RSM_ExportAircraftPosition();
        }
        //Serial.println("Calling APRS_Export...");
        //disp = "Calling APRS_Export...";
        //OLED_write(disp, 0, 24, true);
        OGN_APRS_Export();
        //Serial.println("... APRS_Export returned");
      }
      if (! OLED_blank)
        OLED_info();
      ExportTimeOGN = seconds();
    }

    if (TimeToKeepAliveOGN() && ground_registred == 1)
    {
      //Serial.println("keepalive...");
      //disp = "keepalive OGN...";
      //OLED_write(disp, 0, 24, true);
      OGN_APRS_KeepAlive();
      //Serial.println("... APRS_KeepAlive returned");
      ExportTimeKeepAliveOGN = seconds();
    }
  
    if ((TimeToStatusOGN() || (ExportTimeStatusOGN==0 && millis()>50000))
        && ground_registred == 1 && position_is_set) {

      //Serial.println("status OGN...");

      static bool first_time = true;
      String resetReason;
      if (first_time)
          resetReason = SoC->getResetReason();

      if (OGN_APRS_Location(&ThisAircraft)
       && OGN_APRS_Status(&ThisAircraft, first_time, resetReason)) {

        first_time = false;

        //Serial.println("... status OGN returned");

        ExportTimeStatusOGN = seconds();
        disp = "status OGN...";
        OLED_write(disp, 0, 24, true);

        msg = "Version: ";
        msg += String(_VERSION);
        msg += " Power: ";
        msg += String(Battery_voltage());
        msg += String(" Uptime: ");
        msg += String(millis() / 3600000);
        msg += String(" GNSS: ");
        msg += String(gnss.satellites.value());
        Logger_send_udp(&msg);

      } else {   // transmission failed

        ExportTimeStatusOGN += 23;    // wait a bit before trying again

      }
    }

    if(TimeToCheckKeepAliveOGN() && ground_registred >= 1){
        Serial.println("check APRS msgs...");
        ground_registred = OGN_APRS_check_messages();
        //Serial.println("... APRS_check_messages returned");
        ExportTimeCheckKeepAliveOGN = seconds();
        MONIT_send_trap();
    }

    yield();

    static int retries = 0;
    if( TimeToCheckWifi() ) {
      //Serial.println("APRS check wifi...");
      OLED_draw_Bitmap(39, 5, 3 , true);
      OLED_write("check connections..", 15, 45, false);
      if(OGN_APRS_check_Wifi()){
        OLED_write("success", 35, 54, false);
        //Serial.println("... success");
        retries = 0;
      } else {
        // OGN_APRS_check_Wifi() did WiFi.disconnect() & WiFi.reconnect()
        DebugLogWrite("WiFi disconnected - trying reconnect...");
        OLED_write("error", 35, 54, false);
        if (retries < 4) {    // 4 * 133 seconds = 9 minutes
            ++retries;
            Serial.print(retries);
            Serial.println(F(" WiFi disconnected - trying..."));
            // WiFi_setup();    // not re-entrant, wifiMulti->run() causes watchdog crash?
        } else {
            DebugLogWrite("WiFi reconnections failed - reboot");
            Serial.println(F("WiFi reconnections failed - reboot"));
            OLED_write("reboot", 35, 54, false);
            delay(1000);
            SoC->reset();    // will run as AP for 10 minutes if WiFi network still not available
        }
      }
      ExportTimeCheckWifi = seconds();
    }

  }  // end if (only as basestation)

  yield();

  /* use export time marker for OLED display cycling in remote station */
  if (TimeToExportOGN() && ognrelay_enable) {
    if (! OLED_blank)
      OLED_info();
    ExportTimeOGN = seconds();
  }

//Serial.println("sleep check...");

  if (ogn_sleepmode > 0)
      sleep_check();

  if(fanet_enable && ground_registred == 1 && TimeToExportFanetService()) {

//Serial.println("FANET check...");

    OLED_draw_Bitmap(14, 0, 2 , true);
      
    if( fanet_transmitter ){
      RSM_receiver();
    }
    else{
      fanet_transmitter = RSM_Setup(ogn_debugport+1);
    }
    ExportTimeFanetService = seconds();
    msg = "current system time  ";
    msg += String(now());
    Logger_send_udp(&msg);
  }

  if (isTimeToUpdateTraffic()) {
    ClearExpired();
    UpdateTrafficTimeMarker = millis();
  }

//Serial.println("disable things check...");

  if (TimeToDisWifi() && ognrelay_enable) {

    if (ExportTimeDisWifi == 0) {

      ExportTimeDisWifi = seconds();  // wait another 333 seconds after the first 333

    } else if (WiFi.getMode() == WIFI_AP) {

#ifdef TBEAM
      turn_LED_off();   /* turn off bright blue LED to save power and signal end of WiFi */
#endif
      //if (settings->nmea_p)
      //    StdOut.println(F("$PSRFS,WIFI_OFF"));
      Serial.println(F("[ino] shutting down WiFI..."));
      DebugLogWrite("ino shut down wifi");
      Web_fini();
      WiFi_fini();
      // on TTGO & T3S3 the green LED will be turned off in Time_loop()
      //OLED_disable();
    }
  }

  if ( (oled_disable > 0 && TimeToDisableOled()) || WiFi.getMode() == WIFI_OFF ) {
     OLED_disable(); 
  }

  yield();

//Serial.println("check button...");

#if defined(TBEAM)
  if (tbeam_button != SOC_UNUSED_PIN) {
    if (!digitalRead(tbeam_button)){   // (middle) button pressed
      int n = 10;
      while(!digitalRead(tbeam_button) && --n>0) {delay(100);}
//Serial.println("button pressed");
      OLED_enable();
      ExportTimeOledDisable = seconds();
    }
  }
#endif
}

void
sleep_check()
{
    static uint32_t check_when = 0;
    if (millis() < check_when)
        return;
    check_when = millis() + 29876;

    bool low_bat = false;
    static uint32_t low_bat_time = 0;
    static bool low_bat_sleep = false;
    float volts = Battery_voltage();
    if (ognrelay_enable)
        remote_voltage = volts;
    if (volts < 3.65 && volts > 2.0) {
        low_bat = true;
        if (low_bat_time == 0) {
            Serial.println(F("local battery voltage < 3.65"));
            low_bat_time = millis();
        } else if (millis() > low_bat_time + 30000) {
            low_bat_sleep = true;
        }
    }
    bool high_bat = false;
    if (volts > 3.95 || volts < 2.0)
        high_bat = true;

    static uint32_t last_synched = 0;
    if (ognrelay_time || ognreverse_time) {
        if (time_synched)
            last_synched = seconds();
        if (low_bat_sleep == false && last_synched == 0) {
            if (high_bat || ognrelay_base) {
                ExportTimeSleep = seconds();      // don't sleep until a while after time-sync
                return;
            }
        }
    }
    bool no_response_from_base = false;
    if (ognrelay_enable && time_synched==false && seconds() > last_synched + 30*60) {
        // did not manage to time sync in 30 minutes - assume base station is down
        no_response_from_base = true;
    }

    /* if clock is valid, tie wakeup time to clock time */
    int localtime = 0;
    int seconds_into_hour = 0;
    bool havetime = (ThisAircraft.timestamp > 1000000);
    if (ognrelay_time || ognreverse_time)
        havetime = havetime && time_synched;
    else
        havetime = havetime && NTP_synched;
    if (havetime) {
        localtime = ThisAircraft.hour + ogn_timezone;
        if (localtime > 23)  localtime -= 24;
        if (localtime <  0)  localtime += 24;
        seconds_into_hour = ThisAircraft.minute * 60;
    }

    static bool will_sleep = false;   // once set, stays set between calls

    if (ognrelay_base && (ognrelay_time || ognreverse_time) && ogn_sleepmode == 2) {

        /* base follows remote */
        if (remote_sleep_length > 0) {
            will_sleep = true;
            sleep_length = 60 * remote_sleep_length;
//Serial.printf("follow remote_sleep_length=%d\r\n", remote_sleep_length);
        }

    } else {
 
        /* sleep if no traffic */
        if (TimeToSleep())
            will_sleep = true;

        /* sleep if evening has come */
        if (havetime && localtime >= ogn_evening) {
Serial.printf("evening sleep: OurTime=%d hour=%d localtime=%d\r\n",
OurTime, ThisAircraft.hour, localtime);
            will_sleep = true;
        }
    }

    /* sleep if battery is low, and has been low */
    if (low_bat_sleep) {
        will_sleep = true;

    /* remote sleep if base station does not respond */
    } else if (ognrelay_enable && no_response_from_base) {
        will_sleep = true;
        sleep_length = 90*60;   // try to connect to base station for 30m of each 2h
    }

    if (! will_sleep) {
        sleep_length = 0;
        sleep_when = 0;
        return;
    }

    if (sleep_when == 0)
        sleep_when = millis();

    if (sleep_length == 0) {

        sleep_length = ogn_wakeuptimer;
        int sleephours = 0;

        if (low_bat)
           sleep_length = 4*3600;   // may be extended below to next morning

Serial.printf("provisional sleep_length=%d\r\n", sleep_length);

        /* if clock is valid, tie wakeup time to clock time */
        if (havetime) {

Serial.printf("localtime=%d, ThisAircraft.minute=%d, seconds_into_hour=%d\r\n",
localtime, ThisAircraft.minute, seconds_into_hour);

          if (ogn_wakeuptimer == 0      /* always sleep until next morning */
               || (localtime >= ogn_evening || (low_bat && localtime >= ogn_evening - 4))) {

            /* sleep until next morning */
            sleephours = ogn_morning - localtime;
            if (sleephours < 0)       /* not yet past midnight */
                sleephours += 24;
            if (low_bat && localtime >= ogn_evening - 2)
                sleephours += 2;      /* no time to charge much today, sleep late tomorrow */
            if (sleephours > 0)
                sleep_length = sleephours * 3600 - seconds_into_hour;

          } else {   /* round up, to top of hour or half-past */

            sleep_length += 1800;
            sleep_length -= (seconds_into_hour + sleep_length) % 1800;

          }

          if (sleep_length < 600)     /* shouldn't happen */
              sleep_length += 1800;

          /* try and arrange so both stations will wake up somewhat _before_ top
             of the hour thus will later both start sleep within the same half-hour */
          if (ogn_rxidle % 1800 == 0)
              sleep_length -= 180;

Serial.printf("sleephours=%d, sleep_length=%d\r\n", sleephours, sleep_length);
        }

    }

    /* delay sleep long enough to report intention to OGN */
    int min_delay = 2*APRS_STATUS_REC + 42;
    if (ognrelay_base && (ognrelay_time || ognreverse_time))
        min_delay *= 2;
    int sleep_delay = (millis() - sleep_when) / 1000;
    if (sleep_delay < min_delay)
        return;

    /* adjust sleep length for the delay */
    if (sleep_length > sleep_delay)
        sleep_length -= sleep_delay;

    if (sleep_length < 240) {
        /* don't bother to sleep less than 4 minutes */
        sleep_when = 0;
        sleep_length = 0;
        ExportTimeSleep = seconds();
        return;
    }

    /* split into hour-long naps since ESP32 has problems sleeping for much longer */
    uint32_t hourcounter = 0;
    if (sleep_length > 3839) {
        hourcounter = sleep_length / 3600;      /* hours */
        sleep_length -= 3600 * hourcounter;     /* remainder, fraction of an hour */
        if (sleep_length < 240) {
            --hourcounter;
            sleep_length += 3600;
        }
    }
Serial.printf("hourcounter=%d, sleep_length=%d\r\n", hourcounter, sleep_length);

    OLED_enable();

    OLED_write("SLEEP...", 0, 24, true);

    if (low_bat)
        DebugLogWrite("low_bat sleep");
    else
        DebugLogWrite("planned sleep");

#if defined(TBEAM)      
    turn_LED_off();
    if (ogn_gnsstime) {
        GNSS_sleep();
        delay(600);
        turn_GNSS_off();   // because GNSS_sleep() didn't always work
    }
#endif 

    String msg = "----> entering sleep mode for ";
    msg += String(hourcounter); 
    msg += " hours + ";
    msg += String(sleep_length); 
    msg += " seconds";
    Serial.println(msg.c_str());
    Logger_send_udp(&msg);

    delay(1000);
    OLED_disable();

    if(!ognrelay_enable)
      SoC->WiFi_disconnect_TCP();

    delay(1500);

    deep_sleep_counter = hourcounter;             /* stored in RTC memory */
    esp_sleep_enable_timer_wakeup(sleep_length*1000000ULL);
//    esp_sleep_enable_ext0_wakeup(GPIO_NUM_26,1);           // which button is this?
    esp_deep_sleep_start();
}
