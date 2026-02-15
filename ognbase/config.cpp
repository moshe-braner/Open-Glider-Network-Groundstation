/*
   CONFIG.cpp
   Copyright (C) 2020 Manuel Rösel

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

#include "SoftRF.h"
#include "EEPROM.h"
#include "global.h"
#include "config.h"
#include "OLED.h"
#include "Log.h"

#include <protocol.h>
#include <freqplan.h>

//START SD
#include <SD.h>
#include <FS.h>

// SD card
#define SD_SCK  14
#define SD_MISO 2
#define SD_MOSI 15
#define SD_CS   13
//END SD

#define ARDUINOJSON_USE_DOUBLE 0
#include <ArduinoJson.h>

int config_done = 0;

//wifi
String ogn_ssid[5];
String ogn_wpass[5];
int    ssid_index = 0;

//radio default values
uint8_t  ogn_band        = RF_BAND_EU;
uint8_t  ogn_protocol_1  = RF_PROTOCOL_LEGACY;
uint8_t  ogn_protocol_2  = RF_PROTOCOL_OGNTP;
bool     ogn_bec         = true;   // bit error correction
uint8_t  noise_sampling  = 2;       // 2=collect background RSSI *not* just between time slots

//aprs default values
String   ogn_callsign    = "callsign";
String   ogn_server      = "aprs.glidernet.org";
uint16_t ogn_port        = 14580;
bool     ogn_debug       = false;
uint16_t ogn_debugport   = 12000;
bool     ogn_itrackbit   = false;
bool     ogn_istealthbit = false;
bool     ogn_mobile      = false;
uint16_t ogn_range       = 100;
bool     ogn_hiderelayed = true;

//sleep mode
int8_t   ogn_sleepmode   = 0;
int8_t   ogn_timezone    = 0;      // derived from longitude, not a user setting
int8_t   ogn_morning     = 10;     // standard time, not daylight savings
int8_t   ogn_evening     = 17;
uint16_t ogn_rxidle      = 3600;   // seconds
uint16_t ogn_wakeuptimer = 3600;

//position
float   ogn_lat              = 0;
float   ogn_lon              = 0;
int     ogn_alt              = 0;
int16_t ogn_geoid_separation = 0;

//fanet service
bool fanet_enable = false;

//zabbix
bool      zabbix_enable = false;
String    zabbix_server = "127.0.0.1";
uint16_t  zabbix_port   = 10051;
String    zabbix_key    = "ogn_base";

//supporters
bool beers_show = false;

//remote logs
bool      remotelogs_enable = false;
String    remotelogs_server = "127.0.0.1";
uint16_t  remotelogs_port = 12000;

//oled 
unsigned long  oled_disable = 0;

//test mode for new functions
bool  testmode_enable = false;

//private network
bool  private_network = false;

//new protocol   (internet protocol by Manuel, not FLARM protocol)
bool new_protocol_enable;
String new_protocol_server;
uint32_t new_protocol_port;

//relay
bool ognrelay_enable = false;    /* if true = remote station */
bool ognrelay_base = false;      /* if true = base station */
bool ognrelay_time = false;      /* if true = relay time from remote to base */
bool ognreverse_time = false;    /* if true = relay time from base to remote */
bool ogn_gnsstime = false;       /* if true = use GNSS time rather than NTP */
uint32_t ognrelay_key = 12345;    /* must be same in both stations for relay_time */

bool ognvoltage_limit = true;    /* if true then try and limit battery charge voltage to 4.0 */
    // at this point this is always true, not changeable via the config file

// time-relay (derived)
bool time_master = false;        /* if true = this station sends time */
bool time_client = false;        /* if true = this station receives time */


#if defined(TTGO) || defined (T3S3)

void performUpdate(Stream &updateSource, size_t updateSize) {

    char buf[32];

    snprintf(buf, sizeof(buf), "update firmware");
    OLED_write(buf, 0, 15, true);
  
   if (Update.begin(updateSize)) {      
      size_t written = Update.writeStream(updateSource);
      if (written == updateSize) {
         Serial.println("Written : " + String(written) + " successfully");
      }
      else {
        snprintf(buf, sizeof(buf), "update error");
        OLED_write(buf, 0, 16, true);         
        }
      if (Update.end()) {
         if (Update.isFinished()) {
            snprintf(buf, sizeof(buf), "update success");
            OLED_write(buf, 0, 24, false); 
            delay(1000);               
         }
         else {
            snprintf(buf, sizeof(buf), "update error");
            OLED_write(buf, 0, 16, true);               
         }
      }
      else {
         snprintf(buf, sizeof(buf), "update error");
         OLED_write(buf, 0, 16, true);         
      }

   }
   else
   {
    snprintf(buf, sizeof(buf), "update error");
    OLED_write(buf, 0, 16, true);   
   }
}

#endif

bool OGN_read_config(void)
{
    const size_t        capacity = 2752;
    DynamicJsonDocument baseConfig(capacity);
    JsonObject          obj;
    File configFile;

    const char *config_files[6] = { "/config.json", 
                                    "/index.html", 
                                    "/update.html", 
                                    "/style.css",
                                    "/key.bin",
                                    "/iv.bin"};    

    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        config_done = -5;
        return false;
    }

#if defined(TTGO) || defined (T3S3)
    /*READ SD Card - config.json & firmware update*/

    char buf[32];

#if defined(T3S3)
    SPI.begin(T3S3_SD_CLK, T3S3_SD_MISO, T3S3_SD_MOSI);
    if(!SD.begin(T3S3_SD_CS)){
      Serial.println("Card Mount Failed");
    }
#else
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI); // TTGO V2   >>> this failed with Core 2.0.2, works with Core 2.0.3
    if(!SD.begin(SD_CS)){                          // >>> 13 - examples on web have no argument passed here?
      Serial.println("Card Mount Failed");
    }
#endif
    else{
      int cardType = SD.cardType();
      uint64_t cardSize = SD.cardSize();
      cardSize >>= 20;
      Serial.printf("SD Card Type: %d = ", cardType);
      if(cardType == CARD_MMC){
        Serial.println("MMC");
      } else if(cardType == CARD_SD){
        Serial.println("SDSC");
      } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }
      Serial.printf("SD Card Size: %dMB\n", (uint32_t)cardSize);
  
      delay(500);

      for(size_t i=0;i<6;i++){
        File file = SD.open(config_files[i], FILE_READ);       
        File tempFile = SPIFFS.open(config_files[i], FILE_WRITE);
  
        size_t config_size = file.size();
        if (config_size > 0){
          Serial.print("update config from sd ");
          Serial.println(config_files[i]);
          snprintf(buf, sizeof(buf), "found config on sdcard");
          OLED_write(buf, 0, 16, true); 
          snprintf(buf, sizeof(buf), config_files[i]);
          OLED_write(buf, 0, 25, true);           
          snprintf(buf, sizeof(buf), "updating");
          OLED_write(buf, 0, 34, false);         
          while(file.available()){
            tempFile.write(file.read());
          }
        }
        delay(1000);
        tempFile.close();
        file.close();
      }
  
      //checking firmware file
      File updateBin = SD.open("/firmware/ognbase.bin");
      size_t updateSize = updateBin.size();
        
      if (updateSize > 0) {
        Serial.println("Try to start update");
        performUpdate(updateBin, updateSize);
        updateBin.close();
        SD.remove("/firmware/ognbase.bin");
        snprintf(buf, sizeof(buf), "reboot");
        OLED_write(buf, 0, 33, false);
        DebugLogWrite("SD firmware update reboot");
        delay(500);
        SoC->reset();         
      }
      else {
         Serial.println("No firmware found");
      }
    }
    SPI.end();
#endif    


    if (SPIFFS.exists(config_files[0])) {
      configFile = SPIFFS.open(config_files[0]);
      if (!configFile)
      {
          Serial.println(F("Failed to open config.json."));
          configFile = SPIFFS.open("/oldconf.json");
          if (!configFile)
          {
              config_done = -3;
              return false;
          }      
          Serial.println(F("Using oldconf.json instead."));
      }      
    } else {
        Serial.println(F("config.json doesnt exist, please upload config.json"));
        OLED_write("no config file", 0, 27, true);
        delay(1000);
        configFile = SPIFFS.open("/oldconf.json");
        if (!configFile)
        {
            config_done = -4;
            return(false);
        }
        Serial.println(F("Using oldconf.json instead."));
        OLED_write("using oldconf", 0, 27, true);
        delay(1000);
    }

    DeserializationError error = deserializeJson(baseConfig, configFile);

    if (error)
    {
        Serial.println(F("Failed to parse json file"));
        Serial.println(error.f_str());
        OLED_write("config file error", 0, 27, true);
        delay(1000);
        configFile.close();
        configFile = SPIFFS.open("/oldconf.json");
        if (!configFile)
        {
            Serial.println(F("using default configuration"));
            //OLED_write("default conf", 0, 27, true);
            //delay(1000);
            config_done = -2;
            return false;
        }
        Serial.println(F("Using oldconf.json instead."));
        OLED_write("using oldconf", 0, 27, true);
        delay(1000);
        error = deserializeJson(baseConfig, configFile);
        if (error) {
            Serial.println(F("Failed to parse oldconf file"));
            Serial.println(error.f_str());
            configFile.close();
            Serial.println(F("using default configuration"));
            OLED_write("default conf", 0, 27, true);
            delay(1000);
            config_done = -2;
            return false;
        }
    }

    obj = baseConfig.as<JsonObject>();
    configFile.close();

    if (!obj.containsKey(F("ognbase"))){
        Serial.println("config.json not valid - version missing");
        OLED_write("config.json - no version", 0, 27, true);
        delay(1000);
        config_done = -1;
        return false;
    } else {
        String jsonversion = "wrong";
        jsonversion = obj["ognbase"]["version"].as<String>();
        if (jsonversion != OGNBASE_HTML_VERSION) {
            Serial.println("Warning: wrong version of config.json");
            OLED_write("config version wrong", 0, 27, true);
            delay(1000);
            config_done = -1;
            // but keep going, so as to not lose wifi contact
        }
    }

    if (!obj.containsKey(F("wifi"))){
        //Serial.println("no wifi configuration found, return setup mode");
        // configFile.close();        
        config_done = 0;
        return false;
    }
    else
    {
        //Serial.println(F("found wifi config!"));
        if (1)
            for (int i=0; i < 5; i++) {
                ogn_ssid[i]  = obj["wifi"]["ssid"][i].as<String>();
                ogn_wpass[i] = obj["wifi"]["pass"][i].as<String>();
            }
    }

    if (obj.containsKey(F("coordinates")))
    {
        //Serial.println(F("found coordinates config!"));
        if (1)
        {
            ogn_lat              = obj["coordinates"]["lat"];
            ogn_lon              = obj["coordinates"]["lon"];
            ogn_alt              = obj["coordinates"]["alt"];
            ogn_geoid_separation = obj["coordinates"]["geoidsep"];
            ogn_mobile           = obj["coordinates"]["mobile"];
        }
    }


    if (obj.containsKey(F("radio")))
    {
        //Serial.println(F("found radio config!"));
        if (1)
        {
            ogn_band        = obj["radio"]["band"];
            if (ogn_band < 1 || ogn_band > 10)    /* override invalid & AUTO with EU */
                ogn_band = 1;
//          ogn_protocol_1  = obj["radio"]["protocol_1"];
ogn_protocol_1  = RF_PROTOCOL_LEGACY;  /* override - only protocol supported for now */
//          ogn_protocol_2  = obj["radio"]["protocol_2"];
ogn_protocol_2  = RF_PROTOCOL_OGNTP;
            ogn_bec         = obj["radio"]["bec"];    // true = enable bit error correction
            noise_sampling  = obj["radio"]["noise"];
            // 0 = skip background RSSI data sampling
            // 1 = only sample in the idle time between slot 1 & slot 0
            // 2 = sample at any time
            //Serial.print(F("noise_sampling="));
            //Serial.println(noise_sampling);
        }
    }

    if (obj.containsKey(F("aprs")))
    {
        //Serial.println(F("found aprs config!"));
        if (1)
        {
            ogn_callsign = obj["aprs"]["callsign"].as<String>();
            ogn_callsign.trim();
            ogn_callsign.replace("_","");
            if(ogn_callsign.length() > 9){
              ogn_callsign = ogn_callsign.substring(0,9);
            }
                        
            ogn_server   = obj["aprs"]["server"].as<String>();
            ogn_port     = obj["aprs"]["port"];

            ogn_debug       = obj["aprs"]["debug"];
            ogn_debugport   = obj["aprs"]["debugport"];
            ogn_itrackbit   = obj["aprs"]["itrackbit"];
            ogn_istealthbit = obj["aprs"]["istealthbit"];
            ogn_range       = obj["aprs"]["range"];
            ogn_hiderelayed = obj["aprs"]["hiderelayed"];
        }
    }

    if (obj.containsKey(F("sleep")))
    {
        //Serial.println(F("found sleep config!"));
        if (1)
        {
            ogn_sleepmode   = obj["sleep"]["mode"];
            ogn_morning     = obj["sleep"]["morning"];
            ogn_evening     = obj["sleep"]["evening"];
            ogn_rxidle      = obj["sleep"]["rxidle"];   // minutes
            ogn_rxidle *= 60;                           // seconds
            if(ogn_rxidle != 0 && ogn_rxidle < 600) {ogn_rxidle = 600;}
            ogn_wakeuptimer = obj["sleep"]["wakeuptimer"];
            ogn_wakeuptimer *= 60;
            if(ogn_wakeuptimer != 0 && ogn_wakeuptimer < 600) {ogn_wakeuptimer = 600;}
        }
    }    


    if (obj.containsKey(F("zabbix")))
    {
        //Serial.println(F("found zabbix config!"));
        if (1)
        {
            zabbix_enable = obj["zabbix"]["enable"];
            zabbix_server = obj["zabbix"]["server"].as<String>();
            ;
            zabbix_port = obj["zabbix"]["port"];
            zabbix_key  = obj["zabbix"]["key"].as<String>();
            ;
        }
    }

    if (obj.containsKey(F("remotelogs")))
    {
        //Serial.println(F("found remotelogs config!"));
        if (1)
        {
            remotelogs_enable = obj["remotelogs"]["enable"];
            remotelogs_server = obj["remotelogs"]["server"].as<String>();
            remotelogs_port = obj["remotelogs"]["port"];
        }
    }

    if (obj.containsKey(F("testmode")))
    {
        //Serial.println(F("found testmode config!"));
        if (1)
        {
            testmode_enable = obj["testmode"]["enable"];
        }
    }

    if (obj.containsKey(F("ognrelay")))
    {
        //Serial.println(F("found relay config!"));
        ognrelay_base = obj["ognrelay"]["basestation"];
        if (ognrelay_base)
            ognrelay_enable = false;                          /* "base" overrides */
        else
            ognrelay_enable = obj["ognrelay"]["enable"];
        if (ognrelay_enable || ognrelay_base)
            ognrelay_time = obj["ognrelay"]["relaytime"];
        else
            ognrelay_time = false;
        ognreverse_time = obj["ognrelay"]["reversetime"];
        if (ognreverse_time)
            ognrelay_time = false;
        ognrelay_key = obj["aprs"]["relaykey"];
        // String key_str = obj["aprs"]["relaykey"].as<String>();
        // ognrelay_key = (uint32_t) std::stoi(key_str,nullptr,16);

        // ognvoltage_limit = obj["ognrelay"]["voltlimit"];

#if defined(TBEAM)
        if (ognrelay_base && ognrelay_time)                     /* gets time from remote */
            ogn_gnsstime = false;
        else if (ognrelay_enable && ognrelay_time)              /* sends time to base */
            ogn_gnsstime = true;
        else if (ognrelay_enable && ognreverse_time)            /* gets time from base */
            ogn_gnsstime = false;
//      else if (ogn_band==RF_BAND_AU || ogn_band==RF_BAND_US)  /* if more than 2 channels */
//          ogn_gnsstime = true;                                /* must use GNSS time - no longer! */
        else
            ogn_gnsstime = obj["ognrelay"]["gnsstime"];
        if (ognrelay_enable || ognrelay_base || (! ogn_gnsstime))
            ogn_mobile = false;
#else
        ogn_gnsstime = false;                                /* no GNSS hardware */
        ogn_mobile = false;
        /*
        if (ogn_band==RF_BAND_AU || ogn_band==RF_BAND_US) {  // more than 2 channels
            ognrelay_enable = false;
            if(ognrelay_base)
                ognrelay_time = true;
        }
        */
#endif
    }

    if (ogn_sleepmode==2 && (! ognrelay_base || (! ognrelay_time && ! ognreverse_time)))
        ogn_sleepmode = 0;

    if (obj.containsKey(F("fanetservice")))
    {
        //Serial.println(F("found fanetservice config!"));
        if (1)
            fanet_enable = obj["fanetservice"]["enable"];
    }

    /*oled_disable*/
    if (obj.containsKey(F("oled")))
    {
        //Serial.println(F("found oled config!"));
        if (1)
            oled_disable = obj["oled"]["disable"];
    }    
    
    /*private network*/
    if (obj.containsKey(F("private")))
    {
        //Serial.println(F("found private network config!"));
        if (1)
            private_network = obj["private"]["encrypt"];
        if (1) {
            webserver_port = obj["private"]["webport"];
            // should be in the range 49152 to 65535
            //   "not assigned, controlled, or registered, used for private ports"
            if (webserver_port < 49152 || webserver_port > 65535) {
                if (private_network)
                    webserver_port = 443;   // <<< is this necessary?
                else
                    webserver_port = 80;    // this is the default if no "private" JSON key
            }
        }
    }

    /*new binary prot*/
    if (obj.containsKey(F("newprot")))
    {
        if (1)
            new_protocol_enable = obj["newprot"]["enable"];
            new_protocol_server = obj["newprot"]["server"].as<String>();
            new_protocol_port = obj["newprot"]["port"];
    }        

    if (obj.containsKey(F("beers")))
        beers_show = obj["beers"]["show"];

    time_master = (ognrelay_enable && ognrelay_time) || (ognrelay_base && ognreverse_time);
    time_client = (ognrelay_base && ognrelay_time) || (ognrelay_enable && ognreverse_time);

if (time_master)
Serial.println("Time master!");
if (time_client)
Serial.println("Time client!");
if (time_master && time_client)
Serial.println(">>>>> both Time client and Time Master! <<<<<");

    if (config_done == 0)
        config_done = 1;
    return true;
}

bool OGN_save_config(void)
{
    const size_t        capacity = 2752;
    DynamicJsonDocument baseConfig(capacity);
    JsonObject          obj;
    char buf[32];

    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return false;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile)
    {
        Serial.println(F("Failed to open config.json readonly"));
        return false;
    }

    DeserializationError ds_error = deserializeJson(baseConfig, configFile);
    configFile.close();
    obj = baseConfig.as<JsonObject>();

    bool error = false;
    if (ds_error) {
        Serial.println(F("Failed to parse config.json file"));
        error = true;
    } else {
        if (!obj.containsKey(F("ognbase"))) {
          Serial.println("config.json not valid - version missing");
          error = true;
        } else {
          String jsonversion = "wrong";
          jsonversion = obj["ognbase"]["version"].as<String>();
          if (jsonversion != OGNBASE_HTML_VERSION) {
            Serial.println("Wrong version of config.json");
            error = true;
          }
        }
    }

    if (error) {
        Serial.println(F("- creating new configuration"));
        /* try and write out a new config file */
        obj["ognbase"]["version"] = OGNBASE_HTML_VERSION;
    }

    // keep a copy of the existing config file if it exists
    if (SPIFFS.exists("/config.json")) {
        if (SPIFFS.exists("/oldconf.json"))
            SPIFFS.remove("/oldconf.json");
        SPIFFS.rename("/config.json", "/oldconf.json");
    }

    configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
        Serial.println(F("Failed to open config.json for writing"));
        return false;
    }

    //position config
/*
    float f = ogn_lat * 1024*1024;
    int32_t i = (int) f;
    f = (float) i;
    f /= 1024*1024;
    snprintf(buf, sizeof(buf), "%.6f", f);
    obj["coordinates"]["lat"] = buf;
    f = ogn_lon * 1024*1024;
    i = (int) f;
    f = (float) i;
    f /= 1024*1024;
    snprintf(buf, sizeof(buf), "%.6f", f);
    obj["coordinates"]["lon"] = buf;
*/
    obj["coordinates"]["lat"]      = ogn_lat;
    obj["coordinates"]["lon"]      = ogn_lon;
    obj["coordinates"]["alt"]      = ogn_alt;
    obj["coordinates"]["geoidsep"] = ogn_geoid_separation;
    obj["coordinates"]["mobile"]   = ogn_mobile;

    //radio config
    obj["radio"]["band"]       = ogn_band;
    obj["radio"]["protocol_1"] = ogn_protocol_1;
    obj["radio"]["protocol_2"] = ogn_protocol_2;
    obj["radio"]["bec"]        = ogn_bec;
    obj["radio"]["noise"]      = noise_sampling;

    //aprs config
    obj["aprs"]["callsign"] = ogn_callsign;
    obj["aprs"]["server"]   = ogn_server;
    obj["aprs"]["port"]     = ogn_port;
    obj["aprs"]["debug"]       = ogn_debug;
    obj["aprs"]["debugport"]   = ogn_debugport;
    //obj["aprs"]["itrackbit"]   = ogn_itrackbit;
    //obj["aprs"]["istealthbit"] = ogn_istealthbit;
    obj["aprs"]["range"]       = ogn_range;
    obj["aprs"]["hiderelayed"] = ogn_hiderelayed;

    //sleep config
    obj["sleep"]["mode"]        = ogn_sleepmode;
    obj["sleep"]["morning"]     = ogn_morning;
    obj["sleep"]["evening"]     = ogn_evening;
    obj["sleep"]["rxidle"]      = ogn_rxidle / 60;
    obj["sleep"]["wakeuptimer"] = ogn_wakeuptimer / 60;

    //wifi config
    obj["wifi"]["ssid"][0] = ogn_ssid[0];
    obj["wifi"]["pass"][0] = ogn_wpass[0];
    // save 2 additional entries not shown in web UI
    if (ogn_ssid[1].length() > 0) {
        obj["wifi"]["ssid"][1] = ogn_ssid[1];
        obj["wifi"]["pass"][1] = ogn_wpass[1];
    }
    if (ogn_ssid[2].length() > 0) {
        obj["wifi"]["ssid"][2] = ogn_ssid[2];
        obj["wifi"]["pass"][2] = ogn_wpass[2];
    }
    // 2 more may be in original config but won't be saved here

    //fanet config
    obj["fanetservice"]["enable"] = (int) fanet_enable;

    //zabbix config
    obj["zabbix"]["enable"] = zabbix_enable;
    obj["zabbix"]["server"] = zabbix_server;
    obj["zabbix"]["port"]   = zabbix_port;
    obj["zabbix"]["key"]    = zabbix_key;

    obj["testmode"]["enable"] = testmode_enable;

    obj["private"]["webport"] = webserver_port;

    obj["oled"]["disable"] = (int) oled_disable;

    // relay config
    obj["ognrelay"]["enable"]      = (int) ognrelay_enable;
    obj["ognrelay"]["basestation"] = (int) ognrelay_base;
    obj["ognrelay"]["relaytime"]   = (int) ognrelay_time;
    obj["ognrelay"]["reversetime"] = (int) ognreverse_time;
    obj["ognrelay"]["gnsstime"]    = (int) ogn_gnsstime;
    obj["ognrelay"]["relaykey"]    = ognrelay_key;
    // obj["ognrelay"]["voltlimit"]   = (int) ognvoltage_limit;

    // snprintf(buf, sizeof(buf), "%06X", ognrelay_key);
    // buf[8] = '\0';
    // char *key_cstr = &buf[2];   /* skip the "0x" */
    // String key_str = key_cstr;
    // obj["ognrelay"]["relaykey"]    = key_str;

    // >>> added Pretty to try and get line breaks:
    if (serializeJsonPretty(obj, configFile) == 0)
        Serial.println(F("Failed to write to file"));

    configFile.close();
    return true;
}
