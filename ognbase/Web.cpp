/*
 * Web.cpp
 * Copyright (C) 2020 Manuel Rösel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "SoC.h"
#include "OLED.h"
#include "EEPROM.h"
#include "Time.h"
#include "RF.h"
#include "global.h"
#include "Battery.h"
#include "Log.h"
#include "config.h"
#include "Traffic.h"
#include <ArduinoJson.h>

#include <ErriezCRC32.h>

#include <FS.h>   // Include the SPIFFS library

#define  U_PART U_FLASH
#define INDEX_CRC 3948812196

extern unsigned long ExportTimeWebRefresh;

File fsUploadFile;

#define countelems(a) (sizeof(a) / sizeof(a[0]))

// Create AsyncWebServer object on port 80
AsyncWebServer wserver(80);
AsyncWebSocket ws("/ws");

AsyncWebSocketClient* globalClient = NULL;

size_t content_len;

static const char upload_html[] PROGMEM =
"<html>\
 <head>\
 <meta http-equiv='Content-Type' content='text/html; charset=utf-8'>\
 </head>\
 <div class = 'upload'>\
 <form method = 'POST' action = '/doUpload' enctype='multipart/form-data'>\
 <input type='file' name='data'/><input type='submit' name='upload' value='Upload' title = 'Upload Files'>\
 </form></div>\
 </html>";

const char *ognopmode = "OGNbase";

static const char stats_templ[] PROGMEM =
"<html>\
 <head>\
 <meta http-equiv='Content-Type' content='text/html; charset=utf-8'>\
 <meta http-equiv='Refresh' content='23'>\
 </head>\
 <p>\r\nOperation mode: %s</p>\
 <br><p>\r\nBase Station Stats:</p>\
 <p>&nbsp;\r\n Traffic packets received: %d</p>\
 <p>&nbsp;\r\n Traffic packets reported: %d</p>\
 <p>&nbsp;\r\n Aircraft seen ever: %d, today: %d</p>\
 <p>&nbsp;\r\n Largest Range: %d km</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <p>&nbsp;\r\n Time-sync restarts: %d</p>\
 <br><p>\r\nRemote Station Stats:</p>\
 <p>&nbsp;\r\n Traffic packets: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  per minute: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct relayed: %d</p>\
 <p>&nbsp;\r\n Time packets sent: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct acknowledged: %d</p>\
 <p>&nbsp;\r\n Time-sync restarts: %d</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <p>&nbsp;\r\n Battery voltage: %.1f</p>\
 </html>";
                                  
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if (type == WS_EVT_CONNECT)

        globalClient = client;

    else if (type == WS_EVT_DISCONNECT)

        globalClient = NULL;
}

// Replaces placeholder with LED state value
String processor(const String& var)
{
    Serial.println(var);
    return "ON";
}

void Web_fini()
{}

void handleUpdate(AsyncWebServerRequest* request)
{
    char* html = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
    request->send(200, "text/html", html);
}

void handleUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final)
{
    if (!index)
        request->_tempFile = SPIFFS.open("/" + filename, "w");
    if (len)
        // stream the incoming chunk to the opened file
        request->_tempFile.write(data, len);
    if (final)
    {
        request->_tempFile.close();
        request->redirect("/");
    }
}

void handleDoUpdate(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final)
{
    String msg;

    if (!index)
    {
        msg = "updating firmware";
        Logger_send_udp(&msg);

       //SPIFFS.format();
        
        content_len = request->contentLength();
        // if filename includes spiffs, update the spiffs partition
        int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;

        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
        {

            Update.printError(Serial);
        }
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
    }

    if (final)
    {
        AsyncWebServerResponse* response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
        response->addHeader("Refresh", "20");
        response->addHeader("Location", "/");
        request->send(response);
        if (!Update.end(true))
            Update.printError(Serial);
        else
        {
            delay(10000);
            ESP.restart();
        }
    }
}

#define STATS_SIZE 1279
char stats_html[STATS_SIZE+1];

void update_stats()
{
  uint32_t zero = 0;
  if (ognrelay_enable && ! ogn_gnsstime && ! ognrelay_time) {
     if (have_approx_time > 0)
        ognopmode = "Remote station, NTP time from base";
     else
        ognopmode = "Remote station, no precise time";
  }
  if (ognrelay_base && ognrelay_time) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        traffic_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        (uint32_t) sync_restarts,
        remote_traffic,
        (uint32_t) packets_per_minute,
        (uint32_t) remote_pctrel,
        (uint32_t) remote_timesent,
        (uint32_t) remote_ack,
        (uint32_t) remote_restarts,
        (uint32_t) remote_bad,
        (uint32_t) remote_other,
        remote_voltage);
  } else if (ognrelay_base) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        traffic_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        zero,
        remote_traffic,
        (uint32_t) packets_per_minute,
        (uint32_t) remote_pctrel,
        zero, zero, zero, zero, zero,
        (float) 0.0);
  } else if (ognrelay_enable) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        zero, zero, zero, zero,
        zero, zero, zero, zero,
        traffic_packets_recvd,
        (uint32_t) packets_per_minute,
        (uint32_t) ((100 * traffic_packets_relayed)
                      / (traffic_packets_recvd+1)),
        (uint32_t) time_packets_sent,
        ((uint32_t) (100 * (ack_packets_recvd+1)) / (time_packets_sent+1)),
        (uint32_t) sync_restarts,
        (uint32_t) bad_packets_recvd,
        (uint32_t) other_packets_recvd,
        (float) Battery_voltage());
  } else {  /* single standalone station */
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        traffic_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        zero, zero,
        (uint32_t) packets_per_minute,
        zero, zero, zero, zero, zero, zero,
        (float) Battery_voltage());
  }
  stats_html[STATS_SIZE] = '\0';
//Serial.println(stats_html);
}

void Web_start()
{
    wserver.begin();
}

void Web_stop()
{
    wserver.end();
}

void Web_setup(ufo_t* this_aircraft)
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    if (!SPIFFS.exists("/index.html"))
    {
        // wserver.on("/", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
        wserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request){
            request->send(200, "text/html", upload_html);
        });

        wserver.on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, handleUpload);

        Web_start();
        return;
    }

    File file = SPIFFS.open("/index.html", "r");
    if (!file)
    {
        Serial.println("An Error has occurred while opening index.html");
        return;
    }

    ws.onEvent(onWsEvent);
    wserver.addHandler(&ws);


    size_t filesize   = file.size();
    char*  index_html = (char *) malloc(filesize + 1);

    file.read((uint8_t *)index_html, filesize);
    index_html[filesize] = '\0';

    file.close();

    char *cp = strstr(index_html, "meta name=\"OGNbase-Version\"");
    if (cp != NULL) {
        cp += 37;
        char versionbuf[8];
        int ii = 0;
        while (*cp != '\"' && *cp != '\0' && ii < 7)
            versionbuf[ii++] = *cp++;
        versionbuf[ii] = '\0';
        if (strcmp(versionbuf,OGNBASE_HTML_VERSION)) {
            Serial.println("Wrong version of index.html");
            OLED_write("Wrong index.html version", 0, 27, true);
            delay(500);
            wserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request){
                request->send(200, "text/html", upload_html);
            });
            wserver.on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, handleUpload);
            Web_start();
            return;
        }
    }

    char*  offset;
    size_t size = 9600;
    char*  Settings_temp = (char *) malloc(size);

    if (Settings_temp == NULL)
        return;

    offset = Settings_temp;

    String station_addr = String(this_aircraft->addr, HEX);
    station_addr.toUpperCase();

    /*Bugfix Issue 20*/
    String pos_lat = "";
    String pos_lon = "";    
    String pos_alt = "";
    String pos_geo = "";

    pos_lat.concat(ogn_lat);
    pos_lon.concat(ogn_lon);
    pos_alt.concat(ogn_alt);
    pos_geo.concat(ogn_geoid_separation);
    /*END  Bugfix*/ 

    String version = SOFTRF_FIRMWARE_VERSION;
    // version += (ognrelay_enable? " - remote" : (ognrelay_base? " - base" : " - standalone"));

    if (ognrelay_base && ognrelay_time)
        ognopmode = "Base station, time relayed from remote";
    else if (ognrelay_base && ogn_gnsstime)
        ognopmode = "Base station, time from GNSS";
    else if (ognrelay_base)
        ognopmode = "Base station, time from NTP";
    else if (ognrelay_enable && ognrelay_time && ogn_gnsstime)
        ognopmode = "Remote station, relaying GNSS time";
    else if (ognrelay_enable && ogn_gnsstime)
        ognopmode = "Remote station, time from GNSS"; 
    else if (ognrelay_enable)
        ognopmode = "Remote station, no precise time";
    else if (ogn_gnsstime)
        ognopmode = "Single station, time from GNSS";
    else
        ognopmode = "Single station, time from NTP";

    snprintf(offset, size, index_html,
             station_addr,
             version.c_str(),
             ognopmode,
             ogn_callsign,
             pos_lat,
             pos_lon,
             pos_alt,
             pos_geo,
             String(ogn_range),
             // (ogn_band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
             (ogn_band == RF_BAND_EU ? "selected" : ""), RF_BAND_EU,
             (ogn_band == RF_BAND_US ? "selected" : ""), RF_BAND_US,
             (ogn_band == RF_BAND_AU ? "selected" : ""), RF_BAND_AU,
             (ogn_band == RF_BAND_NZ ? "selected" : ""), RF_BAND_NZ,
             (ogn_band == RF_BAND_RU ? "selected" : ""), RF_BAND_RU,
             (ogn_band == RF_BAND_CN ? "selected" : ""), RF_BAND_CN,
             (ogn_band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK,
             (ogn_band == RF_BAND_IN ? "selected" : ""), RF_BAND_IN,
             (ogn_band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK, // should be IL
             (ogn_band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK, // should be KR
             (ogn_protocol_1 == RF_PROTOCOL_LEGACY ? "selected" : ""),
             RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_OGNTP ? "selected" : ""),
             RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_P3I ? "selected" : ""),
             RF_PROTOCOL_P3I, p3i_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_FANET ? "selected" : ""),
             RF_PROTOCOL_FANET, fanet_proto_desc.name,

             (ogn_protocol_2 == RF_PROTOCOL_LEGACY ? "selected" : ""),
             RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_OGNTP ? "selected" : ""),
             RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_P3I ? "selected" : ""),
             RF_PROTOCOL_P3I, p3i_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_FANET ? "selected" : ""),
             RF_PROTOCOL_FANET, fanet_proto_desc.name,

             (ogn_debug == true ? "selected" : ""),
             (ogn_debug == false ? "selected" : ""),
             String(ogn_debugport),

             (ogn_itrackbit == true ? "selected" : ""), "True",
             (ogn_itrackbit == false ? "selected" : ""), "False",

             (ogn_istealthbit == true ? "selected" : ""), "True",
             (ogn_istealthbit == false ? "selected" : ""), "False",

             ogn_ssid[0].c_str(),
             
             /*Hide Wifi Password*/
             "hidepass",

             (ogn_sleepmode == 0 ? "selected" : ""), "Disabled",
             (ogn_sleepmode == 1 ? "selected" : ""), "Full",
             (ogn_sleepmode == 2 ? "selected" : ""), "without GPS", //zabbix_trap_en

             String(ogn_rxidle),
             String(ogn_wakeuptimer),

             (zabbix_enable == 0 ? "selected" : ""), "Disabled",
             (zabbix_enable == 1 ? "selected" : ""), "Enabled"
             );
            
    size_t len  = strlen(offset);
    String html = String(offset);

    wserver.on("/", HTTP_GET, [html](AsyncWebServerRequest* request){
        request->send(200, "text/html", html);
    });

    // Route to load style.css file
    wserver.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    wserver.on("/update", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpdate(request);
        request->redirect("/");
    });

    wserver.on("/doUpdate", HTTP_POST,
               [](AsyncWebServerRequest* request) {},
               [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data,
                  size_t len, bool final) {
        handleDoUpdate(request, filename, index, data, len, final);
    });

    // wserver.on("/upload", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
    wserver.on("/upload", HTTP_GET, [](AsyncWebServerRequest* request){
        request->send(200, "text/html", upload_html);
    });

    snprintf(stats_html, STATS_SIZE, "stats not available yet");

    wserver.on("/stats", HTTP_GET, [](AsyncWebServerRequest* request){
        update_stats();
        Serial.println(F("requesting stats page..."));
        request->send(200, "text/html", String(stats_html));
    });

    wserver.on("/refresh", HTTP_GET, [](AsyncWebServerRequest* request){
        /* refresh the stats at the top of the status page in 2 sec */
        ExportTimeWebRefresh = millis()/1000 - 21;
        request->redirect("/");
    });    

    wserver.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        request->redirect("/");
        SoC->reset();
    });    

    wserver.on("/remote_reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        if (reboot_remote()) {
          Serial.println(F("sent remote reboot packet..."));
          OLED_write("remote reboot...", 0, 27, true);
          delay(1000);
          // should display something on the web page too.
          // for now just some hints:
          remote_sats = 0;            /* hints that remote is restarting */
          remote_uptime = 0;
          uptime = 0;
          ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top of the status page */
        }
        request->redirect("/");
    });    

    wserver.on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, handleUpload);

    wserver.on("/clear", HTTP_GET, [](AsyncWebServerRequest* request){
        Serial.println(F("Formatting spiffs..."));
        SPIFFS.format();
        request->redirect("/");
    });    

    // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
    wserver.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("callsign"))
            ogn_callsign = request->getParam("callsign")->value().c_str();
            ogn_callsign.trim();
            ogn_callsign.replace("_","");
            if(ogn_callsign.length() > 9){
              ogn_callsign = ogn_callsign.substring(0,9);
            }
        if (request->hasParam("ogn_lat"))
            ogn_lat = request->getParam("ogn_lat")->value().toFloat();

        if (request->hasParam("ogn_lon"))
            ogn_lon = request->getParam("ogn_lon")->value().toFloat();

        if (request->hasParam("ogn_alt"))
            ogn_alt = request->getParam("ogn_alt")->value().toInt();

        if (request->hasParam("ogn_freq"))
            ogn_band = request->getParam("ogn_freq")->value().toInt();

        if (request->hasParam("ogn_proto"))
            ogn_protocol_1 = request->getParam("ogn_proto")->value().toInt();

        ogn_protocol_1 = RF_PROTOCOL_LEGACY;    /* override, since only LEGACY supported for now */

        if (request->hasParam("ogn_proto2"))
            ogn_protocol_2 = request->getParam("ogn_proto2")->value().toInt();

        if (request->hasParam("ogn_d1090"))
            settings->d1090 = request->getParam("ogn_d1090")->value().toInt();

        if (request->hasParam("ogn_gdl90"))
            settings->gdl90 = request->getParam("ogn_gdl90")->value().toInt();

        if (request->hasParam("ogn_nmea"))
            settings->nmea_out = request->getParam("ogn_nmea")->value().toInt();

        if (request->hasParam("ogn_no_track_bit"))
            settings->no_track = request->getParam("ogn_no_track_bit")->value().toInt();

        if (request->hasParam("ogn_stealth_bit"))
            settings->stealth = request->getParam("ogn_stealth_bit")->value().toInt();

        if (request->hasParam("ogn_aprs_debug"))
            ogn_debug= request->getParam("ogn_aprs_debug")->value().toInt();

        if (request->hasParam("aprs_debug_port"))
            ogn_debugport = request->getParam("aprs_debug_port")->value().toInt();

        if (request->hasParam("ogn_range"))
            ogn_range = request->getParam("ogn_range")->value().toInt();

        //if (request->hasParam("ogn_agc"))
        //    settings->sxlna = request->getParam("ogn_agc")->value().toInt();

        if (request->hasParam("ogn_ssid"))
            ogn_ssid[0] = request->getParam("ogn_ssid")->value().c_str();

        if (request->hasParam("ogn_wifi_password")){
            if (request->getParam("ogn_wifi_password")->value() != "hidepass"){
              ogn_wpass[0] = request->getParam("ogn_wifi_password")->value().c_str();
            }
        }
            
        //geoid_separation
        if (request->hasParam("ogn_geoid"))
            ogn_geoid_separation = request->getParam("ogn_geoid")->value().toInt();

        if (request->hasParam("ogn_ignore_track"))
            ogn_itrackbit= request->getParam("ogn_ignore_track")->value().toInt();

        if (request->hasParam("ogn_ignore_stealth"))
            ogn_istealthbit= request->getParam("ogn_ignore_stealth")->value().toInt();

        if (request->hasParam("ogn_deep_sleep"))
            ogn_sleepmode = request->getParam("ogn_deep_sleep")->value().toInt();

        if (request->hasParam("ogn_sleep_time"))
        {
            if (request->getParam("ogn_sleep_time")->value().toInt() <= 60)
                ogn_rxidle = 60;
            else
                ogn_rxidle = request->getParam("ogn_sleep_time")->value().toInt();
        }
        if (request->hasParam("ogn_wakeup_time"))
            ogn_wakeuptimer= request->getParam("ogn_wakeup_time")->value().toInt();

        if (request->hasParam("zabbix_trap_en"))
            zabbix_enable = request->getParam("zabbix_trap_en")->value().toInt();


        request->redirect("/");
#if 0
        // ogn_reset_all
        if (request->hasParam("ogn_reset_all"))
            if (request->getParam("ogn_reset_all")->value() == "on")
            {
                SPIFFS.format();
                RF_Shutdown();
                delay(200);
                SoC->reset();
            }
#endif
        EEPROM_store();
        OGN_save_config();
        RF_Shutdown();
        delay(200);
        SoC->reset();
    });

    SoC->swSer_enableRx(true);
    free(Settings_temp);
    free(index_html);

    // Start server
    Web_start();
    delay(500);
    ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top on next web_loop() */
}

void Web_loop(void)
{
    if (globalClient != NULL && globalClient->status() == WS_CONNECTED)
    {
        String values;

#if defined(TBEAM)
        values = (ognrelay_base && ognrelay_time) ? remote_sats : gnss.satellites.value();
#else
        values = remote_sats;   // was: power
#endif
        values += "_";
        values += numseen_1hr;     // was: RF_last_rssi
        values += "_";
        values += (ognrelay_base && ognrelay_time) ? remote_uptime : uptime;
        values += "_";
        values += traffic_packets_recvd;    // was satfix
        values += "_";
        values += packets_per_minute;   // was: timestamp
        values += "_";
        values += numtracked;  // was: largest_range
        globalClient->text(values);
    }

    // update_stats();
}
