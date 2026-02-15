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
#include "APRS.h"

#include <string>

#include <ArduinoJson.h>

#include <ErriezCRC32.h>

#include <FS.h>   // Include the SPIFFS library

#define  U_PART U_FLASH
#define INDEX_CRC 3948812196

extern unsigned long ExportTimeWebRefresh;

File fsUploadFile;

#define countelems(a) (sizeof(a) / sizeof(a[0]))

//AsyncWebServer wserver(80);
unsigned int webserver_port = 80;  // default
AsyncWebServer *wserver = NULL;
// - Create AsyncWebServer object later, on port number specified in config
AsyncWebSocket ws("/ws");

AsyncWebSocketClient* globalClient = NULL;

static const char upload_templ[] PROGMEM =
"<html>\
 <head>\
 <meta http-equiv='Content-Type' content='text/html; charset=utf-8'>\
 <meta http-equiv='cache-control' content='no-cache'>\
 </head>\
 <p>%s</p>\
 <p>%s</p>\
 Files on device:<br>%s<br>\
 <div class = 'upload'>\
 <form method = 'POST' action = '/doUpload' enctype='multipart/form-data'>\
 <input type='file' name='data'/><input type='submit' name='upload' value='Upload' title = 'Upload Files'>\
 </form></div>\
 <p><a href='oldconf' class='upload'>Restore Old Config</a></p>\
 <p><a href='dnload' class='upload'>Download Config</a></p>\
 <p><a href='clear' class='clear'>Clear All Files</a></p>\
 <p><a href='update' class='clear'>Update Firmware</a></p>\
 <p><a href='reboot' class='clear'>Reboot</a></p>\
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
 <p>&nbsp;\r\n Uptime: %d minutes</p>\
 <p>&nbsp;\r\n Free RAM: %d bytes</p>\
 <p>&nbsp;\r\n Incoming packets bad CRC: %d</p>\
 <p>&nbsp;\r\n Incoming packets corrected: %d</p>\
 <p>&nbsp;\r\n Traffic packets received: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  per minute: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  in old protocol: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  air-relayed: %d</p>\
 <p>&nbsp;\r\n Traffic packets reported: %d</p>\
 <p>&nbsp;\r\n Aircraft seen ever: %d, today: %d</p>\
 <p>&nbsp;\r\n Largest Range: %d km</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n lat: %.4f</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n lon: %.4f</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n alt: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n ID: %06X</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <br><p>\r\nRemote Station Stats:</p>\
 <p>&nbsp;\r\n Uptime: %d minutes</p>\
 <p>&nbsp;\r\n Free RAM: %d bytes</p>\
 <p>&nbsp;\r\n Incoming packets bad CRC: %d</p>\
 <p>&nbsp;\r\n Incoming packets corrected: %d</p>\
 <p>&nbsp;\r\n Traffic packets: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  per minute: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  in old protocol: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  air-relayed: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct relayed to base: %d</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <p>&nbsp;\r\n Battery voltage: %.1f</p>\
 <br><p>\r\nTime-Relay Stats:</p>\
 <p>&nbsp;\r\n Time packets sent: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct acknowledged: %d</p>\
 <p>&nbsp;\r\n Time packets received: %d</p>\
 <p>&nbsp;\r\n Time-sync restarts: %d</p>\
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

static const char update_html[] PROGMEM =
 "<form method='POST' action='/doUpdate' enctype='multipart/form-data'>\
  <input type='file' name='update'><input type='submit' value='Update'></form>";

void handleUpdate(AsyncWebServerRequest* request)
{
    request->send(200, "text/html", update_html);
}

const char *index_msg = "index.html OK";

#define UPLHTMSIZE 1250
#define FILELSTSIZ 600

void handleUpload(AsyncWebServerRequest* request)
{
  char *upload_html = (char *) malloc(UPLHTMSIZE);
  if (! upload_html) {
      Serial.println("cannot allocate memory for upload page");
      return;
  }
  bool listfiles = true;
  char *filelist = (char *) malloc(FILELSTSIZ);
  if (! filelist) {
      Serial.println("cannot allocate memory for file list");
      listfiles = false;
  }
  const char *msg;
#if defined(ESP32)
  if (ttgo_on_tbeam)
      msg = "non-T-Beam binary running on T-Beam";
  else
#endif
  if (config_done < -4)
      msg = "could not open SPIFFS file system";
  else if (config_done == -4)
      msg = "config.json did not exist at boot";
  else if (config_done == -3)
      msg = "could not open config.json at boot";
  else if (config_done == -2)
      msg = "error parsing config.json at boot";
  else if (config_done == -1)
      msg = "config.json version not valid at boot";
  else if (config_done == 0)
      msg = "no wifi info in config.json at boot";
  else
      msg = "success parsing config.json at boot";
#if FILE_LOGGER
  if (! ogn_debug)
      SPIFFS.remove("/debuglog.txt");
#endif
  if (listfiles) {
    filelist[0] = '\0';
    int nfiles = 0;
    File root = SPIFFS.open("/");
    Serial.println(F("Files in SPIFFS:"));
    File file = root.openNextFile();
    while(file){
        Serial.print("... ");
        Serial.print(file.name());
        Serial.print("  [");
        Serial.print(file.size());
        Serial.println(" bytes]");
        int len = strlen(filelist);
        if (len < FILELSTSIZ-80) {
          snprintf(filelist+len, FILELSTSIZ-len,
             "&nbsp;&nbsp;%s&nbsp;&nbsp;[%d bytes]<br>", file.name(), file.size());
        } else {
          Serial.println("...");
          snprintf(filelist+len, FILELSTSIZ-len, "...<br>");
          break;
        }
        ++nfiles;
        file = root.openNextFile();
    }
    if (nfiles == 0) {
        Serial.println("... (none)");
        snprintf(filelist, FILELSTSIZ, "&nbsp;&nbsp;(none)<br>");
    }
    file.close();
    root.close();
  }
  snprintf(upload_html, UPLHTMSIZE, upload_templ, msg, index_msg,
       (listfiles? filelist : "(cannot allocate memory for file list)"));
  request->send(200, "text/html", upload_html);
  if (listfiles)
      free(filelist);
  free(upload_html);
}

void HandleOldconf(AsyncWebServerRequest* request)
{
            if (SPIFFS.exists("/oldconf.json")) {
                if (SPIFFS.exists("/config.json"))
                    SPIFFS.remove("/config.json");
                SPIFFS.rename("/oldconf.json", "/config.json");
            }
}

void DoUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final)
{
    bool was_open = false;
#if FILE_LOGGER
    if (strcmp(filename.c_str(),"debuglog.txt")==0) {
        if (DebugLogOpen) {
            DebugLog.close();
            was_open = true;
        }
    }
#endif
    if (!index) {
        // if uploading config, keep a copy of the existing config
        if (strcmp(filename.c_str(),"config.json")==0) {
            if (SPIFFS.exists("/config.json")) {
                if (SPIFFS.exists("/oldconf.json"))
                    SPIFFS.remove("/oldconf.json");
                SPIFFS.rename("/config.json", "/oldconf.json");
            }
        }
        request->_tempFile = SPIFFS.open("/" + filename, "w");
    }
    if (len) {
        // stream the incoming chunk to the opened file
        Serial.print(F("writing "));
        Serial.print(len);
        Serial.println(F("bytes to file..."));
        request->_tempFile.write(data, len);
    }
    if (final)
    {
        Serial.println(F("closing file..."));
        request->_tempFile.close();
        request->redirect("/");
    }
#if FILE_LOGGER
    if (was_open)
        DebugLog = SPIFFS.open("/debuglog.txt", FILE_APPEND);
#endif
}

void handleDnload(AsyncWebServerRequest* request)
{
  if (SPIFFS.exists("/config.json")) {
    request->send(SPIFFS, "/config.json", String(), true);
    Serial.println(F("Sent config.json file"));
  } else {
    request->send(404, "text/plain", "config.json not found");
    Serial.println(F("Config.json not found"));
  }
}

void handleDnldlog(AsyncWebServerRequest* request)
{
  bool was_open = false;
#if FILE_LOGGER
  if (DebugLogOpen) {
      DebugLog.close();
      was_open = true;
  }
#endif
  if (SPIFFS.exists("/debuglog.txt")) {
    request->send(SPIFFS, "/debuglog.txt", String(), true);
    Serial.println(F("Sent debuglog.txt file"));
  } else {
    request->send(404, "text/plain", "debuglog.txt not found");
    Serial.println(F("debuglog.txt not found"));
  }
#if FILE_LOGGER
  if (was_open)
      DebugLog = SPIFFS.open("/debuglog.txt", FILE_APPEND);
#endif
}

//void notFound(AsyncWebServerRequest *request) {
//    request->send(404, "text/plain", "file not found");
//}

void handleDoUpdate(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final)
{
    static int ota_counter;
    static size_t ota_bytes;
    static size_t content_len;
    String msg;

    if (!index)
    {
        msg = "updating firmware";
        Logger_send_udp(&msg);

       //SPIFFS.format();
        
        content_len = request->contentLength();
        Serial.print("OTA update: content_len ");
        Serial.println(content_len);
        ota_counter = 0;
        ota_bytes = 0;
        // if filename includes spiffs, update the spiffs partition
        int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
        {
            Update.printError(Serial);
        }
    }

    ++ota_counter;
    ota_bytes += len;
    if ((ota_counter & 0x3F) == 0) {
        char buf[16];
        snprintf(buf, 16, "update %d%%...", (100 * ota_bytes)/(content_len+1));
        OLED_write(buf, 0, 27, true);
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
    }

    if (final)
    {
        Serial.println("\r\nOTA finalizing...");
        AsyncWebServerResponse* response = request->beginResponse(302,
             "text/plain", "Please wait while the device reboots");
        response->addHeader("Refresh", "20");
        response->addHeader("Location", "/");
        request->send(response);
        if (!Update.end(true))
            Update.printError(Serial);
        else
        {
            OLED_write("rebooting...", 0, 27, true);
            DebugLogWrite("firmware update restart");
            delay(5000);
            RF_Shutdown();
            WiFi_fini();
            delay(500);
            SoC->reset();
        }
    }
}

#define STATS_SIZE 2199
char stats_html[STATS_SIZE+1];

void update_stats()
{
  uint32_t zero = 0;
  if (ognrelay_base && ognrelay_time) {
     if (time_synched)
        ognopmode = "Base station, got time from remote";
     else
        ognopmode = "Base station awaiting time from remote";
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime, ESP.getFreeHeap(),
        packets_failed_crc,
        packets_corrected,
        traffic_packets_recvd, zero,
        old_protocol_packets_recvd,
        air_relayed_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        farthest_lat,
        farthest_lon,
        (uint32_t) farthest_alt,
        (uint32_t) farthest_ID,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        remote_uptime, zero, zero, zero,
        remote_traffic,
        (uint32_t) packets_per_minute,
        zero, zero,
        (uint32_t) remote_pctrel,
        (uint32_t) remote_bad,
        (uint32_t) remote_other,
        remote_voltage,
        (uint32_t) remote_timesent,
        (uint32_t) remote_ack,
        zero,
        (uint32_t) remote_restarts);
  } else if (ognrelay_base && ognreverse_time) {
     if (time_synched)
        ognopmode = "Base station sending time, remote synched";
     else
        ognopmode = "Base station trying to send time to remote";
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime, ESP.getFreeHeap(),
        packets_failed_crc,
        packets_corrected,
        traffic_packets_recvd, zero,
        old_protocol_packets_recvd,
        air_relayed_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        farthest_lat,
        farthest_lon,
        (uint32_t) farthest_alt,
        (uint32_t) farthest_ID,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        remote_uptime, zero, zero, zero,
        remote_traffic,
        (uint32_t) packets_per_minute,
        zero, zero,
        (uint32_t) remote_pctrel,
        (uint32_t) remote_bad,
        (uint32_t) remote_other,
        remote_voltage,
        (uint32_t) time_packets_sent,    // we also have remote_timerecvd but don't report it
        (uint32_t) (time_packets_sent? ((100*ack_packets_recvd) / time_packets_sent) : 0),
        zero,
        (uint32_t) sync_restarts);
  } else if (ognrelay_enable) {
     if (ognreverse_time) {
         if (time_synched)
             ognopmode = "Remote station, got time from base";
         else
             ognopmode = "Remote station awaiting time from base";
     } else if (ognrelay_time) {
         if (time_synched)
             ognopmode = "Remote station sending time, base synched";
         else
             ognopmode = "Remote station trying to send time to base";
     }
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        zero, zero, zero, zero, zero,
        zero, zero, zero, zero, zero,
        zero, zero,
        (float) 0, (float) 0,
        zero, zero,
        zero, zero,
        uptime, ESP.getFreeHeap(),
        packets_failed_crc,
        packets_corrected,
        traffic_packets_recvd,
        (uint32_t) packets_per_minute,
        old_protocol_packets_recvd,
        air_relayed_packets_recvd,
        (uint32_t) ((100 * traffic_packets_relayed)
                      / (traffic_packets_recvd+1)),
        (uint32_t) bad_packets_recvd,
        (uint32_t) other_packets_recvd,
        (float) Battery_voltage(),
        (ognrelay_time? (uint32_t) time_packets_sent : zero),
        (ognrelay_time? (uint32_t) (time_packets_sent? ((100*ack_packets_recvd) / time_packets_sent) : 0) : zero),
        (ognreverse_time ? (uint32_t) time_packets_recvd : zero),
        (uint32_t) sync_restarts);
  } else {  // single standalone station, or base with no time relay
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime, ESP.getFreeHeap(),
        packets_failed_crc,
        packets_corrected,
        traffic_packets_recvd,
        (uint32_t) packets_per_minute,
        old_protocol_packets_recvd,
        air_relayed_packets_recvd,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        farthest_lat,
        farthest_lon,
        (uint32_t) farthest_alt,
        (uint32_t) farthest_ID,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        zero, zero, zero, zero, zero, zero,
        zero, zero, zero, zero, zero,
        (float) Battery_voltage(),
        zero, zero, zero, zero);
  }
  stats_html[STATS_SIZE] = '\0';
Serial.print("stats_html size: ");  Serial.println(strlen(stats_html));
//Serial.println(stats_html);
}

#define NOISE_SIZE 1199
char noise_text[NOISE_SIZE+1];

void update_noise()
{
    char *p = noise_text;
    int n = NOISE_SIZE;
    int nchans = (ogn_band==RF_BAND_US? 65 : ogn_band==RF_BAND_AU? 24 : ogn_band==RF_BAND_EU? 2 : 1);
    for (int i=0; i<nchans; i++) {
        int count = noise_count[i];
        int data = noise_data[i];
        noise_count[i] = 0;
        noise_data[i] = 0;
        snprintf(p, n, "%d,%d,%d\r\n", i, count, data);
        int m = strlen(p);
        p += m;
        n -= m;
    }
    noise_text[NOISE_SIZE] = '\0';
//Serial.println(noise_text);
}

void update_rnoise()
{
    char *p = noise_text;
    int n = NOISE_SIZE;
    int nchans = (ogn_band==RF_BAND_US? 65 : ogn_band==RF_BAND_AU? 24 : ogn_band==RF_BAND_EU? 2 : 1);
    for (int i=0; i<nchans; i++) {
        int count = remote_noise_count[i];
        int data = remote_noise_data[i];
        remote_noise_count[i] = 0;
        remote_noise_data[i] = 0;
        snprintf(p, n, "%d,%d,%d\r\n", i, count, data);
        int m = strlen(p);
        p += m;
        n -= m;
    }
    noise_text[NOISE_SIZE] = '\0';
//Serial.println(noise_text);
}

void Web_start()
{
    wserver->begin();
}

void Web_stop()
{
    wserver->end();
}

/* if no config, just offer to upload config (and a few other ops) */
void mini_server()
{
    // wserver->on("/", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
    //wserver->on("/", HTTP_GET, [](AsyncWebServerRequest* request){
    //    request->send(200, "text/html", upload_html);
    //});

    wserver->on("/", HTTP_GET, [](AsyncWebServerRequest* request){
      handleUpload(request);
    });

    wserver->on("/oldconf", HTTP_GET, [](AsyncWebServerRequest* request){
      HandleOldconf(request);
    });

    wserver->on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, DoUpload);

    // download config even though it is invalid - so can fix it
    wserver->on("/dnload", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnload(request);
    });

    wserver->on("/debuglog.txt", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnldlog(request);
    });

    wserver->on("/clear", HTTP_GET, [](AsyncWebServerRequest* request){
        Serial.println(F("Formatting spiffs..."));
        SPIFFS.format();
        request->redirect("/");
    });

    wserver->on("/update", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpdate(request);
        request->redirect("/");
    });

    wserver->on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        request->redirect("/");
        DebugLogWrite("mini_web_server reboot");
        delay(500);
        SoC->reset();
    });    

    Web_start();
}

int ssid_num = 0;

bool status_page_setup()
{

    if (!SPIFFS.exists("/index.html"))
    {
        index_msg = "index.html does not exist";
        Serial.println(index_msg);
        return false;
    }

    File file = SPIFFS.open("/index.html", "r");
    if (!file)
    {
        index_msg = "Error reading index.html - erased";
        Serial.println(index_msg);
        SPIFFS.remove("/index.html");
        return false;
    }

    yield();

    ws.onEvent(onWsEvent);
    wserver->addHandler(&ws);

    size_t filesize = file.size();
    char* index_html = (char *) malloc(filesize + 1);
    if (index_html == NULL) {
        index_msg = "failed to allocate RAM for reading index.html";
        Serial.println(index_msg);
        return false;
    }

    file.read((uint8_t *)index_html, filesize);
    index_html[filesize] = '\0';
    file.close();

    bool wrong_version = true;
    char *cp = strstr(index_html, "meta name=\"OGNbase-Version\"");
    if (cp != NULL) {
        cp += 37;
        char versionbuf[8];
        int ii = 0;
        while (*cp != '\"' && *cp != '\0' && ii < 7)
            versionbuf[ii++] = *cp++;
        versionbuf[ii] = '\0';
        if (strcmp(versionbuf,OGNBASE_HTML_VERSION) == 0)
            wrong_version = false;
    }

    if (wrong_version) {
        index_msg = "Wrong version of index.html - erased";
        Serial.println(index_msg);
        OLED_write("Wrong index.html version", 0, 27, true);
        delay(1000);
        SPIFFS.remove("/index.html");
        free(index_html);
        return false;
    }

    char* offset = NULL;
    size_t size = 12300;    // currently fits in under 9000
    char* Settings_temp = (char *) malloc(size);
    if (Settings_temp == NULL) {
        index_msg = "failed to allocate RAM for web page";
        Serial.println(index_msg);
        free(index_html);
        return false;
    }

    Serial.print("Free memory after Web_setup() malloc()s: ");
    Serial.println(ESP.getFreeHeap());

    index_msg = "";

    offset = Settings_temp;

    String station_addr = String(ThisAircraft.addr, HEX);
    station_addr.toUpperCase();

    // to_string() formats to 6 decimals
    //String pos_lat = to_string(ogn_lat);
    //String pos_lon = to_string(ogn_lon);
    char buf_lat[16];
    snprintf(buf_lat,16,"%.6f",ogn_lat);
    char buf_lon[16];
    snprintf(buf_lon,16,"%.6f",ogn_lon);
    String pos_alt = "";
    pos_alt.concat(ogn_alt);
    String pos_geo = "";
    pos_geo.concat(ogn_geoid_separation);

    String version = SOFTRF_FIRMWARE_VERSION;
    version += " (";
    version += rf_chip->name;
    version += ")";

    // the first few messages here will never be shown, because
    // the status page is not loaded without a configuration file
    if (config_done == -5)
        ognopmode = "could not open SPIFFS file system";
    else if (config_done == -4)
        ognopmode = "config.json did not exist at boot";
    else if (config_done == -3)
        ognopmode = "could not open config.json at boot";
    else if (config_done == -2)
        ognopmode = "error parsing config.json at boot";
    else if (config_done == -1)
        ognopmode = "config.json version not valid at boot";
    else if (config_done <= 0)
        ognopmode = "no wifi info in config.json at boot";
    else if (ognrelay_base && ognrelay_time)
        ognopmode = "Base station, time relayed from remote";
    else if (ognrelay_base && ogn_gnsstime)
        ognopmode = "Base station, time from GNSS";
    else if (ognrelay_base)
        ognopmode = "Base station, time from NTP";
    else if (ognrelay_enable && ognrelay_time && ogn_gnsstime)
        ognopmode = "Remote station, relaying GNSS time";
    else if (ognrelay_enable && ogn_gnsstime)
        ognopmode = "Remote station, time from GNSS"; 
    else if (ognrelay_enable && ognreverse_time)
        ognopmode = "Remote station, expecting time from base";
    else if (ognrelay_enable)
        ognopmode = "Remote station, no time source";
    else if (ogn_gnsstime)
        ognopmode = "Single station, time from GNSS";
    else
        ognopmode = "Single station, time from NTP";

    yield();

    snprintf(offset, size, index_html,
             station_addr,
             version.c_str(),
             ognopmode,
             ogn_callsign,
             buf_lat,
             buf_lon,
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
             (ogn_band == RF_BAND_IL ? "selected" : ""), RF_BAND_IL,
             (ogn_band == RF_BAND_KR ? "selected" : ""), RF_BAND_KR,

             (ogn_mobile ? "selected" : ""),
             (ogn_mobile ? "" : "selected"),

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

             //ogn_ssid[0].c_str(),
             WiFi.SSID().c_str(),

             /*Hide Wifi Password*/
             "hidepass",

             (ogn_sleepmode == 0 ? "selected" : ""), "Disabled",
             (ogn_sleepmode == 1 ? "selected" : ""), "Enabled",
             (ogn_sleepmode == 2 ? "selected" : ""), "Follow Remote",

             (zabbix_enable == 0 ? "selected" : ""), "Disabled",
             (zabbix_enable == 1 ? "selected" : ""), "Enabled",

             String(ogn_rxidle/60),
             String(ogn_wakeuptimer/60),
             String(ogn_morning),
             String(ogn_evening),

             (ognrelay_enable == 0 ? "selected" : ""), "Disabled",
             (ognrelay_enable == 1 ? "selected" : ""), "Enabled",
             (ognrelay_base == 0 ? "selected" : ""), "Disabled",
             (ognrelay_base == 1 ? "selected" : ""), "Enabled",

             (ognrelay_time == 0 ? "selected" : ""), "Disabled",
             (ognrelay_time == 1 ? "selected" : ""), "Enabled",
             (ognreverse_time == 0 ? "selected" : ""), "Disabled",
             (ognreverse_time == 1 ? "selected" : ""), "Enabled",

             (ogn_gnsstime == 0 ? "selected" : ""), "Disabled",
             (ogn_gnsstime == 1 ? "selected" : ""), "Enabled",
             (testmode_enable == 0 ? "selected" : ""), "Disabled",
             (testmode_enable == 1 ? "selected" : ""), "Enabled",

             "hidekey", "hidekey"
             );

    size_t len = strlen(offset);
    if (len+2 < size) {
        Serial.print("Size of web page: ");
        Serial.println(len);
    } else {
        Serial.print("Web page needs more bytes than the allocated ");
        Serial.println(size);
        free(Settings_temp);
        free(index_html);
        index_msg = "allocated RAM insufficient for web page";
        return false;
    }

    yield();

    //String status_page_string = String(Settings_temp);

    wserver->on("/", HTTP_GET, [Settings_temp](AsyncWebServerRequest* request){
        request->send(200, "text/html", Settings_temp);
    });

    free(Settings_temp);
    free(index_html);

    return true;
}

void Web_setup(ufo_t* this_aircraft)
{
    Serial.print("Free memory entering Web_setup(): ");
    Serial.println(ESP.getFreeHeap());

    wserver = new AsyncWebServer(webserver_port);
    if (wserver == NULL) {
        Serial.println("failed to create web server object");
        return;
    }

#if defined(ESP32)
    if (ttgo_on_tbeam)
    {
        index_msg = "Please load TBEAM firmware";
        Serial.println(index_msg);
        mini_server();
        OLED_write(index_msg, 0, 27, true);
        delay(9000);
        return;
    }
#endif

    if (!SPIFFS.begin(true))
    {
        index_msg = "An Error has occurred while mounting SPIFFS";
        Serial.println(index_msg);
        mini_server();
        return;
    }

    if (status_page_setup() == false) {
        mini_server();
        return;
    }

    ssid_num = 0;
    for (int sn=0; sn<5; sn++) {
       if (WiFi.SSID() == ogn_ssid[sn]) {
           ssid_num = sn;   // using the SSID from this slot
Serial.print("using SSID #");
Serial.println(ssid_num);
          // if new SSID entered later, overwrite this same slot
       }
    }

    // Route to load style.css file
    wserver->on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    wserver->on("/update", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpdate(request);
        request->redirect("/");
    });

    wserver->on("/doUpdate", HTTP_POST,
               [](AsyncWebServerRequest* request) {},
               [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data,
                  size_t len, bool final) {
        handleDoUpdate(request, filename, index, data, len, final);
    });

    // wserver->on("/upload", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
    //wserver->on("/upload", HTTP_GET, [](AsyncWebServerRequest* request){
    //    request->send(200, "text/html", upload_html);
    //});

    wserver->on("/upload", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpload(request);
    });

    wserver->on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, DoUpload);

    wserver->on("/oldconf", HTTP_GET, [](AsyncWebServerRequest* request){
      HandleOldconf(request);
    });

    wserver->on("/dnload", HTTP_GET, [](AsyncWebServerRequest* request){
        handleDnload(request);
    });

    wserver->on("/debuglog.txt", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnldlog(request);
    });

    wserver->on("/clear", HTTP_GET, [](AsyncWebServerRequest* request){
        Serial.println(F("Formatting spiffs..."));
        SPIFFS.format();
        request->redirect("/");
    });    

    snprintf(stats_html, STATS_SIZE, "stats not available yet");

    wserver->on("/stats", HTTP_GET, [](AsyncWebServerRequest* request){
        update_stats();
        Serial.println(F("requesting stats page..."));
        request->send(200, "text/html", String(stats_html));
    });

    wserver->on("/noise", HTTP_GET, [](AsyncWebServerRequest* request){
        update_noise();
        Serial.println(F("requesting noise page..."));
        request->send(200, "text/plain", String(noise_text));
    });

    wserver->on("/remote_noise", HTTP_GET, [](AsyncWebServerRequest* request){
        update_rnoise();
        Serial.println(F("requesting remote noise page..."));
        request->send(200, "text/plain", String(noise_text));
    });

    wserver->on("/refresh", HTTP_GET, [](AsyncWebServerRequest* request){
        /* refresh the stats at the top of the status page in 2 sec */
        ExportTimeWebRefresh = millis()/1000 - 21;
        request->redirect("/");
    });    

    wserver->on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        request->redirect("/");
        DebugLogWrite("web_server reboot");
        delay(500);
        SoC->reset();
    });    

    wserver->on("/remote_reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        if (reboot_remote()) {
          Serial.println(F("sent remote reboot packet..."));
          OLED_write("remote reboot...", 0, 27, true);
          delay(1000);
          // should display something on the web page too.
          // for now just some hints:
          remote_sats = 0;            /* hints that remote is restarting */
          remote_uptime = 0;
          //uptime = 0;
          // - but none of the above is shown on the status page any more!
          remote_voltage = 0;
          ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top of the status page */
        }
        request->redirect("/");
    });    

//    wserver->onNotFound([](AsyncWebServerRequest *request){
//        request->send(404, "text/plain", "file not found.");
//    });

//    wserver->onNotFound(notFound);

    yield();

    // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
    wserver->on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
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

        //geoid_separation
        if (request->hasParam("ogn_geoid"))
            ogn_geoid_separation = request->getParam("ogn_geoid")->value().toInt();

        if (request->hasParam("ogn_mobile"))
            ogn_mobile = request->getParam("ogn_mobile")->value().toInt();

        yield();

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

        if (request->hasParam("ogn_aprs_debug"))
            ogn_debug= request->getParam("ogn_aprs_debug")->value().toInt();

        if (request->hasParam("aprs_debug_port"))
            ogn_debugport = request->getParam("aprs_debug_port")->value().toInt();

        if (request->hasParam("ogn_range"))
            ogn_range = request->getParam("ogn_range")->value().toInt();

        //if (request->hasParam("ogn_agc"))
        //    settings->sxlna = request->getParam("ogn_agc")->value().toInt();

        yield();

        if (request->hasParam("ogn_ssid"))
               ogn_ssid[ssid_num] = request->getParam("ogn_ssid")->value().c_str();
        if (request->hasParam("ogn_wifi_password")){
              if (request->getParam("ogn_wifi_password")->value() != "hidepass")
                ogn_wpass[ssid_num] = request->getParam("ogn_wifi_password")->value().c_str();
        }

        if (request->hasParam("ogn_deep_sleep"))
            ogn_sleepmode = request->getParam("ogn_deep_sleep")->value().toInt();

        if (request->hasParam("zabbix_trap_en"))
            zabbix_enable = request->getParam("zabbix_trap_en")->value().toInt();

        yield();

        if (request->hasParam("ogn_sleep_time")) {
            ogn_rxidle = 60 * request->getParam("ogn_sleep_time")->value().toInt();
            if (ogn_rxidle != 0 && ogn_rxidle < 600)  ogn_rxidle = 600;
        }
        if (request->hasParam("ogn_wakeup_time")) {
            ogn_wakeuptimer = 60 * request->getParam("ogn_wakeup_time")->value().toInt();
            if (ogn_wakeuptimer != 0 && ogn_wakeuptimer < 600)  ogn_wakeuptimer = 600;
        }

        if (request->hasParam("ogn_morning")) {
            ogn_morning = request->getParam("ogn_morning")->value().toInt();
            if (ogn_morning <  5)  ogn_morning =  5;
            if (ogn_morning > 14)  ogn_morning = 14;
        }
        if (request->hasParam("ogn_evening")) {
            ogn_evening = request->getParam("ogn_evening")->value().toInt();
            if (ogn_evening < 15)  ogn_evening = 15;
            if (ogn_evening > 22)  ogn_evening = 22;
        }

        if (request->hasParam("relay_enable"))
            ognrelay_enable = request->getParam("relay_enable")->value().toInt();
        if (request->hasParam("base_enable"))
            ognrelay_base = request->getParam("base_enable")->value().toInt();
        if (ognrelay_base)
            ognrelay_enable = 0;
        if (request->hasParam("relay_time"))
            ognrelay_time = request->getParam("relay_time")->value().toInt();
        if (request->hasParam("reverse_time"))
            ognreverse_time = request->getParam("reverse_time")->value().toInt();
        if (ognreverse_time)
            ognrelay_time = 0;
        if (request->hasParam("gnss_time"))
            ogn_gnsstime = request->getParam("gnss_time")->value().toInt();

        if (request->hasParam("ogn_relay_key")) {
            if (request->getParam("ogn_relay_key")->value() != "hidekey") {
              ognrelay_key = request->getParam("ogn_relay_key")->value().toInt();
            }
        }

        if (request->hasParam("webserver_port")) {
            if (request->getParam("webserver_port")->value() != "hidekey") {
              unsigned int port = request->getParam("webserver_port")->value().toInt();
              if (port==80 || (port >= 49152 && port <= 65535))
                  webserver_port = port;
            }
        }

        if (request->hasParam("test_mode_en"))
            testmode_enable = request->getParam("test_mode_en")->value().toInt();

        beers_show = false;

        yield();

        request->redirect("/");
#if 0
        // ogn_reset_all
        if (request->hasParam("ogn_reset_all"))
            if (request->getParam("ogn_reset_all")->value() == "on")
            {
                SPIFFS.format();
                RF_Shutdown();
                DebugLogWrite("ogn_reset_all reboot");
                delay(500);
                SoC->reset();
            }
#endif
        EEPROM_store();
        OGN_save_config();
        DebugLogWrite("web settings reboot");
        RF_Shutdown();
        delay(500);
        SoC->reset();
    });

    SoC->swSer_enableRx(true);

    yield();

    Serial.print("Free memory after Web_setup() free(): ");
    Serial.println(ESP.getFreeHeap());

    // Start server
    Web_start();
    delay(500);
    ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top on next web_loop() */

    Serial.print("Free memory after Web_start(): ");
    Serial.println(ESP.getFreeHeap());
}

void Web_loop(void)
{
    if (globalClient != NULL && globalClient->status() == WS_CONNECTED)
    {
        String values;

        values = (remote_voltage > 0 ? remote_voltage : Battery_voltage());
        values += "_";
        bool yesno = false;
        if (ognrelay_time || ognreverse_time)
            yesno = time_synched;
        else if (ogn_gnsstime)
            yesno = isValidGNSStime();
        else
            yesno = NTP_synched;
        if (yesno)
            values += "OK";
        else
            values += "--";
        values += "_";
        if (aprs_registred >= 1)
            values += "OK";
        else
            values += "--";
        values += "_";
        if (ognrelay_enable)
            values += traffic_packets_relayed;
        else
            values += traffic_packets_reported;
        values += "_";
        if (ognrelay_enable)
            values += "--";
        else
            values += numseen_today;
        values += "_";
        values += numtracked;
        globalClient->text(values);
    }

    if (ogn_mobile) {
        static bool mobile_located = false;
        if (mobile_located == false && isValidGNSSFix()) {
            mobile_located = true;
            // rebuild status page - to show the lat/lon
            if (status_page_setup() == false) {
                wserver->on("/", HTTP_GET, [](AsyncWebServerRequest* request){
                  handleUpload(request);
                });
            }
        }
    }

    // update_stats();
}
