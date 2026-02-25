/*
 * Log.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
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

#include "SoC.h"
#include "Log.h"
#include "EEPROM.h"
#include "PNET.h"
#include "GNSS.h"
#include "global.h"
#include "Time.h"

File DebugLog;
bool DebugLogOpen = false;

void OpenDebugLog()
{
#if FILE_LOGGER
  if (ogn_debug) {
    //if (SPIFFS.begin(true)) {
      bool append = false;
      if (SPIFFS.exists("/debuglog.txt") && SPIFFS.totalBytes() - SPIFFS.usedBytes() > 10000)
          append = true;
      DebugLog = SPIFFS.open("/debuglog.txt", (append? FILE_APPEND : FILE_WRITE));
      if (DebugLog) {
          DebugLogOpen = true;
      } else {
          Serial.println(F("Failed to open debuglog.txt"));
      }
    //} else {
    //    Serial.println(F("Failed to start SPIFFS"));
    //}
  }
#endif
}

void DebugLogWrite(const char *s)
{
#if FILE_LOGGER
    if (! DebugLogOpen)
        return;
    char buf[80];
    snprintf (buf, sizeof(buf), "[%02d:%02d] %s\r\n",
       ThisAircraft.hour, ThisAircraft.minute, s);
    DebugLog.write((const uint8_t *)buf, strlen(buf));
    DebugLog.flush();
#endif
}

void LogDate()
{
#if FILE_LOGGER
    static bool done = false;
    if (! DebugLogOpen)
        return;
    if (this_month == 0)   // UTC time not known yet
        return;
    if (done)              // only report the date once
        return;
    done = true;
    String resetReason, logmsg;
    logmsg = "version ";
    logmsg += SOFTRF_FIRMWARE_VERSION;
    resetReason = SoC->getResetReason();
    logmsg += " restarted after " + resetReason;
    char buf[80];
    snprintf (buf, sizeof(buf), "Date: %d/%d %s\r\n", this_month, this_day, logmsg.c_str());
    DebugLogWrite(buf);
#endif
}

void Logger_send_udp(String* buf)
{
#if UDP_LOGGER

    if (ogn_debug && !ognrelay_enable)
    {
        int  debug_len = buf->length() + 1;
        byte debug_msg[debug_len];
        buf->getBytes(debug_msg, debug_len);
        SoC->WiFi_transmit_UDP_debug(ogn_debugport, debug_msg, debug_len);
        
        if(remotelogs_enable){
          char *encrypted;
          size_t encrypted_len;       
          //PNETencrypt(debug_msg, debug_len, &encrypted, &encrypted_len);
          //SoC->WiFi_transmit_UDP(remotelogs_server.c_str(), remotelogs_port, (byte*)encrypted, encrypted_len);
          //free(encrypted);
          }        
    }

#endif
}

#if LOGGER_IS_ENABLED

#define LOGFILE "/Logfile.txt"

File      LogFile;
FSInfo    fs_info;
FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

void Logger_setup()  // <<<>>> this is not called from anywhere
{
    char LogFilename[] = LOGFILE;


    if (SPIFFS.begin())
    {
        Serial.println(F("SPIFFS volume is mounted successfully."));

        SPIFFS.info(fs_info);

        Serial.println();
        Serial.print(F("Total bytes: "));
        Serial.println(fs_info.totalBytes);
        Serial.print(F("Used bytes: "));
        Serial.println(fs_info.usedBytes);
        Serial.print(F("Block size: "));
        Serial.println(fs_info.blockSize);
        Serial.print(F("Page size: "));
        Serial.println(fs_info.pageSize);

        //if (SPIFFS.exists(LogFilename)) SPIFFS.remove(LogFilename);

        LogFile = SPIFFS.open(LogFilename, "a+");

        if (!LogFile)
        {
            Serial.print(F("Unable to open log file: "));
            Serial.println(LogFilename);
        }
        else
        {
            LogFile.println();
            LogFile.println(F("******* Logging is restarted *******"));
            LogFile.print(F("*** Storage free space: "));
            LogFile.print(fs_info.totalBytes - fs_info.usedBytes);
            LogFile.println(F(" bytes ***"));

            //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
            ftpSrv.begin("softrf", "softrf");
        };

    } else {
        Serial.println(F("ERROR: Unable to mount SPIFFS volume."));
    }
}

void Logger_loop()
{
    ftpSrv.handleFTP();
}

void Logger_fini()
{
    LogFile.close();
    SPIFFS.end();
}

#endif /* LOGGER_IS_ENABLED */
