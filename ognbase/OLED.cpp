/*
 * OLED.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include <Wire.h>

#include "OLED.h"
#include "EEPROM.h"
#include "Time.h"
#include "RF.h"
#include "GNSS.h"
#include "Battery.h"
#include "Traffic.h"
#include "global.h"
#include "Web.h"
#include "version.h"
#include "SoftRF.h"
#include "logos.h"

//SSD1306Wire display(SSD1306_OLED_I2C_ADDR, SDA, SCL, GEOMETRY_128_64, I2C_TWO, 400000);  // ADDRESS, SDA, SCL
// construct using the existing SDA, SCL:
SSD1306Wire display(SSD1306_OLED_I2C_ADDR, -1, -1, GEOMETRY_128_64, I2C_TWO, 400000);

bool display_init = false;
bool display_enabled = false;
bool OLED_blank = false;

int rssi      = 0;
int oled_site = 0;

enum
{
    OLED_PAGE_RADIO,
    OLED_PAGE_OTHER,
    OLED_PAGE_COUNT
};

const char* OLED_Protocol_ID[8] = {
    [RF_PROTOCOL_LEGACY]    = "L",
    [RF_PROTOCOL_OGNTP]     = "O",
    [RF_PROTOCOL_P3I]       = "P",
    [RF_PROTOCOL_ADSB_1090] = "A",
    [RF_PROTOCOL_ADSB_UAT]  = "U",
    [RF_PROTOCOL_FANET]     = "F"
};

const char* ISO3166_CC[12] = {
    [RF_BAND_AUTO] = "--",
    [RF_BAND_EU]   = "EU",
    [RF_BAND_US]   = "US",
    [RF_BAND_AU]   = "AU",
    [RF_BAND_NZ]   = "NZ",
    [RF_BAND_RU]   = "RU",
    [RF_BAND_CN]   = "CN",
    [RF_BAND_UK]   = "UK",
    [RF_BAND_IN]   = "IN",
    [RF_BAND_IL]   = "IL",
    [RF_BAND_KR]   = "KR"
};

bool OLED_probe() {
  // look for OLED on first I2C port
  Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire1.endTransmission() == 0) {
    display.setWire(&Wire1);
    return true;
#if defined(TBEAM)
  } else {
    // look for OLED on second I2C port
    Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
    if (Wire.endTransmission() == 0) {
      display.setWire(&Wire);
      return true;
    }
#endif
  }
  return false;
}

void OLED_setup()
{
    display_init = display.init();
    if (! display_init)
        return;
    display.flipScreenVertically();
    display_enabled = true;
}

void OLED_write(const char* text, short x, short y, bool clear)
{
    if (!display_enabled){return;}
      
    display.displayOn();
    if (clear)
        display.clear();
    display.drawString(x, y, text);
    display.display();
}

void OLED_disable()
{
  if (!display_enabled){return;}

  OLED_blank = true;
  display.clear();
  display.displayOff();
  display_enabled = false;
}

void OLED_enable()
{
  display_enabled = true;
  OLED_blank = false;
  display.displayOn();
}

void OLED_draw_Bitmap(int16_t x, int16_t y, uint8_t bm, bool clear)
{
    if (!display_enabled){return;}
    
    display.displayOn();
    if (clear)
        display.clear();
    if (bm == 1)
        display.drawXbm(x, y, 50, 28, wifi_50_28);
    if (bm == 2)
        // fanet_export_75_75
        display.drawXbm(x, y, 100, 63, fanet_export_100_63);
    if (bm == 3) //network network_50_44
        display.drawXbm(x, y, 50, 44, network_50_44);
    if (bm == 100)
        // supporter_50_64
        display.drawXbm(x, y, 50, 64, supporter_50_64);
    display.display();
}

void OLED_display()
{
    //display.displayOn();
    display.display();
//Serial.println("... OLED_info returning");
}

void OLED_info()
{
    if (!display_enabled){return;}

//Serial.println("OLED_info()...");

    char     buf[64];
    uint32_t disp_value;

    int     bars = 0;
    int32_t RSSI;

    RSSI = WiFi.RSSI();

    if (RSSI > -55)
        bars = 5;
    else if (RSSI < -55 & RSSI > -65)
        bars = 4;
    else if (RSSI < -65 & RSSI > -70)
        bars = 3;
    else if (RSSI < -70 & RSSI > -78)
        bars = 2;
    else if (RSSI < -78 & RSSI > -82)
        bars = 1;
    else
        bars = 0;
    Serial.print("Bars: ");
    Serial.println(bars);

    if (display_init)
    {
        //display.displayOff();
        display.displayOn();
        display.clear();

        display.setFont(ArialMT_Plain_24);
        display.drawString(95, 0, "RX");
        Serial.print("RX: ");
        display.setFont(ArialMT_Plain_16);
        snprintf(buf, sizeof(buf), "%d", rx_packets_counter);
        Serial.println(buf);
        display.drawString(100, 28, buf);
        display.setFont(ArialMT_Plain_10);

        //Serial.print("OLED page: ");
        //Serial.println(oled_site);

        if (oled_site == 0)
        {
            snprintf(buf, sizeof(buf), "ID: %06X", ThisAircraft.addr);
            display.drawString(0, 0, buf);
            Serial.println(buf);

            if (WiFi.getMode() == WIFI_OFF)
                snprintf(buf, sizeof(buf), "WIFI OFF");
            else if (WiFi.getMode() == WIFI_AP)
                // snprintf(buf, sizeof(buf), "SSID: %s", WiFi.SSID().c_str());
                snprintf(buf, sizeof(buf), "IP: %s", WiFi.softAPIP().toString().c_str());
            else
                snprintf(buf, sizeof(buf), "IP: %s", WiFi.localIP().toString().c_str());
            display.drawString(0, 9, buf);
            Serial.println(buf);
            Serial.print("MAC Address: ");
            Serial.println(WiFi.macAddress());

            if(ognrelay_enable)
              snprintf(buf, sizeof(buf), "CS-R: %s", ogn_callsign);
            else
              snprintf(buf, sizeof(buf), "CS: %s", ogn_callsign);
            display.drawString(0, 18, buf);
            Serial.println(buf);

            const char *band = ISO3166_CC[ogn_band];
            snprintf(buf, sizeof(buf), "Band: %s", (band==NULL? "?" : band));
            display.drawString(0, 27, buf);
            Serial.println(buf);

            //OLED_Protocol_ID[ThisAircraft.protocol]
            const char *protocol = OLED_Protocol_ID[ogn_protocol_1];
            snprintf(buf, sizeof(buf), "Prot: %s", (protocol==NULL? "?" : protocol));
            display.drawString(0, 36, buf);
            Serial.println(buf);

            //snprintf(buf, sizeof(buf), "UTC: %02d:%02d", hour(now()), minute(now()));
            snprintf(buf, sizeof(buf), "UTC: %02d:%02d:%02d",
                ThisAircraft.hour, ThisAircraft.minute, ThisAircraft.second);
            display.drawString(0, 45, buf);
            Serial.println(buf);

            if (!ognrelay_enable && !ogn_gnsstime) {
                //display.drawString(0, 54, "NTP: True");
                //display.drawString(0, 54, "Position Set");
                snprintf(buf, sizeof(buf), "MAC: %s", WiFi.macAddress().c_str());
                display.drawString(0, 54, buf);
                //Serial.println(buf);
            } else {
                if (ognrelay_base & ognrelay_time)
                  snprintf(buf, sizeof(buf), "R-GNSS: %d", remote_sats);
                else if (ogn_gnsstime)
                  snprintf(buf, sizeof(buf), "GNSS: %d", gnss.satellites.value());
                else
                  snprintf(buf, sizeof(buf), "GNSS not used");
                display.drawString(0, 54, buf);
                Serial.println(buf);
            }
            /*
               disp_value = settings->range;
               itoa(disp_value, buf, 10);
               snprintf (buf, sizeof(buf), "Range: %s", buf);
               display.drawString(0, 63, buf);
             */
            oled_site = 1;
            OLED_display();
            return;
        }
        if (oled_site == 1)
        {
            for (int b=0; b <= bars; b++)
                display.fillRect(100 + (b * 5), 65 - (b * 3), 3, b * 3);

            //version
            // snprintf(buf, sizeof(buf), "Version: %s", _VERSION);
            snprintf(buf, sizeof(buf), "Version: %s", SOFTRF_FIRMWARE_VERSION);
            display.drawString(0, 0, buf);
            Serial.println(buf);

            // RSSI
            disp_value = RF_last_rssi;
            snprintf(buf, sizeof(buf), "RSSI: %d", disp_value);
            display.drawString(0, 9, buf);
            Serial.println(buf);

            //ogndebug
            disp_value = ogn_debug;
            snprintf(buf, sizeof(buf), "Debug: %d", disp_value);
            display.drawString(0, 18, buf);
            //Serial.println(buf);

            //ogndebugp
            disp_value = ogn_debug;
            snprintf(buf, sizeof(buf), "DebugP: %d", disp_value);
            display.drawString(0, 27, buf);
            //Serial.println(buf);

            //bool ignore_stealth;
            //disp_value = ogn_istealthbit;
            //snprintf(buf, sizeof(buf), "IStealth: %d", disp_value);
            //display.drawString(0, 36, buf);
            //Serial.println(buf);

            //bool ignore_no_track;
            //disp_value = ogn_itrackbit;
            //snprintf(buf, sizeof(buf), "ITrack: %d", disp_value);
            //display.drawString(0, 45, buf);
            //Serial.println(buf);

            //zabbix_en
            disp_value = zabbix_enable;
            snprintf(buf, sizeof(buf), "Zabbix: %d", disp_value);
            display.drawString(0, 54, buf);
            //Serial.println(buf);

            if(!ognrelay_enable )
              oled_site = 2;
            else
              oled_site = 3;
            OLED_display();
            return;
        }

        if (oled_site == 2)
        {
            //display.clear();
            snprintf(buf, sizeof(buf), "SSID List");
            display.drawString(0, 0, buf);
            Serial.println(buf);

            for (int i=0; i < 5; i++)
                if (ogn_ssid[i] != "")
                {
                    snprintf(buf, sizeof(buf), "%s", ogn_ssid[i].c_str());
                    display.drawString(0, (i + 1) * 9, buf);
                    Serial.println(buf);
                }

            //for (int b=0; b <= bars; b++)
            //    display.fillRect(100 + (b * 5), 40 - (b * 6), 3, b * 6);

            snprintf(buf, sizeof(buf), "connected to %s", WiFi.SSID().c_str());
            display.drawString(0, 54, buf);
            Serial.println(buf);

            oled_site = 3;
            OLED_display();
            return;
        }

        if (oled_site == 3)
        {
            //display.clear();
            snprintf(buf, sizeof(buf), "POSITION DATA");
            display.drawString(0, 0, buf);
            //Serial.println(buf);

            snprintf(buf, sizeof(buf), "LAT: %.4f", ThisAircraft.latitude);
            display.drawString(0, 9, buf);
            Serial.println(buf);

            snprintf(buf, sizeof(buf), "LON:: %.4f", ThisAircraft.longitude);
            display.drawString(0, 18, buf);
            Serial.println(buf);

            snprintf(buf, sizeof(buf), "ALT: %.2f", ThisAircraft.altitude);
            display.drawString(0, 27, buf);
            Serial.println(buf);

            snprintf(buf, sizeof(buf), "GEOID: %.2f", ThisAircraft.geoid_separation);
            display.drawString(0, 36, buf);
            Serial.println(buf);

            snprintf(buf, sizeof(buf), "GPS-FIX: %s", isValidFix() ? "true" : "false");
            display.drawString(0, 45, buf);       
            Serial.println(buf);

            if (ognrelay_base) {
              if (remote_voltage == 0.0)
                snprintf(buf, sizeof(buf), "RELAY-Base R:---V");
              else
                snprintf(buf, sizeof(buf), "RELAY-Base R:%.1fV", remote_voltage);
            }
            else if (ognrelay_enable)
              snprintf(buf, sizeof(buf), "RELAY-Remote %.1fV", Battery_voltage());
            else
              snprintf(buf, sizeof(buf), "Single-Station");
            display.drawString(0, 54, buf);                                                                           
            Serial.println(buf);

            if (beers_show)
                oled_site = 4;
            else
                oled_site = 0;
           OLED_display();
           return;
        }  

        if (oled_site == 4)
        {
            String beer_supporters[] = {"manuel, guy,", "jozef, uwe,", "camille, caz"};
            display.clear();
            snprintf(buf, sizeof(buf), "Bier supporters");
            display.drawString(0, 0, buf);
            //Serial.println(buf);
            for (int i=0; i < sizeof(beer_supporters)/sizeof(beer_supporters[0]); i++) {
                snprintf(buf, sizeof(buf), "%s", beer_supporters[i]);
                display.drawString(0, (i + 1) * 9, buf);
            }
            snprintf(buf, sizeof(buf), "many thanks !!");
            display.drawString(0, 52, buf);

            OLED_draw_Bitmap(75, 0, 100, false);
            oled_site = 0;
            //OLED_display();
            return;
        }
    }
}
