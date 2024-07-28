/*
 * SoftRF.h
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

#ifndef SOFTRF_H
#define SOFTRF_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <TimeLib.h>
#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#if defined(TBEAM) || defined(TTGO)
#error ESP32S3 target but TBEAM or TTGO in build_opt.h
#endif
#endif
#if defined(CONFIG_IDF_TARGET_ESP32)
#if defined(T3S3)
#error ESP32 target but T3S3 in build_opt.h
#endif
#endif

#include "version.h"

#define SOFTRF_FIRMWARE_VERSION "MB145"   // _VERSION
#define SOFTRF_IDENT            "OGNB-"
#define OGNBASE_HTML_VERSION    "MB107"

#define ENTRY_EXPIRATION_TIME   41 /* seconds */
#define LED_EXPIRATION_TIME     5  /* seconds */
#define EXPORT_EXPIRATION_TIME  63  /* seconds */

/*
 * If you need for SoftRF to operate in wireless
 * client mode - specify your local AP's SSID/PSK:
 *
 * #define MY_ACCESSPOINT_SSID "My_AP_SSID"
 * #define MY_ACCESSPOINT_PSK  "My_AP_PSK"
 *
 * If SoftRF's built-in AP is not stable enough for you, consider
 * to use "reverse" operation when your smartphone is acting
 * as an AP for the SoftRF unit as a client:
 *
 * #define MY_ACCESSPOINT_SSID "AndroidAP"
 * #define MY_ACCESSPOINT_PSK  "12345678"
 */

// Default mode is AP with
// SSID: SoftRF-XXXXXX
// KEY:  12345678
// IP: 192.168.1.1
// NETMASK: 255.255.255.0

#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define GDL90_DST_PORT    4000
#define D1090_DST_PORT    4001
#define NMEA_UDP_PORT     10110
#define NMEA_TCP_PORT     2000

/*
 * Serial I/O default values.
 * Can be overridden by platfrom-specific code.
 */
#if !defined(SERIAL_IN_BR)
/*
 * 9600 is default value of NMEA baud rate
 * for most of GNSS modules
 * being used in SoftRF project
 */
#define SERIAL_IN_BR      9600
#endif
#if !defined(SERIAL_IN_BITS)
#define SERIAL_IN_BITS    SERIAL_8N1
#endif

/*
 * 38400 is known as maximum baud rate
 * that HC-05 Bluetooth module
 * can handle without symbols loss.
 *
 * Applicable for Standalone Edition. Inherited by most of other SoftRF platforms.
 */
#define STD_OUT_BR        38400
#define STD_OUT_BITS      SERIAL_8N1

#if !defined(SERIAL_OUT_BR)
#define SERIAL_OUT_BR     STD_OUT_BR
#endif
#if !defined(SERIAL_OUT_BITS)
#define SERIAL_OUT_BITS   STD_OUT_BITS
#endif

#define UAT_RECEIVER_BR   2000000

#if defined(PREMIUM_PACKAGE) && !defined(RASPBERRY_PI)
#define ENABLE_AHRS
#endif /* PREMIUM_PACKAGE */


/* data structure for keeping track of how many unique aircraft IDs were seen */
typedef struct id_list_entry {
    uint32_t addr;
    time_t timestamp;
    id_list_entry *prev;
    id_list_entry *next;
    bool invisible;
} id_list_entry_t;

typedef struct UFO
{
    uint8_t raw[36];

    time_t timestamp;
    time_t timereported;
    uint8_t hour, minute, second;
    bool waiting;   /* waiting to be relayed, or to be sent to OGN */

    uint32_t addr;
    uint8_t addr_type;
    uint8_t msg_type;
    uint8_t protocol;
    uint8_t aircraft_type;
    float latitude;
    float longitude;
    float altitude;
    float pressure_altitude;
    float course;         /* CoG */
    float speed;          /* ground speed in knots */
    float turnrate;

    /* previous data for comparison */
    time_t prevtime;
    float prevlat;
    float prevlon;
    float prevalt;

    float vs;     /* feet per minute */

    bool stealth;
    bool no_track;

    int8_t ns[4];
    int8_t ew[4];

    float geoid_separation; /* metres */
    uint16_t hdop;          /* cm */
    int8_t rssi;
    int8_t snr;             // computed from RSSIs during and after reception
    int8_t bec;             // number of bits corrected

    /* 'legacy' specific data */
    uint8_t gpsA;
    uint8_t gpsB;
    int8_t alarm_level;
    float distance;
    float bearing;
    bool airborne;
    bool relayed;

    /* ADS-B (ES, UAT, GDL90) specific data */
    uint8_t callsign[8];

    /* where it is in list of traffic seen */
    id_list_entry_t *listed;
} ufo_t;

typedef struct hardware_info
{
    byte model;
    byte revision;
    byte soc;
    byte pmu;
    byte rf;
    byte gnss;
    byte baro;
    byte display;
#if defined(ENABLE_AHRS)
    byte ahrs;
#endif /* ENABLE_AHRS */
} hardware_info_t;

enum
{
    SOFTRF_MODE_NORMAL,
    SOFTRF_MODE_GROUND,
    SOFTRF_MODE_WATCHOUT,
    SOFTRF_MODE_BRIDGE,
    SOFTRF_MODE_RELAY,
    SOFTRF_MODE_TXRX_TEST,
    SOFTRF_MODE_LOOPBACK,
    SOFTRF_MODE_UAV,
    SOFTRF_MODE_RECEIVER
};

enum
{
    SOFTRF_MODEL_STANDALONE,
    SOFTRF_MODEL_PRIME,
    SOFTRF_MODEL_UAV,
    SOFTRF_MODEL_PRIME_MK2,
    SOFTRF_MODEL_RASPBERRY,
    SOFTRF_MODEL_UAT,
    SOFTRF_MODEL_SKYVIEW,
    SOFTRF_MODEL_RETRO,
    SOFTRF_MODEL_SKYWATCH,
    SOFTRF_MODEL_DONGLE,
    SOFTRF_MODEL_MULTI,
    SOFTRF_MODEL_UNI,
    SOFTRF_MODEL_MINI,
    OGNBASE_MODEL_T3S3,
    OGNBASE_MODEL_ESP32S3_PSRAM,
    OGNBASE_MODEL_ESP32S3_NO_PSRAM
};

extern bool time_synced;

extern ufo_t           ThisAircraft;
extern hardware_info_t hw_info;
extern const float     txrx_test_positions[90][2] PROGMEM;

extern void shutdown(const char *);

#define TXRX_TEST_NUM_POSITIONS (sizeof(txrx_test_positions) / sizeof(float) / 2)
#define TXRX_TEST_ALTITUDE    438.0
#define TXRX_TEST_COURSE      280.0
#define TXRX_TEST_SPEED       50.0
#define TXRX_TEST_VS          -300.0

//#define ENABLE_TTN
//#define ENABLE_BT_VOICE
//#define TEST_PAW_ON_NICERF_SV610_FW466
#define  DO_GDL90_FF_EXT

#define UDP_LOGGER 0

#define FILE_LOGGER 1

// FILE_LOGGER only outputs a few messages to debuglog.txt (if ogn_debug is enabled)
// LOGGER_IS_ENABLED redirects StdOut to another log file

#define LOGGER_IS_ENABLED 0

#if LOGGER_IS_ENABLED
#define StdOut  LogFile
#else
#define StdOut  Serial
#endif /* LOGGER_IS_ENABLED */

#endif /* SOFTRF_H */
