/*
 * Platform_ESP32.h
 * Copyright (C) 2018-2020 Linar Yusupov
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
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#include "sdkconfig.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiClient.h>
#include <SPIFFS.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
/* For OGNbase allow tracking more objects - they get prioritized */
#define MAX_TRACKING_OBJECTS    30

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_STANDALONE

#define SerialOutput            Serial
#define SoftwareSerial          HardwareSerial
#define swSer                   Serial1
#define UATSerial               Serial2
#define EEPROM_commit()         EEPROM.commit()

// this may allow USB logging in non-CDC mode
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#if ARDUINO_USB_CDC_ON_BOOT
// leave Serial as is
#else
#define Serial                  USBSerial
#endif
#endif

#define isValidFix()            isValidGNSSFix()

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=4
#define WIRE_FINI(bus)          { bus.end(); }
#else
#define WIRE_FINI(bus)          { } /* AC 1.0.x has no Wire.end() */
#endif

/* Adafruit_NeoPixel still has "flickering" issue of ESP32 caused by 1 ms scheduler */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define USE_ADAFRUIT_NEO_LIBRARY
#else

/*
 * NeoPixelBus is already "flickering-free" on ESP32 (with I2S)
 * but the "Core" needs update onto the most recent one
 */
#define USE_NEOPIXELBUS_LIBRARY
#endif

#if defined(USE_NEOPIXELBUS_LIBRARY)
#include <NeoPixelBus.h>

#define uni_begin()             strip.Begin()
#define uni_show()              strip.Show()
#define uni_setPixelColor(i, c) strip.SetPixelColor(i, c)
#define uni_numPixels()         strip.PixelCount()
#define uni_Color(r, g, b)        RgbColor(r, g, b)
#define color_t                 RgbColor

extern NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip;
#else /* USE_ADAFRUIT_NEO_LIBRARY */
#include <Adafruit_NeoPixel.h>

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r, g, b)        strip.Color(r, g, b)
#define color_t                 uint32_t

extern Adafruit_NeoPixel strip;
#endif /* USE_NEOPIXELBUS_LIBRARY */

#define LEDC_CHANNEL_BUZZER     0
#define BACKLIGHT_CHANNEL       ((uint8_t)1)
#define LEDC_RESOLUTION_BUZZER  8

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX    23
#define SOC_GPIO_PIN_GNSS_TX    12
#define SOC_GPIO_PIN_LED        25
#define SOC_GPIO_PIN_BUZZER     13
#define SOC_GPIO_PIN_BATTERY    36

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN

#define SOC_GPIO_PIN_STATUS   (hw_info.model != SOFTRF_MODEL_PRIME_MK2 ? \
                               SOC_UNUSED_PIN :                        \
                               (hw_info.revision == 2 ?                \
                                SOC_GPIO_PIN_TBEAM_LED_V02 :          \
                                (hw_info.revision == 5 ?              \
                                 SOC_GPIO_PIN_TBEAM_LED_V05 :        \
                                 ((hw_info.revision == 11 || hw_info.revision == 12) ?     \
                                  SOC_GPIO_PIN_TBEAM_LED_V11 :      \
                                  SOC_UNUSED_PIN))))

#define SOC_GPIO_PIN_GNSS_PPS (hw_info.model != SOFTRF_MODEL_PRIME_MK2 ? \
                               SOC_UNUSED_PIN :                        \
                               ((hw_info.revision >= 8 && hw_info.revision <= 12) ?                \
                                SOC_GPIO_PIN_TBEAM_V08_PPS :          \
                                SOC_UNUSED_PIN))

/* SPI (does match Heltec & TTGO LoRa32 pins mapping) */
#define SOC_GPIO_PIN_MOSI       27
#define SOC_GPIO_PIN_MISO       19
#define SOC_GPIO_PIN_SCK        5
#define SOC_GPIO_PIN_SS         18

/* NRF905 */
#define SOC_GPIO_PIN_TXE        26
#define SOC_GPIO_PIN_CE         2
#define SOC_GPIO_PIN_PWR        14

/* SX1276 [RFM95W] (does match Heltec & TTGO LoRa32 pins mapping) */
#define SOC_GPIO_PIN_RST        14
#define SOC_GPIO_PIN_DIO0       26
#define SOC_GPIO_PIN_SDA        14
#define SOC_GPIO_PIN_SCL        2

/* TTGO T-BEAM section */
// GPS module
#define SOC_GPIO_PIN_TBEAM_V05_RX       12
#define SOC_GPIO_PIN_TBEAM_V05_TX       15
#define SOC_GPIO_PIN_TBEAM_V08_RX       34
#define SOC_GPIO_PIN_TBEAM_V08_TX       12
#define SOC_GPIO_PIN_TBEAM_V08_PPS      37
// battery voltage
#define SOC_GPIO_PIN_TBEAM_V05_BATTERY  35
// status LED
#define SOC_GPIO_PIN_TBEAM_LED_V02      21
#define SOC_GPIO_PIN_TBEAM_LED_V05      14
#define SOC_GPIO_PIN_TBEAM_LED_V11      4
// button
#define SOC_GPIO_PIN_TBEAM_V05_BUTTON   39
#define SOC_GPIO_PIN_TBEAM_V08_BUTTON   38
// PMU
#define SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ  35
// SX1276 RESET
#define SOC_GPIO_PIN_TBEAM_RF_RST_V02   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TBEAM_RF_RST_V05   23
// SX1262 BUSY
#define SOC_GPIO_PIN_TBEAM_RF_BUSY_V08  32
// 1st I2C bus on the T-Beam
#define SOC_GPIO_PIN_TBEAM_SDA          13
#define SOC_GPIO_PIN_TBEAM_SCL          2
// pin to use for solar shunt
#define TBEAM_SOLAR_SHUNT               25

/* TTGO T-Watch section */
// GPS module
#define SOC_GPIO_PIN_TWATCH_RX          34
#define SOC_GPIO_PIN_TWATCH_TX          33
// button
#define SOC_GPIO_PIN_TWATCH_BUTTON      36

// PMU
#define SOC_GPIO_PIN_TWATCH_PMU_IRQ     35

// TFT
#define SOC_GPIO_PIN_TWATCH_TFT_MOSI    19
#define SOC_GPIO_PIN_TWATCH_TFT_MISO    SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TWATCH_TFT_SCK     18
#define SOC_GPIO_PIN_TWATCH_TFT_SS      5
#define SOC_GPIO_PIN_TWATCH_TFT_DC      27
#define SOC_GPIO_PIN_TWATCH_TFT_RST     SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TWATCH_TFT_BL      12

#define LV_HOR_RES                      (240) //Horizontal
#define LV_VER_RES                      (240) //vertical

// 1st I2C bus on the T-Watch
#define SOC_GPIO_PIN_TWATCH_SEN_SDA     21
#define SOC_GPIO_PIN_TWATCH_SEN_SCL     22

#define SSD1306_OLED_I2C_ADDR   0x3C

// Hardware pin definitions for TTGO V2 Board with OLED SSD1306 0,96" I2C Display
#define TTGO_V2_OLED_PIN_RST    U8X8_PIN_NONE // connected to CPU RST/EN
#define TTGO_V2_OLED_PIN_SDA    21
#define TTGO_V2_OLED_PIN_SCL    22
#define TTGO_V2_PIN_GNSS_RX     34
#define TTGO_V2_PIN_GNSS_TX     12
#define TTGO_V2_PIN_GNSS_PPS    39

// Hardware pin definitions for Heltec and TTGO-V1 LoRa-32 Boards with OLED SSD1306 I2C Display
#define HELTEC_OLED_PIN_RST     U8X8_PIN_NONE // 16
#define HELTEC_OLED_PIN_SDA     4
#define HELTEC_OLED_PIN_SCL     15

// Hardware pin definitions for TTGO T3S3 LORA Board with OLED
#define T3S3_BATTERY        1   // battery voltage (ADC)
#define T3S3_USB_DP        20
#define T3S3_USB_DN        19
#define T3S3_OLED_PIN_RST  U8X8_PIN_NONE
#define T3S3_OLED_PIN_SDA  18   // 1st I2C bus
#define T3S3_OLED_PIN_SCL  17
#define T3S3_GREEN_LED     37   // active high
#define T3S3_SD_MISO        2
#define T3S3_SD_MOSI       11
#define T3S3_SD_CLK        14
#define T3S3_SD_CS         13
#define T3S3_SD_SD1         4
#define T3S3_SD_SD2        12
#define T3S3_SX1276_IO0     9
#define T3S3_SX1276_IO1    33
#define T3S3_SX1276_IO2    34
#define T3S3_SX1276_IO3    21
#define T3S3_SX1276_IO4    10
#define T3S3_SX1276_IO5    36
#define T3S3_SX12xx_RST     8
#define T3S3_SX12xx_SEL     7
#define T3S3_SX12xx_SCK     5
#define T3S3_SX12xx_MOSI    6
#define T3S3_SX12xx_MISO    3
#define T3S3_SX1262_IO1    33
#define T3S3_SX1262_BUSY   34
#define T3S3_SOLAR_SHUNT   16

#define PAXCOUNTER_GREEN_LED     25   // active high

extern WebServer server;

#if defined(TBEAM)
bool on_ext_power(void);
extern int tbeam_button;

void turn_LED_on(void);
void turn_LED_off(void);

void turn_GNSS_on(void);
void turn_GNSS_off(void);

void charge_limit(void);
void solar_shunt(void);
#endif

#if defined(TTGO)
void green_LED(bool state);
#endif

extern bool ttgo_on_tbeam;

#if defined(T3S3)
void green_LED(bool state);
void solar_shunt(void);
#endif

float ESP32_VbusVoltage();   // should fold into SoC

void ESP32_post_setup(void);

enum rst_reason
{
    REASON_DEFAULT_RST = 0,      /* normal startup by power on */
    REASON_WDT_RST = 1,          /* hardware watch dog reset */
    REASON_EXCEPTION_RST = 2,    /* exception reset, GPIO status won't change */
    REASON_SOFT_WDT_RST = 3,     /* software watch dog reset, GPIO status won't change */
    REASON_SOFT_RESTART = 4,     /* software restart ,system_restart , GPIO status won't change */
    REASON_DEEP_SLEEP_AWAKE = 5, /* wake up from deep-sleep */
    REASON_EXT_SYS_RST = 6       /* external system reset */
};

enum esp32_board_id {
  ESP32_DEVKIT,
  ESP32_TTGO_V2_OLED,
  ESP32_HELTEC_OLED,
  ESP32_TTGO_T_BEAM,
  ESP32_TTGO_T_BEAM_SUPREME,
  ESP32_TTGO_T_WATCH,
  ESP32_S2_T8_V1_1,
  ESP32_S3_DEVKIT,
  ESP32_TTGO_T3S3
};

struct rst_info
{
    uint32_t reason;
    uint32_t exccause;
    uint32_t epc1;
    uint32_t epc2;
    uint32_t epc3;
    uint32_t excvaddr;
    uint32_t depc;
};

/* Boya Microelectronics Inc. */
#define BOYA_ID                 0x68
#define BOYA_BY25Q32AL          0x4016

#define MakeFlashId(v, d)        ((v << 16) | d)

/* Disable brownout detection (avoid unexpected reset on some boards) */
#define ESP32_DISABLE_BROWNOUT_DETECTOR 0

#define NMEA_TCP_SERVICE
#define USE_NMEALIB
//#define USE_NMEA_CFG
//#define USE_AT6558_SETUP
#define USE_BASICMAC

#define EXCLUDE_CC13XX

#define POWER_SAVING_WIFI_TIMEOUT 1200
                 /* 1200 seconds = 20 minutes */  // was 600000UL ms.

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
