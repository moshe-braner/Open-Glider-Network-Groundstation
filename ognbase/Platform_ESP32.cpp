/*
 * Platform_ESP32.cpp
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

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/adc_channel.h>   /* >>> added */
#include <soc/efuse_reg.h>
#include <rom/rtc.h>
#include <rom/spi_flash.h>
#include <flashchips.h>
#include <axp20x.h>
#define  XPOWERS_CHIP_AXP2102
#include <XPowersLib.h>
#if defined(T3S3)
// error: 'VSPI' was not declared
#else
#include <TFT_eSPI.h>
#endif

#include "Platform_ESP32.h"
#include "SoC.h"

#include "EEPROM.h"
#include "PNET.h"
#include "RF.h"
#include "WiFi.h"

#include "Log.h"
#include <ESP32Ping.h>

#include "Battery.h"

#include "global.h"

#include <battery.h>
//#include <U8x8lib.h>

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss  = SOC_GPIO_PIN_SS,
    .txe  = LMIC_UNUSED_PIN,
    .rxe  = LMIC_UNUSED_PIN,
    .rst  = SOC_GPIO_PIN_RST,
    .dio  = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_TXE,
    .tcxo = LMIC_UNUSED_PIN,
};

WebServer server(80);

AXP20X_Class axp_xxx;
XPowersPMU   axp_2xxx;

WiFiClient   client;
WiFiClient   zclient;

#if !defined(T3S3)
static TFT_eSPI*  tft  = NULL;
#endif

static int esp32_board = ESP32_DEVKIT; /* default */

static portMUX_TYPE GNSS_PPS_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE PMU_mutex      = portMUX_INITIALIZER_UNLOCKED;
volatile bool       PMU_Irq        = false;

static bool GPIO_21_22_are_busy = false;

bool ttgo_on_tbeam = false;    // indicates running the wrong binary

static union
{
    uint8_t efuse_mac[6];
    uint64_t chipmacid;
};

static bool     OLED_display_probe_status = false;
static bool     OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;
extern bool     loopTaskWDTEnabled;

/*const char* OLED_Protocol_ID[] = {
    [RF_PROTOCOL_LEGACY]    = "L",
    [RF_PROTOCOL_OGNTP]     = "O",
    [RF_PROTOCOL_P3I]       = "P",
    [RF_PROTOCOL_ADSB_1090] = "A",
    [RF_PROTOCOL_ADSB_UAT]  = "U",
    [RF_PROTOCOL_FANET]     = "F"
   };*/

const char SoftRF_text[]   = "SoftRF";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";

static void IRAM_ATTR ESP32_PMU_Interrupt_handler()
{
    portENTER_CRITICAL_ISR(&PMU_mutex);
    PMU_Irq = true;
    portEXIT_CRITICAL_ISR(&PMU_mutex);
}

static uint32_t ESP32_getFlashId()
{
    return g_rom_flashchip.device_id;
}

uint32_t flash_id_;

static void ESP32_setup()
{
#if !defined(SOFTRF_ADDRESS)

    esp_err_t ret         = ESP_OK;
    uint8_t   null_mac[6] = {0};

    ret = esp_efuse_mac_get_custom(efuse_mac);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
        /* If get custom base MAC address error, the application developer can decide what to do:
         * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
         * nothing.
         */

        ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
        chipmacid = ESP.getEfuseMac();
    }
    else if (memcmp(efuse_mac, null_mac, 6) == 0)
    {
        ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
        chipmacid = ESP.getEfuseMac();
    }
#endif /* SOFTRF_ADDRESS */

#if ESP32_DISABLE_BROWNOUT_DETECTOR
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

    if (psramFound())
    {
        uint32_t flash_id = ESP32_getFlashId();
        flash_id_ = flash_id;

        /*
         *    Board         |   Module   |  Flash memory IC
         *  ----------------+------------+--------------------
         *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
         *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
         *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
         *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
         *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
         *  TTGO T22 V11    |            | BOYA_BY25Q32AL
         *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
         *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
         *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
         *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
         *  Ai-T NodeMCU-S3 | ESP-S3-12K | GIGADEVICE_GD25Q64C
         *  TTGO S3 Core    |            | GIGADEVICE_GD25Q64C
         */

        switch(flash_id)
        {
#if defined(CONFIG_IDF_TARGET_ESP32)
        case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
          /* ESP32-WROVER module with ESP32-NODEMCU-ADAPTER */
          hw_info.model = SOFTRF_MODEL_STANDALONE;
          break;
        case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q128_V):
          hw_info.model = SOFTRF_MODEL_SKYWATCH;
          break;
        case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
        case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
        default:
          hw_info.model = SOFTRF_MODEL_PRIME_MK2;
#if defined(TBEAM)
          heap_caps_malloc_extmem_enable(8000);    // <<< try and keep the "dense" BEC table in regular RAM
#else
          ttgo_on_tbeam = true;
#endif
          break;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#if defined(T3S3)
        case 2113558:                              // = 0x204016, by experiment, GD25Q32 & similar?
          esp32_board = ESP32_TTGO_T3S3;
          hw_info.model = OGNBASE_MODEL_T3S3;
          heap_caps_malloc_extmem_enable(8000);    // <<< try and keep the "dense" BEC table in regular RAM
          break;
        default:
          esp32_board = ESP32_S3_DEVKIT;
          hw_info.model = OGNBASE_MODEL_ESP32S3_PSRAM;
          heap_caps_malloc_extmem_enable(8000);
          break;
#else
        default:
#error "compiling for ESP32-S3 but T3S3 not #defined!"
#endif
#else
        default:
#error "This ESP32 family build variant is not supported!"
#endif
          break;
        }

    }
    else   // no PSRAM
    {

#if defined(CONFIG_IDF_TARGET_ESP32)
        uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
        uint32_t pkg_ver  = chip_ver & 0x7;
        if (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4)
        {
            esp32_board    = ESP32_TTGO_V2_OLED;
            lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
            lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;
        }
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        esp32_board = ESP32_S3_DEVKIT;
        hw_info.model = OGNBASE_MODEL_ESP32S3_NO_PSRAM;
#endif
    }

    //ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);

    if (hw_info.model == SOFTRF_MODEL_SKYWATCH)
    {
        esp32_board = ESP32_TTGO_T_WATCH;

        Wire1.begin(SOC_GPIO_PIN_TWATCH_SEN_SDA , SOC_GPIO_PIN_TWATCH_SEN_SCL);
        Wire1.beginTransmission(AXP202_SLAVE_ADDRESS);
        bool has_axp202 = false;
        has_axp202 = (Wire1.endTransmission() == 0);
        if (has_axp202) {

          hw_info.pmu = PMU_AXP202;

          axp_xxx.begin(Wire1, AXP202_SLAVE_ADDRESS);

          axp_xxx.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);
          axp_xxx.adc1Enable(0xFF, AXP202_OFF);

          axp_xxx.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

          axp_xxx.setPowerOutPut(AXP202_LDO2, AXP202_ON); // BL
          axp_xxx.setPowerOutPut(AXP202_LDO3, AXP202_ON); // S76G (MCU + LoRa)
          axp_xxx.setLDO4Voltage(AXP202_LDO4_1800MV);
          axp_xxx.setPowerOutPut(AXP202_LDO4, AXP202_ON); // S76G (Sony GNSS)

          pinMode(SOC_GPIO_PIN_TWATCH_PMU_IRQ, INPUT_PULLUP);

          attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_PMU_IRQ),
                          ESP32_PMU_Interrupt_handler, FALLING);

          axp_xxx.adc1Enable(AXP202_BATT_VOL_ADC1, AXP202_ON);
          axp_xxx.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
          axp_xxx.clearIRQ();
        } else {
          WIRE_FINI(Wire1);
        }
    }
    else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
    {
        esp32_board = ESP32_TTGO_T_BEAM;

        bool has_axp192 = false;
        bool has_axp2101 = false;

        Wire1.begin(TTGO_V2_OLED_PIN_SDA , TTGO_V2_OLED_PIN_SCL);
        Wire1.beginTransmission(AXP192_SLAVE_ADDRESS);
        int Wire_Trans_rval = Wire1.endTransmission();

        bool has_axp = (Wire_Trans_rval == 0);
        //bool has_axp192 = false;
        if (has_axp)
          has_axp192 = (axp_xxx.begin(Wire1, AXP192_SLAVE_ADDRESS) == AXP_PASS);

        if (has_axp192) {

          hw_info.revision = 8;
          hw_info.pmu = PMU_AXP192;

          axp_xxx.setChargingTargetVoltage(AXP202_TARGET_VOL_4_2V);
          axp_xxx.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

          axp_xxx.setPowerOutPut(AXP192_LDO2,  AXP202_ON);
          axp_xxx.setPowerOutPut(AXP192_LDO3,  AXP202_ON);
          axp_xxx.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
          axp_xxx.setPowerOutPut(AXP192_DCDC2, AXP202_ON); // NC
          axp_xxx.setPowerOutPut(AXP192_EXTEN, AXP202_ON);

          axp_xxx.setDCDC1Voltage(3300); //       AXP192 power-on value: 3300
          axp_xxx.setLDO2Voltage (3300); // LoRa, AXP192 power-on value: 3300
          axp_xxx.setLDO3Voltage (3000); // GPS,  AXP192 power-on value: 2800

          pinMode(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, INPUT /* INPUT_PULLUP */);     // GPIO pin 35

          attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ),
                          ESP32_PMU_Interrupt_handler, FALLING);

          axp_xxx.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
          axp_xxx.clearIRQ();
        } else {
          has_axp2101 = has_axp && axp_2xxx.begin(Wire1,
                                                       AXP2101_SLAVE_ADDRESS,
                                                       TTGO_V2_OLED_PIN_SDA,
                                                       TTGO_V2_OLED_PIN_SCL);
          if (has_axp2101) {

            // Set the minimum common working voltage of the PMU VBUS input,
            // below this value will turn off the PMU
            axp_2xxx.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

            // Set the maximum current of the PMU VBUS input,
            // higher than this value will turn off the PMU
            axp_2xxx.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

            // DCDC1 1500~3400mV, IMAX=2A
            axp_2xxx.setDC1Voltage(3300); // ESP32,  AXP2101 power-on value: 3300

            // ALDO 500~3500V, 100mV/step, IMAX=300mA
            axp_2xxx.setButtonBatteryChargeVoltage(3100); // GNSS battery

            axp_2xxx.setALDO2Voltage(3300); // LoRa, AXP2101 power-on value: 2800
            axp_2xxx.setALDO3Voltage(3300); // GPS,  AXP2101 power-on value: 3300

            // axp_2xxx.enableDC1();
            axp_2xxx.enableButtonBatteryCharge();

            axp_2xxx.enableALDO2();
            axp_2xxx.enableALDO3();

            axp_2xxx.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
            axp_2xxx.setChargingLedMode(XPOWERS_CHG_LED_ON);

            pinMode(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, INPUT /* INPUT_PULLUP */);

            attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ),
                            ESP32_PMU_Interrupt_handler, FALLING);

            axp_2xxx.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
            axp_2xxx.clearIrqStatus();

            //These lines were added to lyusupov's version of SoftRF in Sep 2023:
            axp_2xxx.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
            axp_2xxx.disableTSPinMeasure();
            axp_2xxx.enableBattVoltageMeasure();

            axp_2xxx.enableIRQ(XPOWERS_AXP2101_PKEY_LONG_IRQ |
                               XPOWERS_AXP2101_PKEY_SHORT_IRQ);

            hw_info.revision = 12;
            hw_info.pmu = PMU_AXP2101;

          } else {  // neither type of AXP

            hw_info.revision = 2;
            // and leave Wire1 active for OLED
          }
        }

        // set up 2nd I2C port - in case OLED is actually there
        Wire.begin(SOC_GPIO_PIN_TBEAM_SDA, SOC_GPIO_PIN_TBEAM_SCL);

        pinMode(TBEAM_SOLAR_SHUNT, OUTPUT);
        digitalWrite(TBEAM_SOLAR_SHUNT, LOW);

    } else {
        // other than SOFTRF_MODEL_PRIME_MK2 - namely the TTGO Paxcounter or T3S3
        // initialize Wire1 for the OLED library
        // (since OLED library was changed to expect this to be done outside)
#if defined(T3S3)
        Wire1.begin(T3S3_OLED_PIN_SDA, T3S3_OLED_PIN_SCL);
        pinMode(T3S3_GREEN_LED, OUTPUT);
        digitalWrite(T3S3_GREEN_LED, LOW);
        pinMode(T3S3_SOLAR_SHUNT, OUTPUT);
        digitalWrite(T3S3_SOLAR_SHUNT, LOW);
#elif defined(TTGO)
        Wire1.begin(TTGO_V2_OLED_PIN_SDA, TTGO_V2_OLED_PIN_SCL);
        pinMode(PAXCOUNTER_GREEN_LED, OUTPUT);
        digitalWrite(PAXCOUNTER_GREEN_LED, LOW);
#endif
    }

#if defined(T3S3)
    lmic_pins.nss  = T3S3_SX12xx_SEL;
    lmic_pins.rst  = T3S3_SX12xx_RST;
    lmic_pins.busy = T3S3_SX1262_BUSY;   // <<< this is also T3S3_SX1276_IO2
#else
    lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
    lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;
#endif
}

// report detected hardware after Serial has been started
void ESP32_post_setup()
{
#if defined(TBEAM)
        Serial.print(F("INFO: TTGO T-Beam rev "));
        Serial.print(hw_info.revision);
        Serial.println(F(" is detected."));
#endif
#if defined(T3S3)
        if (hw_info.model == OGNBASE_MODEL_T3S3)
            Serial.println(F("T3S3 with PSRAM is detected"));
        else if (hw_info.model == OGNBASE_MODEL_ESP32S3_PSRAM)
            Serial.println(F("ESP32-S3, PSRAM enabled"));
        else if (hw_info.model == OGNBASE_MODEL_ESP32S3_NO_PSRAM)
            Serial.println(F("ESP32-S3, PSRAM not enabled"));
#endif
#if defined(TTGO)
        if (esp32_board == ESP32_TTGO_V2_OLED)
            Serial.println(F("TTGO paxcounter detected"));
        else
            Serial.println(F("TTGO paxcounter not detected"));
#endif
}

static void ESP32_loop()
{
    bool is_irq = false;
    bool down = false;

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:

      portENTER_CRITICAL_ISR(&PMU_mutex);
      is_irq = PMU_Irq;
      portEXIT_CRITICAL_ISR(&PMU_mutex);

      if (is_irq) {

        if (axp_xxx.readIRQ() == AXP_PASS) {

          if (axp_xxx.isPEKLongtPressIRQ()) {
            down = true;
          }
          if (axp_xxx.isPEKShortPressIRQ()) {

          }

          axp_xxx.clearIRQ();
        }

        portENTER_CRITICAL_ISR(&PMU_mutex);
        PMU_Irq = false;
        portEXIT_CRITICAL_ISR(&PMU_mutex);
      }

      break;

    case PMU_AXP2101:
      portENTER_CRITICAL_ISR(&PMU_mutex);
      is_irq = PMU_Irq;
      portEXIT_CRITICAL_ISR(&PMU_mutex);

      if (is_irq) {

        axp_2xxx.getIrqStatus();

        if (axp_2xxx.isPekeyLongPressIrq()) {
          down = true;
        }
        if (axp_2xxx.isPekeyShortPressIrq()) {

        }

        axp_2xxx.clearIrqStatus();

        portENTER_CRITICAL_ISR(&PMU_mutex);
        PMU_Irq = false;
        portEXIT_CRITICAL_ISR(&PMU_mutex);
      }

      break;

    case PMU_NONE:
    default:
      break;
    }

    if (down) {
        DebugLogWrite("button shutdown()");
        shutdown("  -- OFF -- ");
    }
}

#if defined(TBEAM)

#if 0
bool on_ext_power()
{
//>>> is this OK for the AXP2101?
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
        if (axp_xxx.getVbusVoltage() > 4000) {
          return true;
        } else {
          return false;
        }
        break;
    case PMU_NONE:
    default:
        return true;
        break;
    }
}
#endif

void turn_LED_on()
{
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
//>>> is this OK for the AXP2101?
        axp_xxx.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
        break;
    case PMU_NONE:
    default:
        break;
    }    
}

void turn_LED_off()
{
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
//>>> is this OK for the AXP2101?
        axp_xxx.setChgLEDMode(AXP20X_LED_OFF);
        break;
    case PMU_NONE:
    default:
        break;
    }    
}

void turn_GNSS_on()
{
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
        axp_xxx.setPowerOutPut(AXP192_LDO3, AXP202_ON);
        axp_xxx.setLDO3Voltage(3000);  // GPS,  AXP192 power-on value: 2800
        break;
    case PMU_AXP2101:
        axp_2xxx.setALDO3Voltage(3300); // GPS,  AXP2101 power-on value: 3300
        axp_2xxx.enableALDO3();
        break;
    case PMU_NONE:
    default:
        break;
    }    
}

void turn_GNSS_off()
{
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
        axp_xxx.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
        break;
    case PMU_AXP2101:
        axp_2xxx.disableALDO3();
        break;
    case PMU_NONE:
    default:
        break;
    }    
}

#endif   /* TBEAM */

#if defined(T3S3) || defined(TTGO)
void green_LED(bool state)
{
#if defined(T3S3)
    digitalWrite(T3S3_GREEN_LED, state? HIGH : LOW);
#elif defined(TTGO)
    digitalWrite(PAXCOUNTER_GREEN_LED, state? HIGH : LOW);
#endif
//Serial.print("green LED -> ");
//Serial.println(state);
}
#endif

static void ESP32_fini()
{
    SPI.end();

    esp_wifi_stop();
    esp_bt_controller_disable();

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    axp_xxx.setChgLEDMode(AXP20X_LED_OFF);

    axp_xxx.setPowerOutPut(AXP202_LDO2, AXP202_OFF); // BL
    axp_xxx.setPowerOutPut(AXP202_LDO4, AXP202_OFF); // S76G (Sony GNSS)
    axp_xxx.setPowerOutPut(AXP202_LDO3, AXP202_OFF); // S76G (MCU + LoRa)

    delay(20);

    esp_sleep_enable_ext1_wakeup(1ULL << SOC_GPIO_PIN_TWATCH_PMU_IRQ,
                                 ESP_EXT1_WAKEUP_ALL_LOW);

// >>> warning: 'ESP_EXT1_WAKEUP_ALL_LOW' is deprecated:
//     wakeup mode "ALL_LOW" is no longer supported after ESP32,
//     please use ESP_EXT1_WAKEUP_ANY_LOW instead

  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
      axp_xxx.setChgLEDMode(AXP20X_LED_OFF);
      delay(2000); /* Keep 'OFF' message on OLED for 2 seconds */
      axp_xxx.setPowerOutPut(AXP192_LDO2,  AXP202_OFF);
      axp_xxx.setPowerOutPut(AXP192_LDO3,  AXP202_OFF);
      axp_xxx.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
      axp_xxx.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
      axp_xxx.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

      delay(20);

      axp_xxx.shutdown();

      break;

    case PMU_AXP2101:
      axp_2xxx.setChargingLedMode(XPOWERS_CHG_LED_OFF);
      delay(2000); /* Keep 'OFF' message on OLED for 2 seconds */
      axp_2xxx.disableButtonBatteryCharge();
      axp_2xxx.disableALDO2();
      axp_2xxx.disableALDO3();

      delay(20);

      axp_2xxx.shutdown();

      break;

    case PMU_NONE:
    default:
      break;
    }

  }

  esp_deep_sleep_start();

}

static void ESP32_reset()
{
    ESP.restart();
}

static uint32_t ESP32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
    uint32_t id = (uint32_t) efuse_mac[5]        | ((uint32_t) efuse_mac[4] << 8) | \
                  ((uint32_t) efuse_mac[3] << 16) | ((uint32_t) efuse_mac[2] << 24);

    /* remap address to avoid overlapping with congested FLARM range */
    if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF))
        id += 0x100000;

    return id;
#else
    return SOFTRF_ADDRESS & 0xFFFFFFFFU;
#endif /* SOFTRF_ADDRESS */
}

static struct rst_info reset_info = {
    .reason = REASON_DEFAULT_RST,
};

static void * ESP32_getResetInfoPtr()
{
    switch (rtc_get_reset_reason(0))
    {
        case POWERON_RESET:
            reset_info.reason = REASON_DEFAULT_RST;
            break;
#if !defined(T3S3)
        case SW_RESET:
            reset_info.reason = REASON_SOFT_RESTART;
            break;
        case OWDT_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case SDIO_RESET:
            reset_info.reason = REASON_EXCEPTION_RST;
            break;
        case TGWDT_CPU_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case SW_CPU_RESET:
            reset_info.reason = REASON_SOFT_RESTART;
            break;
        case EXT_CPU_RESET:
            reset_info.reason = REASON_EXT_SYS_RST;
            break;
#endif
        case DEEPSLEEP_RESET:
            reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
            break;
        case TG0WDT_SYS_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case TG1WDT_SYS_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case RTCWDT_SYS_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case INTRUSION_RESET:
            reset_info.reason = REASON_EXCEPTION_RST;
            break;
        case RTCWDT_CPU_RESET:
            reset_info.reason = REASON_WDT_RST;
            break;
        case RTCWDT_BROWN_OUT_RESET:
            reset_info.reason = REASON_EXT_SYS_RST;
            break;
        case RTCWDT_RTC_RESET:
            /* Slow start of GD25LQ32 causes one read fault at boot time with current ESP-IDF */
            if (ESP32_getFlashId() == MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32))
                reset_info.reason = REASON_DEFAULT_RST;
            else
                reset_info.reason = REASON_WDT_RST;
            break;
        default:
            reset_info.reason = REASON_DEFAULT_RST;
    }

    return (void *) &reset_info;
}

static String ESP32_getResetInfo()
{
    switch (rtc_get_reset_reason(0))
    {
        case POWERON_RESET:
            return F("Vbat power on reset");
#if !defined(T3S3)
        case SW_RESET:
            return F("Software reset digital core");
        case OWDT_RESET:
            return F("Legacy watch dog reset digital core");
        case SDIO_RESET:
            return F("Reset by SLC module, reset digital core");
        case TGWDT_CPU_RESET:
            return F("Time Group reset CPU");
        case SW_CPU_RESET:
            return F("Software reset CPU");
        case EXT_CPU_RESET:
            return F("for APP CPU, reseted by PRO CPU");
#endif
        case DEEPSLEEP_RESET:
            return F("Deep Sleep reset digital core");
        case TG0WDT_SYS_RESET:
            return F("Timer Group0 Watch dog reset digital core");
        case TG1WDT_SYS_RESET:
            return F("Timer Group1 Watch dog reset digital core");
        case RTCWDT_SYS_RESET:
            return F("RTC Watch dog Reset digital core");
        case INTRUSION_RESET:
            return F("Instrusion tested to reset CPU");
        case RTCWDT_CPU_RESET:
            return F("RTC Watch dog Reset CPU");
        case RTCWDT_BROWN_OUT_RESET:
            return F("Reset when the vdd voltage is not stable");
        case RTCWDT_RTC_RESET:
            return F("RTC Watch dog reset digital core and rtc module");
        default:
            return F("No reset information available");
    }
}

static String ESP32_getResetReason()
{
    switch (rtc_get_reset_reason(0))
    {
        case POWERON_RESET:
            return F("POWERON_RESET");
#if !defined(T3S3)
        case SW_RESET:
            return F("SW_RESET");
        case OWDT_RESET:
            return F("OWDT_RESET");
        case SDIO_RESET:
            return F("SDIO_RESET");
        case TGWDT_CPU_RESET:
            return F("TGWDT_CPU_RESET");
        case SW_CPU_RESET:
            return F("SW_CPU_RESET");
        case EXT_CPU_RESET:
            return F("EXT_CPU_RESET");
#endif
        case DEEPSLEEP_RESET:
            return F("DEEPSLEEP_RESET");
        case TG0WDT_SYS_RESET:
            return F("TG0WDT_SYS_RESET");
        case TG1WDT_SYS_RESET:
            return F("TG1WDT_SYS_RESET");
        case RTCWDT_SYS_RESET:
            return F("RTCWDT_SYS_RESET");
        case INTRUSION_RESET:
            return F("INTRUSION_RESET");
        case RTCWDT_CPU_RESET:
            return F("RTCWDT_CPU_RESET");
        case RTCWDT_BROWN_OUT_RESET:
            return F("RTCWDT_BROWN_OUT_RESET");
        case RTCWDT_RTC_RESET:
            return F("RTCWDT_RTC_RESET");
        default:
            return F("DEFAULT_RESET");
    }
}

static uint32_t ESP32_getFreeHeap()
{
    return ESP.getFreeHeap();
}

static long ESP32_random(long howsmall, long howBig)
{
    return random(howsmall, howBig);
}

static uint32_t ESP32_maxSketchSpace()
{
    return 0x1E0000;
}

static const int8_t ESP32_dB_to_power_level[21] = {
    8,  /* 2    dB, #0 */
    8,  /* 2    dB, #1 */
    8,  /* 2    dB, #2 */
    8,  /* 2    dB, #3 */
    8,  /* 2    dB, #4 */
    20, /* 5    dB, #5 */
    20, /* 5    dB, #6 */
    28, /* 7    dB, #7 */
    28, /* 7    dB, #8 */
    34, /* 8.5  dB, #9 */
    34, /* 8.5  dB, #10 */
    44, /* 11   dB, #11 */
    44, /* 11   dB, #12 */
    52, /* 13   dB, #13 */
    52, /* 13   dB, #14 */
    60, /* 15   dB, #15 */
    60, /* 15   dB, #16 */
    68, /* 17   dB, #17 */
    74, /* 18.5 dB, #18 */
    76, /* 19   dB, #19 */
    78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
    if (dB > 20)
        dB = 20;

    if (dB < 0)
        dB = 0;

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static IPAddress ESP32_WiFi_get_broadcast()
{
    tcpip_adapter_ip_info_t info;
    IPAddress               broadcastIp;

    if (WiFi.getMode() == WIFI_STA)
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
    else
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &info);
    broadcastIp = ~info.netmask.addr | info.ip.addr;

    return broadcastIp;
}

static void ESP32_WiFi_transmit_UDP_debug(int port, byte* buf, size_t size)
{
    IPAddress    ClientIP;
    unsigned int localPort = 8888;

    WiFiMode_t mode = WiFi.getMode();
    int        i    = 0;

    switch (mode)
    {
        case WIFI_STA:
            ClientIP    = WiFi.gatewayIP();
            ClientIP[3] = 200;          
            Uni_Udp.begin(localPort);
            Uni_Udp.beginPacket(ClientIP, port);
            Uni_Udp.write(buf, size);
            Uni_Udp.endPacket();  
            break;
        case WIFI_AP:
            break;
        case WIFI_OFF:
        default:
            break;
    }
}

static void ESP32_WiFi_transmit_UDP(const char* host, int port, byte* buf, size_t size)
{
    WiFiMode_t mode = WiFi.getMode();
    int        i    = 0;

    switch (mode)
    {
        case WIFI_STA:
            Serial.println("Transmitting UDP Packet");
            Serial.println(host);
            Serial.println(port);
            Uni_Udp.beginPacket(host, port);
            Uni_Udp.write(buf, size);
            Uni_Udp.endPacket();

            break;
        case WIFI_AP:
        case WIFI_OFF:
        default:
            break;
    }
}

static int ESP32_WiFi_connect_TCP(const char* host, int port)
{
    if (client.connected())     // <<< added to avoid extra connections
        return 1;

    Serial.println("Reconnecting TCP...");

    yield();

    //disableLoopWDT();  // sometimes hangs, better to let watchdog reboot the system

    bool ret = Ping.ping(host, 2);
    yield();

    if (ret == false) {
        Serial.println("Ping host failed");
        return 0;
    }

    // Serial.println("... ping OK");

    ret = client.connect(host, port, 5000);
    yield();

    //enableLoopWDT();

    if (ret)
        return 1;

    Serial.println("client.connect failed");
    return 0;
}

static int ESP32_WiFi_disconnect_TCP()
{
    client.stop();
    return 0;       // without a return value this causes CRASHES!
}

static int ESP32_WiFi_transmit_TCP(String message)
{
    if (client.connected())
    {
        client.print(message);
        return 1;
    }
    return 0;
}

static int ESP32_WiFi_receive_TCP(char* RXbuffer, int RXbuffer_size)
{
    --RXbuffer_size;    // <<< room for trailing null char
    int i = 0;
    int n = 0;
    if (client.connected())
    {
        while (client.available()) {       // try and clear congestion
            if (i < RXbuffer_size)
                RXbuffer[i++] = client.read();
            else
                char c = client.read();    // discard excess chars
            if (++n > 1000)
                break;                     // handle the rest next time around the loop()
            if ((n & 0x80) == 0)
                yield();
        }
        if (n > 1000)
            Serial.println("TCP RX major congestion");
        else if (i == RXbuffer_size)
            Serial.println("TCP RX congestion");
        RXbuffer[i] = '\0';
        return i;
    }
    client.stop();
    return -1;
}

static int ESP32_WiFi_isconnected_TCP()
{
    return client.connected();
}

static int ESP32_WiFi_connect_TCP2(const char* host, int port)
{
    bool ret = Ping.ping(host, 2);

    if (ret)
    {
        if (!zclient.connect(host, port, 5000))
            return 0;
        return 1;
    }
    return 0;
}

static int ESP32_WiFi_disconnect_TCP2()
{
    zclient.stop();
    return 0;
}

static int ESP32_WiFi_transmit_TCP2(String message)
{
    if (zclient.connected())
    {
        zclient.print(message);
        return 0;
    }
    return 0;
}

static int ESP32_WiFi_receive_TCP2(char* RXbuffer, int RXbuffer_size)
{
    int i = 0;

    if (zclient.connected())
    {
        while (zclient.available() && i < RXbuffer_size - 1) {
            RXbuffer[i] = zclient.read();
            i++;
            RXbuffer[i] = '\0';
        }
        return i;
    }
    zclient.stop();
    return -1;
}

static int ESP32_WiFi_isconnected_TCP2()
{
    return zclient.connected();
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static bool ESP32_WiFi_hostname(String aHostname)
{
    return WiFi.setHostname(aHostname.c_str());
}

static int ESP32_WiFi_clients_count()
{
    WiFiMode_t mode = WiFi.getMode();

    switch (mode)
    {
        case WIFI_AP:
            wifi_sta_list_t stations;
            ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

            tcpip_adapter_sta_list_t infoList;
            ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

            return infoList.num;
        case WIFI_STA:
        default:
            return -1; /* error */
    }
}

static bool ESP32_EEPROM_begin(size_t size)
{
    return EEPROM.begin(size);
}

static void ESP32_SPI_begin()
{
#if defined(T3S3)
    //if (esp32_board == ESP32_TTGO_T3S3)
        SPI.begin(T3S3_SX12xx_SCK, T3S3_SX12xx_MISO, T3S3_SX12xx_MOSI, T3S3_SX12xx_SEL);
#else
    if (esp32_board != ESP32_TTGO_T_WATCH)
        SPI.begin(SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
    else
        SPI.begin(SOC_GPIO_PIN_TWATCH_TFT_SCK, SOC_GPIO_PIN_TWATCH_TFT_MISO,
                  SOC_GPIO_PIN_TWATCH_TFT_MOSI, -1);
#endif
}

static void ESP32_swSer_begin(unsigned long baud)
{
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
    {
        if (hw_info.revision >= 8)
            swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TBEAM_V08_RX, SOC_GPIO_PIN_TBEAM_V08_TX);
        else
            swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TBEAM_V05_RX, SOC_GPIO_PIN_TBEAM_V05_TX);
    }
    else
    {
        if (esp32_board == ESP32_TTGO_T_WATCH)
        {
            Serial.println(F("INFO: TTGO T-Watch is detected."));
            swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TWATCH_RX, SOC_GPIO_PIN_TWATCH_TX);
        }
        else if (esp32_board == ESP32_TTGO_V2_OLED)
        {
            /* 'Mini' (TTGO T3 + GNSS) */
            Serial.print(F("INFO: TTGO T3 rev. "));
            Serial.print(hw_info.revision);
            Serial.println(F(" is detected."));
            swSer.begin(baud, SERIAL_IN_BITS, TTGO_V2_PIN_GNSS_RX, TTGO_V2_PIN_GNSS_TX);
        }
        else
            /* open Standalone's GNSS port */
            swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
    }

    /* Default Rx buffer size (256 bytes) is sometimes not big enough */
    // swSer.setRxBufferSize(512);

    /* Need to gather some statistics on variety of flash IC usage */
    Serial.print(F("Flash memory ID: "));
    Serial.println(ESP32_getFlashId(), HEX);
}

static void ESP32_swSer_enableRx(boolean arg)
{}

static void ESP32_Battery_setup()
{
    if ((hw_info.model == SOFTRF_MODEL_PRIME_MK2 &&
         hw_info.revision >= 8)                     ||
        hw_info.model == SOFTRF_MODEL_SKYWATCH)
    {
        /* T-Beam v08 and T-Watch have PMU */

        /* TBD */
    }
    else
#if defined(T3S3)
        calibrate_voltage(ADC1_GPIO1_CHANNEL);
#else
        calibrate_voltage(hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
                          (esp32_board == ESP32_TTGO_V2_OLED && hw_info.revision == 16) ?
                          ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
#endif
}

static float ESP32_Battery_voltage()
{
    float voltage = 0.0;

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
      if (axp_xxx.isBatteryConnect()) {
        voltage = axp_xxx.getBattVoltage();
      }
      break;

    case PMU_AXP2101:
      if (axp_2xxx.isBatteryConnect()) {
        voltage = axp_2xxx.getBattVoltage();
      }
      break;

    case PMU_NONE:
    default:
      voltage = (float) read_voltage();

      /* T-Beam v02-v07 and T3 V2.1.6 have voltage divider 100k/100k on board */
      if (hw_info.model == SOFTRF_MODEL_PRIME_MK2   ||
         (esp32_board   == ESP32_TTGO_V2_OLED && hw_info.revision == 16)  ||
          esp32_board   == ESP32_TTGO_T3S3 || esp32_board == ESP32_S3_DEVKIT) {
        voltage += voltage;
      }
      break;
    }

    return voltage * 0.001f;
}

float ESP32_VbusVoltage()
{
    float voltage;

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP202:
      voltage = 0.001f * (float)axp_xxx.getVbusVoltage();
      break;
    case PMU_AXP2101:
      voltage = 0.001f * (float)axp_2xxx.getVbusVoltage();
      break;
    case PMU_NONE:
    default:
      voltage = 0.0;
      break;
    }
    return voltage;
}

#if defined(TBEAM)
void charge_limit()
{
    if (ognvoltage_limit && hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
      if (hw_info.pmu == PMU_AXP2101)
        axp_2xxx.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V);
      if (hw_info.pmu == PMU_AXP192)
        axp_xxx.setChargingTargetVoltage(AXP202_TARGET_VOL_4_1V);     // 4V not available
    }
}

void solar_shunt()
{
    if (ognvoltage_limit && hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
        //if (hw_info.pmu == PMU_NONE || hw_info.pmu == PMU_AXP192) {
        // do this for AXP2101 too, in case of external battery
            if (ESP32_Battery_voltage() > 4.0)
                digitalWrite(TBEAM_SOLAR_SHUNT, HIGH);
            else
                digitalWrite(TBEAM_SOLAR_SHUNT, LOW);
        //}
    }
}
#endif

#if defined(T3S3)
void solar_shunt()
{
    if (ognvoltage_limit && hw_info.model == OGNBASE_MODEL_T3S3) {
        if (ESP32_Battery_voltage() > 4.0)
            digitalWrite(T3S3_SOLAR_SHUNT, HIGH);
        else
            digitalWrite(T3S3_SOLAR_SHUNT, LOW);
    }
}
#endif

static void IRAM_ATTR ESP32_GNSS_PPS_Interrupt_handler()
{
    portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
    //if (millis() > PPS_TimeMarker + 600)
        PPS_TimeMarker = millis();  /* millis() has IRAM_ATTR */
    portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
}

static unsigned long ESP32_get_PPS_TimeMarker()
{
    unsigned long rval;
    portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
    rval = PPS_TimeMarker;
    portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
    return rval;
}

static void ESP32_UATSerial_begin(unsigned long baud)
{
    /* open Standalone's I2C/UATSerial port */
    UATSerial.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_CE, SOC_GPIO_PIN_PWR);
}

static void ESP32_UATSerial_updateBaudRate(unsigned long baud)
{
    UATSerial.updateBaudRate(baud);
}

static void ESP32_UATModule_restart()
{
    digitalWrite(SOC_GPIO_PIN_TXE, LOW);
    pinMode(SOC_GPIO_PIN_TXE, OUTPUT);

    delay(100);

    digitalWrite(SOC_GPIO_PIN_TXE, HIGH);

    delay(100);

    pinMode(SOC_GPIO_PIN_TXE, INPUT);
}

static void ESP32_WDT_setup()
{
    enableLoopWDT();
}

static void ESP32_WDT_fini()
{
    disableLoopWDT();
}

int tbeam_button = SOC_UNUSED_PIN;

static void ESP32_Button_setup()
{
#if defined(TBEAM)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (hw_info.revision >= 8 && hw_info.revision <= 12)
        tbeam_button = SOC_GPIO_PIN_TBEAM_V08_BUTTON;   // 38
    else if (hw_info.revision < 8)
        tbeam_button = SOC_GPIO_PIN_TBEAM_V05_BUTTON;   // 39
  }
  if (tbeam_button != SOC_UNUSED_PIN)
      pinMode(tbeam_button, INPUT);
#endif
}

static void ESP32_Button_loop()
{
    /* TODO */
}

static void ESP32_Button_fini()
{
    /* TODO */
}

const SoC_ops_t ESP32_ops = {
    SOC_ESP32,
    "ESP32",
    ESP32_setup,
    ESP32_loop,
    ESP32_fini,
    ESP32_reset,
    ESP32_getChipId,
    ESP32_getResetInfoPtr,
    ESP32_getResetInfo,
    ESP32_getResetReason,
    ESP32_getFreeHeap,
    ESP32_random,
    ESP32_maxSketchSpace,
    ESP32_WiFi_setOutputPower,
    ESP32_WiFi_transmit_UDP,
    ESP32_WiFi_transmit_UDP_debug,
    ESP32_WiFi_connect_TCP,
    ESP32_WiFi_disconnect_TCP,
    ESP32_WiFi_transmit_TCP,
    ESP32_WiFi_receive_TCP,
    ESP32_WiFi_isconnected_TCP,
    ESP32_WiFi_connect_TCP2,
    ESP32_WiFi_disconnect_TCP2,
    ESP32_WiFi_transmit_TCP2,
    ESP32_WiFi_receive_TCP2,
    ESP32_WiFi_isconnected_TCP2,    
    ESP32_WiFiUDP_stopAll,
    ESP32_WiFi_hostname,
    ESP32_WiFi_clients_count,
    ESP32_EEPROM_begin,
    ESP32_SPI_begin,
    ESP32_swSer_begin,
    ESP32_swSer_enableRx,
    ESP32_Battery_setup,
    ESP32_Battery_voltage,
    ESP32_GNSS_PPS_Interrupt_handler,
    ESP32_get_PPS_TimeMarker,
    ESP32_UATSerial_begin,
    ESP32_UATModule_restart,
    ESP32_WDT_setup,
    ESP32_WDT_fini,
    ESP32_Button_setup,
    ESP32_Button_loop,
    ESP32_Button_fini
};

#endif /* ESP32 */
