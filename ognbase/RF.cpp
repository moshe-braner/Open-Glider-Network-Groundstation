/*
 * RF.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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
#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "RF.h"
#include "Time.h"
#include "Protocol_Legacy.h"
#include "Protocol_OGNTP.h"
#include "Protocol_P3I.h"
#include "Protocol_FANET.h"
#include "Protocol_UAT978.h"
#include "SoC.h"
#include "EEPROM.h"
#include "Web.h"
#include "global.h"
#include "Log.h"
#include "GNSS.h"
#include "PNET.h"
#include <fec.h>

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;
uint32_t txms = 0;

uint32_t packets_failed_crc = 0;
uint32_t packets_corrected = 0;

int32_t noise_data[65];
uint16_t noise_count[65];

int32_t remote_noise_data[65];
uint16_t remote_noise_count[65];

uint32_t RF_last_crc = 0;
int8_t RF_last_rssi = 0;
int8_t RF_last_rx_chan = 0;
int8_t RF_last_idle_rssi = 0;
bool RF_have_noise_sample = false;
int8_t avg_idle_rssi = -100;    // running average used for SNR calc
int8_t RF_last_snr  = 0;
int8_t RF_last_bec  = 0;

FreqPlan    RF_FreqPlan;
static bool RF_ready = false;
time_t RF_time;

static size_t RF_tx_size    = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

size_t (* protocol_encode)(void *, ufo_t *);
bool   (* protocol_decode)(void *, ufo_t *, ufo_t *);

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;   // default but we won't use it

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

static bool sx1276_probe(void);

static bool sx1262_probe(void);

static void sx12xx_setup(void);

static void sx12xx_channel(uint8_t);
static bool sx12xx_receive(void);

static void sx12xx_transmit(void);

static void sx12xx_shutdown(void);

/* typedef struct rfchip_ops_struct
{
  byte type;
  const char name[8];
  bool (* probe)();
  void (* setup)();
  void (* channel)(uint8_t);
  bool (* receive)();
  void (* transmit)();
  void (* shutdown)();
} rfchip_ops_t; */

void null_void() { };
void null_void1(uint8_t c) { };
bool null_bool() { return false; };

const rfchip_ops_t null_ops = {
    RF_IC_NONE,
    "NONE",
    null_bool,
    null_void,
    null_void1,
    null_bool,
    null_void,
    null_void
};

const rfchip_ops_t sx1276_ops = {
    RF_IC_SX1276,
    "SX1276",
    sx1276_probe,
    sx12xx_setup,
    sx12xx_channel,
    sx12xx_receive,
    sx12xx_transmit,
    sx12xx_shutdown
};

const rfchip_ops_t sx1262_ops = {
    RF_IC_SX1262,
    "SX1262",
    sx1262_probe,
    sx12xx_setup,
    sx12xx_channel,
    sx12xx_receive,
    sx12xx_transmit,
    sx12xx_shutdown
};

String Bin2Hex(byte* buffer, size_t size)
{
    String str = "";
    for (int i=0; i < size; i++) {
        byte c = buffer[i];
        str += (c < 0x10 ? "0" : "") + String(c, HEX);
    }
    return str;
}

uint8_t parity(uint32_t x)
{
    uint8_t parity=0;
    while (x > 0) {
        if (x & 0x1)
            parity++;
        x >>= 1;
    }
    return parity % 2;
}

byte RF_setup(void)
{
    for (int i=0; i<65; i++) {
        noise_count[i] = 0;
        noise_data[i] = 0;
        remote_noise_count[i] = 0;
        remote_noise_data[i] = 0;
    }

    if (rf_chip == NULL)
    {
        if (sx1276_ops.probe())
        {
            rf_chip = &sx1276_ops;
            SX12XX_LL = &sx127x_ll_ops;
        }
        else if (sx1262_ops.probe())
        {
            rf_chip   = &sx1262_ops;
            SX12XX_LL = &sx126x_ll_ops;
        }

        if (rf_chip && rf_chip->name)
        {
            Serial.print(rf_chip->name);
            Serial.println(F(" RFIC is detected."));
        }
        else
        {
            Serial.println(F("WARNING! None of supported RFICs is detected!"));
            rf_chip   = &null_ops;
        }
    }

    RF_FreqPlan.setPlan(ogn_band);

    if (rf_chip)
    {
        rf_chip->setup();

        const rf_proto_desc_t *p;

        switch (ogn_protocol_1)
        {
          case RF_PROTOCOL_OGNTP:     p = &ogntp_proto_desc;  break;
          case RF_PROTOCOL_P3I:       p = &p3i_proto_desc;    break;
          case RF_PROTOCOL_FANET:     p = &fanet_proto_desc;  break;
          case RF_PROTOCOL_ADSB_UAT:  p = &uat978_proto_desc; break;
          case RF_PROTOCOL_LEGACY:
          default:                    p = &legacy_proto_desc; break;
        }

// >>> the following includes hardcoded constants for Legacy protocol:

// >>> the new time slots code currently does NOT use these values.

        RF_timing         = p->tm_type;

        ts                = &Time_Slots;
        ts->air_time      = p->air_time;
        ts->interval_min  = p->tx_interval_min;
        ts->interval_max  = p->tx_interval_max;
        ts->interval_mid  = (p->tx_interval_max + p->tx_interval_min) / 2;
        ts->s0.begin      = 400;   // p->slot0.begin;
        ts->s1.begin      = 800;   // p->slot1.begin;
        ts->s0.duration   = 400;   // p->slot0.end - p->slot0.begin;
        ts->s1.duration   = 400;   // p->slot1.end - p->slot1.begin;

        uint16_t duration = ts->s0.duration + ts->s1.duration;
        ts->adj = duration > ts->interval_mid ? 0 : (ts->interval_mid - duration) / 2;

        crc_setup();  // in lib_crc_bec - sets up lookup tables

        return rf_chip->type;
    }
    else
        return RF_IC_NONE;
}

uint8_t RF_current_slot = 0;
uint8_t txchan = 0;
uint8_t rxchan = 0;
uint32_t RF_OK_until = 0;
uint32_t TxTimeMarker = 0;
uint32_t TxEndMarker  = 0;

void RF_loop()
{
    if (!RF_ready)
    {
        if (RF_FreqPlan.Plan == RF_BAND_AUTO)
        {
            if (ThisAircraft.latitude || ThisAircraft.longitude)
            {
                RF_FreqPlan.setPlan((int32_t)(ThisAircraft.latitude  * 600000),
                                    (int32_t)(ThisAircraft.longitude * 600000));
                RF_ready = true;
            }
        }
        else
            RF_ready = true;
    }

    /*
     * New code by Moshe Braner for frequency hopping & time slots.
     * Hardcoded specifically for Legacy Protocol.
     * More correct than original SoftRF code, and uses less CPU time.
     */

    bool non_sync = (OurTime < 1000000 || ((time_master || time_client) && !time_synched));

    /* OurTime & ref_time_ms were updated in Time_loop() */

    uint32_t now_ms = millis();
    uint32_t ms_since_pps;
    if (now_ms > ref_time_ms)
        ms_since_pps = now_ms - ref_time_ms;
    else
        ms_since_pps = 0;
    if (ms_since_pps >= 1000) {
      ++OurTime;
      ref_time_ms += 1000;
      ms_since_pps -= 1000;
    }
    time_t prev_RF_time = RF_time;
    RF_time = OurTime;
    if (! non_sync && ms_since_pps < 300 && ogn_band <= RF_BAND_AU) {
        /* channel does not change at PPS rollover in middle of slot 1 */
        --RF_time;
    }

    if (now_ms < RF_OK_until) {
        if (non_sync || RF_time == prev_RF_time) {
            /* channels already computed */
            return;
        }
        // else go ahead and recompute
//Serial.printf("recomputing slots at ms_since_pps = %d\r\n", ms_since_pps);
//Serial.printf("RF_time %d != prev_RF_time %d, ms_since_pps %d, now_ms %d, RF_OK_until %d\r\n",
//RF_time, prev_RF_time, ms_since_pps, now_ms, RF_OK_until);
    }

    /* if only one channel is allowed... */
    /* also until time is synched use channel 0 to communicate between relay ends  */

    if (non_sync || ogn_band > RF_BAND_AU) {
      rxchan = 0;
      txchan = 0;
      /* transmit only once per second */
      RF_OK_until  = ref_time_ms + 1000;
      TxTimeMarker = ref_time_ms + SoC->random(0, 995);
      TxEndMarker  = ref_time_ms + 995;
        return;
    }

    /* Only 2 channels in Europe, listening to only one of them is good enough */
    /* Thus if no exact time, use channel 0 for receiving FLARM, channel 1 for relay */
    /* (FLARM packets will be relayed as-is, without decrypting, since no time) */

    if (ogn_band == RF_BAND_EU && ognrelay_enable
              && !ogn_gnsstime && !ognreverse_time) {    // remote without exact time
        txchan = 1;
        rxchan = 0;
        RF_OK_until  = ref_time_ms + 1000;
        TxTimeMarker = ref_time_ms + SoC->random(0, 995);
        TxEndMarker  = ref_time_ms + 995;
        return;
    }

    /* the actual time slots computation for regions with more than one channel */

    if (ms_since_pps >= 300 && ms_since_pps < 800) {

      RF_current_slot = 0;
      RF_OK_until  = ref_time_ms + 800;
      TxEndMarker  = ref_time_ms + 795;
      TxTimeMarker = ref_time_ms + 400 + SoC->random(0, 395);
//      if (TxTimeMarker < now_ms)  // can only happen if setting channel after beginning of time slot
//          TxTimeMarker = TxEndMarker;   // skip transmitting for remainder of this time slot

    } else if (ms_since_pps >= 800) {

      RF_current_slot = 1;
      RF_OK_until  = ref_time_ms + 1300;
      TxEndMarker  = ref_time_ms + 1195;
      TxTimeMarker = ref_time_ms + 800 + SoC->random(0, 395);
//      if (TxTimeMarker < now_ms)
//          TxTimeMarker = TxEndMarker;

    } else {  // if (ms_since_pps < 300) - should rarely happen since OK_until 1300

      RF_current_slot = 1;
      /* channel does not change at PPS rollover in middle of slot 1 */
      RF_OK_until  = ref_time_ms + 300;
      TxTimeMarker = RF_OK_until;         // do not transmit until next slot
      TxEndMarker  = RF_OK_until;

    }

    /* Use OGN channel for relay from remote to base */

    if (ognrelay_enable) {
      rxchan = RF_FreqPlan.getChannel(RF_time, RF_current_slot, 0);      // rx on FLARM freq
      if (RF_current_slot == 0 && (RF_time & 0x07) == 0) {               // once every 8 seconds
          txchan = rxchan;
          TxTimeMarker = TxEndMarker;                                    // do not transmit
      } else {
          txchan = RF_FreqPlan.getChannel(RF_time, RF_current_slot, 1);  // tx on OGNTP freq
      }
    } else if (ognrelay_base) {
      txchan = RF_FreqPlan.getChannel(RF_time, RF_current_slot, 0);      // tx on FLARM freq
      if (RF_current_slot == 0 && (RF_time & 0x07) == 0) {               // once every 8 seconds
          rxchan = txchan;                                               // listen to FLARMs directly
          TxTimeMarker = TxEndMarker;                                    // and do not transmit
      } else {
          rxchan = RF_FreqPlan.getChannel(RF_time, RF_current_slot, 1);  // rx on OGNTP freq
      }
    } else {   // single-station
        rxchan = RF_FreqPlan.getChannel(RF_time, RF_current_slot, 0);    // FLARM freq, rx only
        txchan = rxchan;
        TxTimeMarker = TxEndMarker;
    }

//Serial.printf("rxch %d, txch %d, Slot %d at %d s + %d ms, tx ok %d - %d, good to %d\r\n",
//rxchan, txchan, RF_current_slot, OurTime, ms_since_pps, TxTimeMarker, TxEndMarker, RF_OK_until);
}

bool RF_TX_ready()
{
    uint32_t now_ms = millis();
    return (now_ms >= TxTimeMarker && now_ms < TxEndMarker);
}

size_t RF_Encode(ufo_t* fop)
{
    size_t size = 0;
    if (RF_ready && protocol_encode) {

        //if (settings->txpower == RF_TX_POWER_OFF)
        //    return size;

        uint32_t now_ms = millis();
        if (RF_TX_ready()) {
            size = (*protocol_encode)((void *) &TxBuffer[0], fop); 
        }

    }

    return size;
}

bool RF_Transmit(size_t size, bool wait)
{
    if (ognrelay_enable==false && ognrelay_base==false) {
        // should not be transmitting
        Serial.println(">>> Single-Station Transmitting?");
        return false;
    }

    //if (size == 0)
    //    return false;

    //if (settings->txpower == RF_TX_POWER_OFF)
    //    return true;

    if (RF_ready && rf_chip && (size > 0)) {

        RF_tx_size = size;

        if (!wait || RF_TX_ready()) {

            uint32_t now_ms = millis();

            if (txms > 0 && now_ms > txms+200) {
               tx_packets_counter++;  /* counts the PREVIOUS transmission */
               txms = 0;
//Serial.printf("tx_packets_counter to %d at millis=%d\r\n", tx_packets_counter, now_ms);
            }

            rf_chip->channel(txchan);
            rf_chip->transmit();         /* this may loop? */

            txms = now_ms;
            RF_tx_size = 0;
            TxTimeMarker = TxEndMarker;  /* do not transmit again until next slot */
            /* do not set next transmit time here - it is done in RF_loop()       */
            delay(10);  // now hopefully transmission is really done
            rf_chip->channel(rxchan);

// Serial.println(">");
//Serial.printf("tx on ch %d at %d s + %d ms\r\n", txchan, OurTime, txms-ref_time_ms);

            return true;
        }
    }

//static int i = 0;
//if (++i > 9) {
//i = 0;
//Serial.print("-");
//}
    return false;
}


bool RF_Receive(void)
{
    static uint8_t prevchan = 255;

    bool rval = false;

    if (RF_ready && rf_chip) {

//        if (rxchan != prevchan) {
//            prevchan = rxchan;
            rf_chip->channel(rxchan);
//        }

        rval = rf_chip->receive();

//if (rval)
//Serial.printf("rx at %d ms rxchan=%d (txchan=%d) RSSI=%d\r\n", millis()-ref_time_ms, rxchan, txchan, RF_last_rssi);
//Serial.printf("rx on ch %d at %d s + %d ms (txchan=%d)\r\n", rxchan, OurTime, millis()-ref_time_ms, txchan);

    }

    return rval;
}

void RF_Shutdown(void)
{
    if (rf_chip)
        rf_chip->shutdown();
}

uint8_t RF_Payload_Size(uint8_t protocol)
{
    switch (protocol)
    {
        case RF_PROTOCOL_LEGACY:
            return legacy_proto_desc.payload_size;
        case RF_PROTOCOL_OGNTP:
            return ogntp_proto_desc.payload_size;
        case RF_PROTOCOL_P3I:
            return p3i_proto_desc.payload_size;
        case RF_PROTOCOL_FANET:
            return fanet_proto_desc.payload_size;
        case RF_PROTOCOL_ADSB_UAT:
            return uat978_proto_desc.payload_size;
        default:
            return 0;
    }
}


osjob_t sx12xx_txjob;
osjob_t sx12xx_timeoutjob;

static void sx12xx_tx_func(osjob_t* job);

static void sx12xx_rx_func(osjob_t* job);

static void sx12xx_rx(osjobcb_t func);

static bool sx12xx_receive_complete  = false;
bool        sx12xx_receive_active    = false;
static bool sx12xx_transmit_complete = false;

static uint8_t sx12xx_channel_prev = 255;

#if defined(USE_BASICMAC)
void os_getDevEui(u1_t* buf)
{ }
u1_t os_getRegion(void)
{
    return REGCODE_EU868;     // <<< should this be set according to configured band?
}

#else
#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
    config.h in the lmic library to set it.
#endif
#endif

// >>> these ops should really be hidden within SX12XX_LL in the library: <<<

#define SX1276_RegVersion          0x42 // common
#define SX1276_FSKRegRssiValue     0x11

static u1_t sx1276_readReg(u1_t addr)
{
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
    return val;
}

static bool sx1276_probe()
{
    u1_t v, v_reset;

    SoC->SPI_begin();

    hal_init_softrf(nullptr);

    // manually reset radio
    hal_pin_rst(0);                              // drive RST pin low
    hal_waitUntil(os_getTime() + ms2osticks(1)); // wait >100us

    v_reset = sx1276_readReg(SX1276_RegVersion);

    hal_pin_rst(2);                              // configure RST pin floating!
    hal_waitUntil(os_getTime() + ms2osticks(5)); // wait 5ms

    v = sx1276_readReg(SX1276_RegVersion);

    pinMode(lmic_pins.nss, INPUT);
    SPI.end();

    if (v == 0x12)
    {
        if (v_reset == 0x12)
            RF_SX12XX_RST_is_connected = false;

        return true;
    }
    else
        return false;
}

#if defined(USE_BASICMAC)

#define CMD_READREGISTER                0x1D
#define REG_LORASYNCWORDLSB         0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

#define sx1262_GetRssiInst 0x15

static uint8_t sx1262_readcmd (uint8_t cmd, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(cmd);
    uint8_t stat = hal_spi(0x00);
    uint8_t i;
    for (i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
    return stat;
}

static void sx1262_ReadRegs(uint16_t addr, uint8_t* data, uint8_t len)
{
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++)
        data[i] = hal_spi(0x00);
    hal_spi_select(0);
}

static uint8_t sx1262_ReadReg(uint16_t addr)
{
    uint8_t val;
    sx1262_ReadRegs(addr, &val, 1);
    return val;
}

static bool sx1262_probe()
{
    u1_t v, v_reset;

    SoC->SPI_begin();

    hal_init_softrf(nullptr);

    // manually reset radio
    hal_pin_rst(0);                              // drive RST pin low
    hal_waitUntil(os_getTime() + ms2osticks(1)); // wait >100us

    v_reset = sx1262_ReadReg(REG_LORASYNCWORDLSB);

    hal_pin_rst(2);                              // configure RST pin floating!
    hal_waitUntil(os_getTime() + ms2osticks(5)); // wait 5ms

    v = sx1262_ReadReg(REG_LORASYNCWORDLSB);

    pinMode(lmic_pins.nss, INPUT);
    SPI.end();

    u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
    if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb)
    {
        if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb)
            RF_SX12XX_RST_is_connected = false;

        return true;
    }
    else
        return false;
}

#endif

static void sx12xx_channel(uint8_t channel)
{
    if (channel != sx12xx_channel_prev)
    {
        uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);
        int8_t   fc        = settings->freq_corr;

        //Serial.print("frequency: "); Serial.println(frequency);

        if (sx12xx_receive_active)
        {
            os_radio(RADIO_RST);
            sx12xx_receive_active = false;
        }

        if (rf_chip->type == RF_IC_SX1276)
        {
            /* correction of not more than 30 kHz is allowed */
            if (fc > 30)
                fc = 30;
            else if (fc < -30)
                fc = -30;
            ;
        }
        else
            /* Most of SX1262 designs use TCXO */
            fc = 0;

        /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
        LMIC.freq = frequency + (fc * 1000);
        //LMIC.freq = 868200000UL;

        sx12xx_channel_prev = channel;
    }
}

static void sx12xx_setup()
{
    SoC->SPI_begin();

    // initialize runtime env
    os_init(nullptr);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    // range test.
    LMIC.agcref = 0x00;

    switch (ogn_protocol_1)
    {
        case RF_PROTOCOL_OGNTP:
            LMIC.protocol   = &ogntp_proto_desc;
            protocol_encode = &ogntp_encode;
            protocol_decode = &ogntp_decode;
            break;
        case RF_PROTOCOL_P3I:
            LMIC.protocol   = &p3i_proto_desc;
            protocol_encode = &p3i_encode;
            protocol_decode = &p3i_decode;
            break;
        case RF_PROTOCOL_FANET:
            LMIC.protocol   = &fanet_proto_desc;
            protocol_encode = &fanet_encode;
            protocol_decode = &fanet_decode;
            break;
        case RF_PROTOCOL_LEGACY:
        default:
            LMIC.protocol   = &legacy_proto_desc;
            protocol_encode = &legacy_encode;
            protocol_decode = &legacy_decode;
            /*
             * Enforce legacy protocol setting for SX1276
             * if other value (UAT) left in EEPROM from other (UATM) radio
             */
            ogn_protocol_1 = RF_PROTOCOL_LEGACY;
            break;
    }

    //switch (settings->txpower)
    //{
    //    case RF_TX_POWER_FULL:

            /* Load regional max. EIRP at first */
            LMIC.txpow = RF_FreqPlan.MaxTxPower;

            if (rf_chip->type == RF_IC_SX1262)
            {
                /* SX1262 is unable to give more than 22 dBm */
                if (LMIC.txpow > 22)
                    LMIC.txpow = 22;
            }
            else
            /* SX1276 is unable to give more than 20 dBm */
            if (LMIC.txpow > 20)
                LMIC.txpow = 20;

            if (ogn_port == 0)   // testing of 2-station setup at short range
                LMIC.txpow = 2;
#if 1
            /*
             * Enforce Tx power limit until confirmation
             * that RFM95W is doing well
             * when antenna is not connected
             */
            if (LMIC.txpow > 17)
                LMIC.txpow = 17;
#endif
    //        break;
    //    case RF_TX_POWER_OFF:
    //    case RF_TX_POWER_LOW:
    //    default:
    //        LMIC.txpow = 2; /* 2 dBm is minimum for RFM95W on PA_BOOST pin */
    //        break;
    //}
}

static void sx12xx_setvars()
{
    if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA)
    {
        LMIC.datarate = LMIC.protocol->bitrate;
        LMIC.syncword = LMIC.protocol->syncword[0];
    }
    else
        LMIC.datarate = DR_FSK;

#if defined(USE_BASICMAC)

#define updr2rps  LMIC_updr2rps

    // LMIC.rps = MAKERPS(sf, BW250, CR_4_5, 0, 0);

    LMIC.noRXIQinversion = true;
    LMIC.rxsyms          = 100;

#endif /* USE_BASICMAC */

    // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
    LMIC.rps = updr2rps(LMIC.datarate);


    if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET)
        /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
        LMIC.rps = setCr(LMIC.rps, CR_4_8);

    // set SX1276 AGC Reference
    /*if(settings->sxlna){
       LMIC.agcref = 0x13;
       // Serial.printf("setting agc ref to 0x%x\n", 0x13);
       }
       else{
       LMIC.agcref = 0x00;
       //Serial.printf("setting agc ref to 0x%x\n", 0x00);
       }*/
}

static void get_idle_rssi()
{
    static uint32_t next_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms < next_ms)
        return;
    next_ms = now_ms + 7;     // long enough to span a packet reception
    uint8_t noise = 200;
    if (rf_chip->type == RF_IC_SX1276)
        noise = sx1276_readReg(SX1276_FSKRegRssiValue);
    else if (rf_chip->type == RF_IC_SX1262)
        sx1262_readcmd(sx1262_GetRssiInst, &noise, 1);
    int8_t rssi = -(int)(noise>>1);   // RSSI as dBm
    // once packet reception starts, apparently the sx1276 RSSI register
    //    returns signal strength at first, and zero later
    if (rssi != 0 && RF_last_idle_rssi != 0) {
        avg_idle_rssi = ((3*avg_idle_rssi + RF_last_idle_rssi) >> 2);   // running average
        bool take_sample = false;
        if (noise_sampling == 2) {
            take_sample = true;
        } else if (noise_sampling == 1) {
            // only sample in the idle time between slots 1 & 0 
            uint32_t ms_since_pps = now_ms - ref_time_ms;
            take_sample = (ms_since_pps > 240 && ms_since_pps < 390);
        }
        if (take_sample) {
            if (noise_count[rxchan] < 0x1000) {
                ++noise_count[rxchan];
                noise_data[rxchan] += rssi;
            }
            RF_last_rx_chan = rxchan;
            RF_last_idle_rssi = rssi;     // used for sending from remote to base
            RF_have_noise_sample = true;
        }
    }
}

static bool sx12xx_receive()
{
    bool success = false;
    //String msg;

    sx12xx_receive_complete = false;

    if (!sx12xx_receive_active)
    {
        // msg = "activating receive...";
        // Logger_send_udp(&msg);    
        sx12xx_setvars();
        sx12xx_rx(sx12xx_rx_func);
        sx12xx_receive_active = true;

    } else if (sx12xx_receive_complete == false) {
        // receiver was already active, but has not received a packet
        get_idle_rssi();     // sample, and side effect: updates avg_idle_rssi
    }

    if (sx12xx_receive_complete == false){
        // execute scheduled jobs and events        
        os_runstep();
    }

    if (sx12xx_receive_complete == true)
    {
        u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

        if (size > sizeof(RxBuffer))
            size = sizeof(RxBuffer);

        //for (u1_t i=0; i < size; i++){
        //    RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
        //}
        memcpy(RxBuffer, &LMIC.frame[LMIC.protocol->payload_offset], (size_t) size);

        RF_last_rssi = LMIC.rssi;    // signal strength during packet reception

        // compute an SNR based on RSSI during reception and average RSSI in preceding 30 ms or so
//Serial.print("pkt_rssi=");
//Serial.print((int)RF_last_rssi);
//Serial.print("  idle_rssi=");
//Serial.println((int)avg_idle_rssi);
        RF_last_snr = (RF_last_rssi - avg_idle_rssi);

        // msg = "Receive complete...";
        // Logger_send_udp(&msg);    

#if 0
        /*decrypt payload for private network*/
        /*if packet is bigger , maybe its encrypted*/
        /*V0.1.0-25*/
          if(size > RF_Payload_Size(ogn_protocol_1)){
            char *decrypted;
            size_t decrypted_len;
            switch (ogn_protocol_1)
              {
               case RF_PROTOCOL_FANET:
                    PNETdecrypt(RxBuffer, size, &decrypted, &decrypted_len);
                    if(decrypted_len == RF_Payload_Size(ogn_protocol_1)){      
                      for(size_t i=0; i<decrypted_len;i++){
                        RxBuffer[i] = decrypted[i];
                          }
                        }
                    free(decrypted);        
                    break;
               case RF_PROTOCOL_LEGACY:
               case RF_PROTOCOL_P3I:
               case RF_PROTOCOL_OGNTP:
                    break;
              }
          }
#endif
        rx_packets_counter++;
        success = true;
    }
    return success;
}

static void sx12xx_transmit()
{
    sx12xx_transmit_complete = false;
    sx12xx_receive_active    = false;

    sx12xx_setvars();
    os_setCallback(&sx12xx_txjob, sx12xx_tx_func);

    while (sx12xx_transmit_complete == false) {
        // execute scheduled jobs and events
        os_runstep();

        yield();
    };
}

static void sx12xx_shutdown()
{
    LMIC_shutdown();
    SPI.end();
}

// Enable rx mode and call func when a packet is received
static void sx12xx_rx(osjobcb_t func)
{
    LMIC.osjob.func = func;
    LMIC.rxtime     = os_getTime(); // RX _now_
    // Enable "continuous" RX for LoRa only (e.g. without a timeout,
    // still stops after receiving a packet)
    os_radio(LMIC.protocol &&
             LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
             RADIO_RXON : RADIO_RX);
    //Serial.println("RX");
}

static void sx12xx_rx_func(osjob_t* job)
{
    u1_t crc8, pkt_crc8;
    u2_t crc16, pkt_crc16;
    u1_t i;

    // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
    sx12xx_receive_active = false;

    /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
    if (LMIC.dataLen == 0)
        return;

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            /* crc16 left not initialized */
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            crc8 = 0x71; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_0000:
            crc16 = 0x0000; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        default:
            crc16 = 0xffff; /* seed value */
            break;
    }

    //Serial.print("Got ");
    //Serial.print(LMIC.dataLen);
    //Serial.println(" bytes");

    switch (LMIC.protocol->type)
    {
        case RF_PROTOCOL_LEGACY:
            /* take in account NRF905/FLARM "address" bytes */
            crc16 = update_crc_ccitt_16(crc16, 0x31);
            crc16 = update_crc_ccitt_16(crc16, 0xFA);
            crc16 = update_crc_ccitt_16(crc16, 0xB6);
            break;
        case RF_PROTOCOL_P3I:
        case RF_PROTOCOL_OGNTP:
        default:
            break;
    }

    for (i = LMIC.protocol->payload_offset;
         i < (LMIC.dataLen - LMIC.protocol->crc_size);
         i++)
    {
        switch (LMIC.protocol->crc_type)
        {
            case RF_CHECKSUM_TYPE_GALLAGER:
            case RF_CHECKSUM_TYPE_NONE:
                break;
            case RF_CHECKSUM_TYPE_CRC8_107:
                update_crc8(&crc8, (u1_t)(LMIC.frame[i]));
                break;
            case RF_CHECKSUM_TYPE_CCITT_FFFF:
            case RF_CHECKSUM_TYPE_CCITT_0000:
            default:
                crc16 = update_crc_ccitt_16(crc16, (u1_t)(LMIC.frame[i]));
                break;
        }

        switch (LMIC.protocol->whitening)
        {
            case RF_WHITENING_NICERF:
                LMIC.frame[i] ^= pgm_read_byte(&whitening_pattern[i - LMIC.protocol->payload_offset]);
                break;
            case RF_WHITENING_MANCHESTER:
            case RF_WHITENING_NONE:
            default:
                break;
        }
#if DEBUG
        Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
    }

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_NONE:
            sx12xx_receive_complete = true;
            break;
        case RF_CHECKSUM_TYPE_GALLAGER:
            if (LDPC_Check((uint8_t *) &LMIC.frame[0]))
            {
#if DEBUG
                Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FEC",
                              LMIC.frame[i], LMIC.frame[i + 1], LMIC.frame[i + 2],
                              LMIC.frame[i + 3], LMIC.frame[i + 4], LMIC.frame[i + 5]);
#endif
                ++packets_failed_crc;
                sx12xx_receive_complete = false;
            }
            else
                sx12xx_receive_complete = true;
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            pkt_crc8 = LMIC.frame[i];
#if DEBUG
            if (crc8 == pkt_crc8)
                Serial.printf(" %02x is valid crc", pkt_crc8);
            else {
                Serial.printf(" %02x is wrong crc", pkt_crc8);
#endif
            if (crc8 == pkt_crc8) {
                sx12xx_receive_complete = true;
            } else {
                sx12xx_receive_complete = false;
                ++packets_failed_crc;
            }
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
        default:
            pkt_crc16 = (LMIC.frame[i] << 8 | LMIC.frame[i + 1]);
#if DEBUG
            if (crc16 == pkt_crc16)
                Serial.printf(" %04x is valid crc", pkt_crc16);
            else
                Serial.printf(" %04x is wrong crc", pkt_crc16);

#endif
            if (crc16 == pkt_crc16) {
                sx12xx_receive_complete = true;
                RF_last_crc = crc16;
                RF_last_bec = 0;
            } else {
                ++packets_failed_crc;
                sx12xx_receive_complete = false;
                if (ogn_bec) {
                    // include the packet's CRC bytes in the computed CRC
                    crc16 = update_crc_ccitt_16(crc16, (u1_t)(LMIC.frame[i]));
                    crc16 = update_crc_ccitt_16(crc16, (u1_t)(LMIC.frame[i+1]));
                    // try bit error correction
                    RF_last_bec = bec_correct_errors( crc16, &LMIC.frame[LMIC.protocol->payload_offset] );
                    if (RF_last_bec != 0) {
                        ++packets_corrected;
                        sx12xx_receive_complete = true;
                    }
                }
            }
            break;
    }

#if DEBUG
    Serial.println();
#endif
}

// Transmit the given string and call the given function afterwards
static void sx12xx_tx(unsigned char* buf, size_t size, osjobcb_t func)
{
    u1_t crc8;
    u2_t crc16;


    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            /* crc16 left not initialized */
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            crc8 = 0x71; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_0000:
            crc16 = 0x0000; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        default:
            crc16 = 0xffff; /* seed value */
            break;
    }

    os_radio(RADIO_RST); // Stop RX first
    delay(1);            // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

    LMIC.dataLen = 0;

    switch (LMIC.protocol->type)
    {
        case RF_PROTOCOL_LEGACY:
            /* take in account NRF905/FLARM "address" bytes */
            crc16 = update_crc_ccitt_16(crc16, 0x31);
            crc16 = update_crc_ccitt_16(crc16, 0xFA);
            crc16 = update_crc_ccitt_16(crc16, 0xB6);
            break;
        case RF_PROTOCOL_P3I:
            /* insert Net ID */
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  8) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
            /* insert byte with payload size */
            LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

            /* insert byte with CRC-8 seed value when necessary */
            if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107)
                LMIC.frame[LMIC.dataLen++] = crc8;

            break;
        case RF_PROTOCOL_OGNTP:
        default:
            break;
    }

    for (u1_t i=0; i < size; i++) {
        switch (LMIC.protocol->whitening)
        {
            case RF_WHITENING_NICERF:
                LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
                break;
            case RF_WHITENING_MANCHESTER:
            case RF_WHITENING_NONE:
            default:
                LMIC.frame[LMIC.dataLen] = buf[i];
                break;
        }

        switch (LMIC.protocol->crc_type)
        {
            case RF_CHECKSUM_TYPE_GALLAGER:
            case RF_CHECKSUM_TYPE_NONE:
                break;
            case RF_CHECKSUM_TYPE_CRC8_107:
                update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
                break;
            case RF_CHECKSUM_TYPE_CCITT_FFFF:
            case RF_CHECKSUM_TYPE_CCITT_0000:
            default:
                crc16 = update_crc_ccitt_16(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
                break;
        }

        LMIC.dataLen++;
    }

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            LMIC.frame[LMIC.dataLen++] = crc8;
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
        default:
            LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
            LMIC.frame[LMIC.dataLen++] = (crc16) & 0xFF;
            break;
    }

    LMIC.osjob.func = func;
    os_radio(RADIO_TX);
}

static void sx12xx_txdone_func(osjob_t* job)
{
    sx12xx_transmit_complete = true;
}

static void sx12xx_tx_func(osjob_t* job)
{
    if (RF_tx_size > 0)
        sx12xx_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx12xx_txdone_func);
}
