/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 *
 * Protocol_Legacy, encoder for legacy radio protocol
 * Copyright (C) 2019-2020 Linar Yusupov
 *
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

#include <math.h>
#include <stdint.h>

#include <protocol.h>

#include "SoftRF.h"
#include "global.h"
#include "Time.h"
#include "RF.h"
#include "Protocol_Legacy.h"
#include "EEPROM.h"
#include "Log.h"

const rf_proto_desc_t legacy_proto_desc = {
    "Legacy",
    .type            = RF_PROTOCOL_LEGACY,
    .modulation_type = RF_MODULATION_TYPE_2FSK,
    .preamble_type   = LEGACY_PREAMBLE_TYPE,
    .preamble_size   = LEGACY_PREAMBLE_SIZE,
    .syncword        = LEGACY_SYNCWORD,
    .syncword_size   = LEGACY_SYNCWORD_SIZE,
    .net_id          = 0x0000, /* not in use */
    .payload_type    = RF_PAYLOAD_INVERTED,
    .payload_size    = LEGACY_PAYLOAD_SIZE,
    .payload_offset  = 0,
    .crc_type        = LEGACY_CRC_TYPE,
    .crc_size        = LEGACY_CRC_SIZE,

    .bitrate   = RF_BITRATE_100KBPS,
    .deviation = RF_FREQUENCY_DEVIATION_50KHZ,
    .whitening = RF_WHITENING_MANCHESTER,
    .bandwidth = RF_RX_BANDWIDTH_SS_125KHZ,

    .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
    .tx_interval_max = LEGACY_TX_INTERVAL_MAX
};

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t* v, int8_t n, const uint32_t key[4])
{
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1)
    {
        /* Coding Part */
        rounds = ROUNDS;
        sum    = 0;
        z      = v[n - 1];
        do {
            sum += DELTA;
            e    = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    }
    else if (n < -1)
    {
        /* Decoding Part */
        n      = -n;
        rounds = ROUNDS;
        sum    = rounds * DELTA;
        y      = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z    = v[n - 1];
            y    = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed)
{
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static const uint32_t table[8] = LEGACY_KEY1;

/* FLARm uses a time-dependent key for encrypting packets */
void make_key(uint32_t *key, uint32_t timestamp, uint32_t address)
{
    int i, ndx;
    for (i = 0; i < 4; i++) {
        ndx    = ((timestamp >> 23) & 1) ? i + 4 : i;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

/* a non-time-dependent key for relayed packets */
void make_relay_key(uint32_t *key)
{
    int i;
    for (i = 0; i < 4; i++)
        key[i] = (ognrelay_key << i) | i;
}

bool legacy_decode(void* buffer, ufo_t* this_aircraft, ufo_t* fop)
{
    String msg;

    legacy_packet_t* pkt = (legacy_packet_t *) buffer;

    fop->addr = pkt->addr;     /* first 32 bits are not encrypted */
//Serial.printf("decoding pkt, unk0=%X, addr=%06X\r\n", pkt->_unk0, pkt->addr);

    if (ognrelay_enable) {

        if (pkt->_unk0 != 0) {    /* received relayed, or special FLARM packet - ignore */
          ++other_packets_recvd;
Serial.printf("received non-traffic pkt, unk0=%X\r\n", pkt->_unk0);
          return false;
        }

    } else if (ognrelay_base) {

        if (pkt->_unk0 != 0xB && pkt->_unk0 != 0xD) {  /* other than relayed packets */
          ++other_packets_recvd;
Serial.printf("received non-relay pkt, unk0=%X, addr=%06X\r\n", pkt->_unk0, pkt->addr);
          return false;          /* ignore normal packets */
        }
        /* otherwise fall through and decode relayed packets */

    } else {    /* single station mode */

        if (pkt->_unk0 != 0) {    /* received packets that are not normal traffic - ignore */
          ++other_packets_recvd;
//Serial.printf("received non-traffic pkt, unk0=%X\r\n", pkt->_unk0);
          return false;
        }
        /* otherwise fall through and decode normal packets */
    }

//    if (pkt->airborne == 0) {
//        ++other_packets_recvd;
//        return false;                /* FLARM sends strange coords with airborne=0 some times */
//    }

//Serial.println("decoding traffic packet...");

    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    if (ognrelay_enable && ! ogn_gnsstime && ! have_approx_time) {
        /* no time data, cannot decrypt */
        fop->timestamp = timestamp;
        fop->stealth  = 0;          /* for queueing in traffic.cpp */
        fop->no_track = 0;
//Serial.println("returning un-decrypted packet");
        return true;
    }

    bool time_sent = (ognrelay_base && (pkt->_unk0 == 0xD));  /* timestamp sent from remote station */

    /* decrypt packet */
    uint32_t key[4];
    if (ognrelay_base) {
        /* relayed packets have different encryption key if remote has time data */
        if (time_sent) {
            make_relay_key(key);
        } else {
            if (pkt->_unk1 == 0)
                timestamp -= 4;       /* estimate average relaying delay */
            else
                timestamp -= 16;      /* remote station had long delay before relaying */
            make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
        }
    } else {   /* single station, or remote with GNSS time */
        make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    }
    uint32_t *p = (uint32_t *) pkt;
    btea(p+1, -5, key);
    
    /* check data integrity */
    if (time_sent) {
        /* compute checksum and reject packet if fails */
        uint16_t sentsum = ((uint16_t) pkt->ew[2] << 7) | (uint16_t) (pkt->ew[3] & 0x7F);
        pkt->ew[2] = 0;
        pkt->ew[3] = 0;
        uint16_t checksum=0;
        for (int ndx = 0; ndx < sizeof (legacy_packet_t); ndx++)
            checksum += (*(((unsigned char *) pkt) + ndx));
        if (checksum != sentsum) {
Serial.println("bad checksum in relayed packet");
                return false;
        }
        fop->rssi  = pkt->ew[1];        /* sent from remote */
        /* skip parity check on re-encrypted relayed packets - we did a checksum instead */
    } else {
        if (ognrelay_base)
            pkt->_unk0 = 0;       /* try and restore original parity */
        uint8_t pkt_parity=0;
        for (int ndx = 0; ndx < sizeof (legacy_packet_t); ndx++)
            pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
        if (pkt_parity % 2) {
//if (ognrelay_base)
//Serial.println("bad parity in relayed packet");
//else
//Serial.println("bad parity in original packet");
            return false;
        }
        // since parity will be right 50% of the time by chance, it's a very weak check!
        fop->rssi  = RF_last_rssi;      /* local reception */
    }

    fop->timestamp = timestamp;
    fop->addr_type = pkt->addr_type;
    fop->stealth   = pkt->stealth;
    fop->no_track  = pkt->no_track;

    /* set hour, minute, second for OGN reporting of this packet */

    if (! time_sent) {
      fop->hour   = this_aircraft->hour;
      fop->minute = this_aircraft->minute;
      fop->second = this_aircraft->second;
    }

    if (ognrelay_enable) {
      if (!ogn_gnsstime) {
        // >>> do some sanity checks here in case approx_time is off by a second
        //     and thus the decrypted data is random garbage
        // - if we knew what is in the "gps" field could check that
        if (pkt->airborne == 0)  return false;
        if (pkt->stealth)   return false;
        if (pkt->no_track)  return false;
      }
      /* do not decode further */
      /* packet will be relayed in Traffic.cpp using legacy_encode() below */
      return true;
    }

    if (ognrelay_base) {

      if (!ogn_gnsstime && !ognrelay_time) {
        // >>> do some sanity checks here in case NTP time is off by a second
        //     and thus the decrypted data is random garbage
        // - if we knew what is in the "gps" field could check that
        if (pkt->airborne == 0)  return false;
        if (pkt->stealth)   return false;
        if (pkt->no_track)  return false;
      }

      if (time_sent) {

          fop->hour   = pkt->ns[1];   /* original H,M,S sent from remote */
          fop->minute = pkt->ns[2];
          fop->second = pkt->ns[3];

      } else {                      /* approximate adjustment for relay delay */
          int delay = (pkt->_unk1 == 0) ? 4 : 16;
          if (fop->second >= delay) {
              fop->second -= delay;
          } else if (fop->minute > 0) {
            --fop->minute;
            fop->second += 60 - delay;
          } else {
            if (fop->hour > 0)
              --fop->hour;
            else
              fop->hour = 23;
            fop->minute = 59;
            fop->second += 60 - delay;
          }
      }
    }  /* single-station leaves HMS as-was in this_aircraft */

    float    ref_lat   = this_aircraft->latitude;
    float    ref_lon   = this_aircraft->longitude;
    float    geo_separ = this_aircraft->geoid_separation;

    // int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
    // int32_t lat       = (pkt->lat - round_lat) % (uint32_t) 0x080000;
    // if (lat >= 0x040000)
    //    lat -= 0x080000;
    // lat = ((lat + round_lat) << 7) /* + 0x40 */;
    int32_t round_lat, round_lon;
    if (ref_lat < 0.0)
        round_lat = -(((int32_t) (-ref_lat * 1e7) + 0x40) >> 7);
    else
        round_lat = ((int32_t) (ref_lat * 1e7) + 0x40) >> 7;
    int32_t ilat = ((int32_t)pkt->lat - round_lat) & 0x07FFFF;
    if (ilat >= 0x040000) ilat -= 0x080000;
    float lat = (float)((ilat + round_lat) << 7) * 1e-7;

    // int32_t round_lon = (int32_t) (ref_lon * 1e7) >> 7;
    // int32_t lon       = (pkt->lon - round_lon) % (uint32_t) 0x100000;
    // if (lon >= 0x080000)
    //     lon -= 0x100000;
    // lon = ((lon + round_lon) << 7) /* + 0x40 */;
    if (ref_lon < 0.0)
        round_lon = -(((int32_t) (-ref_lon * 1e7) + 0x40) >> 7);
    else
        round_lon = ((int32_t) (ref_lon * 1e7) + 0x40) >> 7;
    int32_t ilon = ((int32_t)pkt->lon - round_lon) & 0x0FFFFF;
    if (ilon >= 0x080000) ilon -= 0x0100000;
    float lon = (float)((ilon + round_lon) << 7) * 1e-7;

    int32_t ns = pkt->ns[0];
    int32_t ew = pkt->ew[0];
    float   speed4 = sqrtf((float)(ew * ew + ns * ns)) * (float)(1 << pkt->smult);

    float direction = 0;
    if (speed4 > 0)
    {
        direction = atan2f(ew, ns) * 180.0 / PI; /* -180 ... 180 */
        direction = (direction >= 0.0 ? direction : direction + 360);
    }

    uint16_t vs_u16 = pkt->vs;
    int16_t  vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1 << 9) ? 0xFC00U : 0));
    int16_t  vs10   = vs_i16 << pkt->smult;

    int16_t alt = pkt->alt;  /* relative to WGS84 ellipsoid */

    fop->protocol = RF_PROTOCOL_LEGACY;

    fop->addr_type     = pkt->addr_type;
    fop->latitude      = lat;
    fop->longitude     = lon;
    fop->altitude      = (float) alt - geo_separ;
    fop->speed         = speed4 / (4 * _GPS_MPS_PER_KNOT);
    fop->course        = direction;
    fop->vs            = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
    fop->aircraft_type = pkt->aircraft_type;

    return true;
}

/* this not actually used in OGNbase */
size_t legacy_encode_this(void* legacy_pkt, ufo_t* this_aircraft)
{
    legacy_packet_t* pkt = (legacy_packet_t *) legacy_pkt;

    int      ndx;
    uint8_t  pkt_parity=0;
    uint32_t key[4];

    uint32_t id        = this_aircraft->addr;
    float    lat       = this_aircraft->latitude;
    float    lon       = this_aircraft->longitude;
    int16_t  alt       = (int16_t) (this_aircraft->altitude + this_aircraft->geoid_separation);
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    float course = this_aircraft->course;
    float speedf = this_aircraft->speed * _GPS_MPS_PER_KNOT;         /* m/s */
    float vsf    = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

    // scale by 4, clamp to max
    uint16_t speed4 = (uint16_t) roundf(speedf * 4.0f);
    if (speed4 > 8 * 128 - 1)
        speed4 = 8 * 128 - 1;

    // scale by 10, clamp to max
    int16_t vs10 = (int16_t) roundf(vsf * 10.0f);
    if (vs10 > 8 * 512 - 1)
        vs10 = 8 * 512 - 1;
    else if (vs10 < -8 * 512)
        vs10 = -8 * 512;


    pkt->smult = 0;
    // first check horizontal speed
    while ((pkt->smult < 3) && (speed4 > 128 - 1)) {
        speed4 >>= 1;
        pkt->smult++;
        vs10 >>= 1;
    }
    // now check vertical speed
    while ((pkt->smult < 3) && ((vs10 > 512 - 1) || (vs10 < -512))) {
        vs10 >>= 1;
        pkt->smult++;
        speed4 >>= 1;
    }

    uint8_t speed = speed4;

    int8_t ns = (int8_t) (speed * cosf(radians(course)));
    int8_t ew = (int8_t) (speed * sinf(radians(course)));

    pkt->vs = vs10;

    pkt->addr = id & 0x00FFFFFF;

#if !defined(SOFTRF_ADDRESS)
    pkt->addr_type = ADDR_TYPE_FLARM; /* ADDR_TYPE_ANONYMOUS */
#else
    pkt->addr_type = (pkt->addr == SOFTRF_ADDRESS ?
                      ADDR_TYPE_ICAO : ADDR_TYPE_FLARM); /* ADDR_TYPE_ANONYMOUS */
#endif

    pkt->parity = 0;

    pkt->stealth  = this_aircraft->stealth;
    pkt->no_track = this_aircraft->no_track;

    pkt->aircraft_type = this_aircraft->aircraft_type;

    pkt->gps = 323;

    // pkt->lat = (uint32_t(lat * 1e7) >> 7) & 0x7FFFF;
    // pkt->lon = (uint32_t(lon * 1e7) >> 7) & 0xFFFFF;
    if (lat < 0.0)
        pkt->lat = (uint32_t) (-(((int32_t) (-lat * 1e7) + 0x40) >> 7)) & 0x07FFFF;
    else
        pkt->lat = (((uint32_t) (lat * 1e7) + 0x40) >> 7) & 0x07FFFF;
    if (lon < 0.0)
        pkt->lon = (uint32_t) (-(((int32_t) (-lon * 1e7) + 0x40) >> 7)) & 0x0FFFFF;
    else
        pkt->lon = (((uint32_t) (lon * 1e7) + 0x40) >> 7) & 0x0FFFFF;

    if (alt < 0)
    {
        pkt->alt = 0; // cannot be negative
    }
    else
    {
        if (alt >= (1 << 13))
        {
            pkt->alt = (1 << 13) - 1; // clamp to Max
        }
        else
            pkt->alt = alt;
    }

    pkt->airborne = speed > 0 ? 1 : 0;
    pkt->ns[0]    = ns;
    pkt->ns[1]    = ns;
    pkt->ns[2]    = ns;
    pkt->ns[3]    = ns;
    pkt->ew[0]    = ew;
    pkt->ew[1]    = ew;
    pkt->ew[2]    = ew;
    pkt->ew[3]    = ew;

    pkt->_unk0 = 0;
    pkt->_unk1 = 0;
    pkt->_unk2 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++)
        pkt_parity += parity(*(((unsigned char *) pkt) + ndx));

    pkt->parity = (pkt_parity % 2);

    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, 5, key);

    return sizeof(legacy_packet_t);
}

/* special encoding for relayed packets */

size_t legacy_encode(void* buffer, ufo_t* fop)
{
    if (fop == &ThisAircraft)
        return legacy_encode_this(buffer, fop);   /* may happen once at start-up */

    memcpy(buffer, fop->raw, sizeof(legacy_packet_t));

    legacy_packet_t* pkt = (legacy_packet_t *) buffer;

    if (ognrelay_enable && (ogn_gnsstime || have_approx_time)) {    /* need to re-encrypt */

        pkt->ns[1] = fop->hour;
        pkt->ns[2] = fop->minute;
        pkt->ns[3] = fop->second;
        pkt->ew[1] = fop->rssi;
        pkt->_unk0 = 0xD;       /* marks as relayed packet with a timestamp */

        int ndx;
        pkt->ew[2] = 0;
        pkt->ew[3] = 0;
        uint16_t checksum=0;
        for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++)
            checksum += (*(((unsigned char *) pkt) + ndx));
        pkt->ew[2] = (checksum & 0x3F80) >> 7;
        pkt->ew[3] = checksum & 0x007F;

        uint32_t key[4];
        make_relay_key(key);
        uint32_t *p = (uint32_t *) pkt;
        btea(p+1, 5, key);

    } else {    /* do not re-encrypt, it is still in the original FLARM encryption */

        /* use bits in the first - unencrypted - word as flags */
        pkt->_unk0 = 0xB;       /* marks as not having a timestamp */
        pkt->_unk1 = (ThisAircraft.timestamp > fop->timestamp + 12) ? 1 : 0;

    }

    return sizeof(legacy_packet_t);
}
