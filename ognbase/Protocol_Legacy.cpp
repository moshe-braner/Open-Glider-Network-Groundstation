/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 * Copyright (C) 2019-2020 Linar Yusupov
 * Copyright (C) 2024 Moshe Braner
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

  .air_time        = 5,   // LEGACY_AIR_TIME,
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,  
    .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
    .tx_interval_max = LEGACY_TX_INTERVAL_MAX
//  .slot0           = {400,  800},
//  .slot1           = {800, 1200}
};

#define DELTA 0x9e3779b9

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t* v, int8_t n, const uint32_t key[4])
{
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

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

// first stage of decrypting/encrypting 2024 protocol
//  - also second stage of encrypting (encode=true)
void btea2(uint32_t *data, bool encode)
{
    // for new protocol, ths btea() stage uses fixed keys
    static const uint32_t keys[4] = { 0xa5f9b21c, 0xab3f9d12, 0xc6f34e34, 0xd72fa378 };
    btea(data+2, (encode? 4 : -4), keys);
}

// second stage of decrypting 2024 protocol
//  - also first stage of encrypting
void scramble(uint32_t *data, uint32_t timestamp)
{

    uint32_t wkeys[4];
    wkeys[0] = data[0];
    wkeys[1] = data[1];
    wkeys[2] = (timestamp >> 4);
    wkeys[3] = 0x956f6c77;         // the scramble KEY

    int z, y, x, sum, p, q;
    //int n = 16;                        // do by bytes instead of longwords
    byte *bkeys = (byte *) wkeys;

    z = bkeys[15];
    sum = 0;
    q = 2;                         // only 2 iterations
    do {
      sum += DELTA;
      y = bkeys[0];
      for (p=0; p<15; p++) {
        x = y;
        y = bkeys[p+1];
        x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
        bkeys[p] = (byte)x;
        z = x & 0xff;
      }
      x = y;
      y = bkeys[0];
      x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
      bkeys[15] = (byte)x;
      z = x & 0xff;
    } while (--q > 0);

    // now XOR results with last 4 words of the packet
    data[2] ^= wkeys[0];
    data[3] ^= wkeys[1];
    data[4] ^= wkeys[2];
    data[5] ^= wkeys[3];
}


/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed)
{
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static const uint32_t table[8] = LEGACY_KEY1;

/* FLARM uses a time-dependent key for encrypting packets */
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

// lookup the divisor for longitude for new protocol
int londiv(int ilat)
{
    static uint8_t table1[] = {
        53,53,54,54,55,55,
        56,56,57,57,58,58,59,59,60,60,
        61,61,62,62,63,63,64,64,65,65,
        67,68,70,71,73,74,76,77,79,80,
        82,83,85,86,88,89,91,94,98,101,
        105,108,112,115,119,122,126,129,137,144,
        152,159,167,174,190,205,221,236,252
    };
    static uint16_t table2[] = {
        267,299,330,362,425,489,552,616,679,743,806
    };
    if (ilat < 14)
        return 52;
    if (ilat < 79)
        return table1[ilat-14];
    if (ilat > 89)
        return 806;
    return table2[ilat-79];
}

static char hexbuf[128];
char *bytes2Hex(byte *buffer, size_t size)
{
  char *p = hexbuf;
  for (int i=0; i < size && i < 63; i++) {
    byte c = buffer[i];
    byte cl = c & 0x0F;
    byte cu = (c >> 4) & 0x0F;
    *p++ = (cu < 0x0A)? '0'+cu : 'A'+cu-0x0A;
    *p++ = (cl < 0x0A)? '0'+cl : 'A'+cl-0x0A;
  }
  *p = '\0';
  return hexbuf;
}

// unpack integer value from a pseudo-floating format
int descale( unsigned int value, unsigned int mbits, unsigned int ebits, unsigned int sbits)
{
    unsigned int offset = (1 << mbits);
    unsigned int signbit = (offset << ebits);
    unsigned int negative = 0;
    if (sbits != 0)
        negative = (value & signbit);
    value &= (signbit - 1);           // ignores signbit and higher
    if (value >= offset) {
        unsigned int exp = value >> mbits;
        value &= (offset - 1);
        value += offset;
        value <<= exp;
        value -= offset;
    }
    return (negative? -(int)value : value);
}

// interpret the data fields in new protocol packet
//     https://pastebin.com/YB1ppAbt
bool latest_decode(void* buffer, ufo_t* this_aircraft, ufo_t* fop)
{
    //uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
    //uint32_t timestamp = (uint32_t) OurTime;
    uint32_t timestamp = (uint32_t) RF_time;   // incremented in RF.cpp 300 ms after PPS

    new_packet_t* pkt = (new_packet_t *) buffer;

    bool original = true;
    if (pkt->msg_type == MSG_TYPE_RAW) {   // relayed in original encryption
        original = false;
        // estimate original timestamp - may be off by several seconds
        if (pkt->_unk1 == 0)
            timestamp -= 4;       /* estimate average relaying delay */
        else
            timestamp -= 16;      /* remote station had long delay before relaying */
        // restore original contents of first word in packet
        pkt->_unk1 = 0;
        pkt->msg_type = MSG_TYPE_NEW;
    }

    /* decrypt packet */
    uint32_t *wp = (uint32_t *) buffer;
    btea2(wp, false);
    scramble(wp, timestamp);

#if 0
    if (testmode_enable)
    /* output the raw (but decrypted) packet as a whole, in hex */
    Serial.printf("pkt,%06X,%ld,%s\r\n",
      fop->addr, timestamp, bytes2Hex((byte *)buffer, sizeof (new_packet_t)));
#endif

    // discard packets that look like a decryption error
    // so far it seems that the last byte of the packet is always 0 if decrypted correctly
    // do not insist on exact timebits if relayed in original encryption
    // note: normal relaying (with time stamp) is re-encoded in the old protocol
    if (pkt->lastbyte != 0 || pkt->needs3 != 3 || (original && pkt->timebits != (timestamp & 0x0F))) {
        Serial.println("rejecting bad decrypt");
        return false;
    }

    fop->protocol = RF_PROTOCOL_LEGACY;   // maybe should change for new protocol

    fop->timestamp = timestamp;
    fop->airborne  = (pkt->airborne > 1);
    fop->stealth   = pkt->stealth;
    fop->no_track  = pkt->no_track;

    fop->msg_type = MSG_TYPE_NEW;
    fop->aircraft_type = pkt->aircraft_type;
//Serial.printf("AC type: %d\n", fop->aircraft_type);
    fop->addr_type = pkt->addr_type;

    int alt = descale(pkt->alt,12,1,0) - 1000;
    fop->altitude = (float) alt - this_aircraft->geoid_separation;

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;

    int32_t round_lat;
    if (ref_lat < 0.0)
        round_lat = -(((int32_t) (-ref_lat * 1e7) + 26) / 52);
    else
        round_lat = ((int32_t) (ref_lat * 1e7) + 26) / 52;
    int32_t ilat = pkt->lat;
    ilat = (ilat - round_lat) & 0x0FFFFF;
    if (ilat >= 0x080000) ilat -= 0x100000;
    float lat = (float)((ilat + round_lat) * 52) * 1e-7;
//Serial.printf("latitude: %f\n", lat);
    fop->latitude = lat;

    int d = londiv((int)fabs(lat));

    int32_t round_lon;
    if (ref_lon < 0.0)
        round_lon = -(((int32_t) (-ref_lon * 1e7) + (d>>1)) / d);
    else
        round_lon = ((int32_t) (ref_lon * 1e7) + (d>>1)) / d;
    int32_t ilon = pkt->lon;
    ilon = (ilon - round_lon) & 0x0FFFFF;
    if (ilon >= 0x080000) ilon -= 0x0100000;
    float lon = (float)((ilon + round_lon) * d) * 1e-7;
//Serial.printf("longitude: %f\n", lon);
    fop->longitude = lon;

    int32_t turnrate = descale(pkt->turnrate,6,2,1);
    fop->turnrate = 0.05 * (float)turnrate;   // hopefully deg/sec
//Serial.printf("turnrate: %.1f deg/sec\n", fop->turnrate);

    uint32_t speed10 = descale(pkt->speed,8,2,0);
    fop->speed = (1.0 / (10.0 * _GPS_MPS_PER_KNOT)) * (float)speed10;
//Serial.printf("speed10: %d/10 mps\n", speed10);           // speed, in units of tenths of meters per second
//Serial.printf("speed: %.1f knots\n", fop->speed);

    int vs10 = descale(pkt->vs,6,2,1);
    fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
//Serial.printf("VS: %.0f fpm\n", fop->vs);

    int course = ((((uint32_t)pkt->course) & 0x03FF) >> 1);    // course as degrees 0-360
    fop->course = (float) course;
//Serial.printf("course: %d\n", course);

    fop->gpsA = descale(pkt->gpsA,3,3,0) / 10;
//Serial.printf("gpsA: 0x%X -> %d\n", pkt->gpsA, fop->gpsA);
    fop->gpsB = descale(pkt->gpsB,2,3,0) >> 2;
//Serial.printf("gpsB: 0x%X -> %d\n", pkt->gpsB, fop->gpsB);

    int32_t unk8 = pkt->unk8;
//Serial.printf("unk8: %d  %x\n", unk8);

    if (testmode_enable)
    Serial.printf("fld,%06X,%ld,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
      fop->addr, timestamp, lat, lon, alt, turnrate, speed10,
        vs10, course, fop->gpsA, fop->gpsB, pkt->airborne, unk8 );

    fop->hour   = this_aircraft->hour;
    fop->minute = this_aircraft->minute;
    fop->second = this_aircraft->second;

    // should do some sanity checks on the data
    // and return false if bad data
    bool bad = false;
    if (fabs(fop->latitude - this_aircraft->latitude) > 2.0
     || fabs(fop->longitude - this_aircraft->longitude) > 3.0)
        bad = true;
    if (course > 360 || speed10 > 1500 || abs(vs10) > 300 || fabs(fop->turnrate) > 60.0)
        bad = true;
    if (fop->airborne == 0 && (vs10 > 100 || vs10 < -100))
        bad = true;
    if (bad) {
        Serial.println("implausible data - rejecting packet");
        return false;
    }

    return true;
}


bool legacy_decode(void* buffer, ufo_t* this_aircraft, ufo_t* fop)
{
    String msg;

    legacy_packet_t* pkt = (legacy_packet_t *) buffer;

    fop->addr = pkt->addr;     /* first 32 bits are not encrypted */
//Serial.printf("decoding pkt, msg_type=%X, addr=%06X\r\n", pkt->msg_type, pkt->addr);

    fop->relayed = false;

    fop->rssi  = RF_last_rssi;
    fop->bec   = RF_last_bec;

    bool traffic_msg = (pkt->msg_type == MSG_TYPE_OLD || pkt->msg_type == MSG_TYPE_NEW);

    // reject inappropriate packets depending on mode

    if (ognrelay_enable) {

        if (!traffic_msg) {    // relayed, or special FLARM packet
          ++other_packets_recvd;
Serial.printf("received non-traffic pkt, msg_type=%X\r\n", pkt->msg_type);
          return false;
        }

    } else if (ognrelay_base) {

        if (pkt->msg_type == MSG_TYPE_ORG) {    // original was in old protocol
            ++old_protocol_packets_recvd;
            pkt->msg_type = MSG_TYPE_REL;
        }
        if (pkt->msg_type != MSG_TYPE_RAW && pkt->msg_type != MSG_TYPE_REL) {  /* other than relayed packets */
          ++other_packets_recvd;
Serial.printf("received non-relay pkt, msg_type=%X, addr=%06X\r\n", pkt->msg_type, pkt->addr);
          return false;          /* ignore normal packets */
        }
        /* fall through and decode relayed packets */

    } else {    /* single station mode */

        if (!traffic_msg) {    /* not normal traffic - ignore */
          ++other_packets_recvd;
//Serial.printf("received non-traffic pkt, msg_type=%X\r\n", pkt->msg_type);
          return false;
        }
        /* fall through and decode normal packets */
    }

//Serial.println("decoding traffic packet...");

    //uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
    uint32_t timestamp = (uint32_t) RF_time;

    if (ognrelay_enable && ! ogn_gnsstime && ! have_reverse_time) {
        /* no time data, cannot decrypt */
        fop->timestamp = timestamp;
        fop->stealth  = 0;          /* for queueing in traffic.cpp */
        fop->no_track = 0;
//Serial.println("returning un-decrypted packet");
        return true;
    }

    if (pkt->msg_type == MSG_TYPE_NEW) {
        if (ognrelay_enable && !ogn_gnsstime && !have_reverse_time)
            return true;        // will relay the encrypted packet
    }

    if (pkt->addr_type > 3) {
        // probably air-relayed by SoftRF
        //   but do some sanity checks below
        ++air_relayed_packets_recvd;
        if (!ognrelay_enable) {
            fop->relayed = true;
            pkt->addr_type &= 3;
        }
    }

    if (pkt->msg_type == MSG_TYPE_NEW || pkt->msg_type == MSG_TYPE_RAW) {
        // original packet in new protocol, or relayed in original encryption
        return latest_decode(buffer, this_aircraft, fop);
    }

    if (pkt->msg_type == MSG_TYPE_OLD && !fop->relayed)     // original in old protocol
        ++old_protocol_packets_recvd;

    fop->msg_type = MSG_TYPE_OLD;   // old protocol, either original or relayed

    bool time_sent = (ognrelay_base && (pkt->msg_type == MSG_TYPE_REL));
          // relayed with timestamp from remote station

    /* decrypt packet */
    uint32_t key[4];
    if (time_sent) {
        /* relayed packets have different encryption key if remote has time data */
        make_relay_key(key);
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

        unsigned int bec_rssi = (unsigned int) pkt->ew[1];     /* sent from remote */
        fop->bec  = (bec_rssi >> 6) & 0x03;
        fop->rssi = (bec_rssi & 0x3F);
        fop->rssi -= 101;
    }

    // fully decode the packet

    uint16_t vs_u16 = pkt->vs;
    int16_t  vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1 << 9) ? 0xFC00U : 0));
    int16_t  vs10   = vs_i16 << pkt->smult;

    /* FLARM sometimes sends packets with implausible data */
    if (pkt->airborne == 0 && (vs10 > 100 || vs10 < -100))
        return false;

    fop->timestamp = timestamp;
    fop->addr_type = pkt->addr_type;
    fop->stealth   = pkt->stealth;
    fop->no_track  = pkt->no_track;

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

    int32_t ns = (((int32_t) pkt->ns[0]) << pkt->smult);      /* quarter-meters per sec */
    int32_t ew = (((int32_t) pkt->ew[0]) << pkt->smult);
    float speed4 = sqrtf((float)(ew * ew + ns * ns));

    float direction = 0;
    float turnrate = 0;
    if (speed4 > 0)
    {
        direction = atan2f(ew, ns) * 180.0 / PI; /* -180 ... 180 */
        ns = (((int32_t) pkt->ns[1]) << pkt->smult);
        ew = (((int32_t) pkt->ew[1]) << pkt->smult);
        turnrate = (atan2f(ew, ns) * 180.0 / PI) - direction;
        if (turnrate >  180.0)  turnrate -= 360.0;
        if (turnrate < -180.0)  turnrate += 360.0;
        if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_TOWPLANE)         // known 4-second intervals
            turnrate *= 0.25;    // change was over 4 seconds
        else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_DROPPLANE)   // unverified assumptions
            turnrate *= 0.25;    // change was over 4 seconds
        else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_POWERED)
            turnrate *= 0.25;    // change was over 4 seconds
        else if (pkt->_unk2 != 1)    // typically gliders
            turnrate *= 0.333;   // change was over 3 seconds
        else
            turnrate *= 0.5;     // change was over 2 seconds
    }

    // additional sanity checks
    if (speed4 > 600.0)
            return false;
    if (abs(vs10) > 300)
            return false;
    if (fabs(turnrate) > 60.0)
            return false;

    int16_t alt = pkt->alt;  /* relative to WGS84 ellipsoid */

    fop->protocol = RF_PROTOCOL_LEGACY;

    fop->addr_type     = pkt->addr_type;
    fop->latitude      = lat;
    fop->longitude     = lon;
    fop->altitude      = (float) alt - geo_separ;
    fop->speed         = speed4 * (1.0 / (4 * _GPS_MPS_PER_KNOT));
    fop->course        = (direction >= 0.0 ? direction : direction + 360);
    fop->vs            = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
    fop->turnrate      = turnrate;
    fop->aircraft_type = pkt->aircraft_type;
    fop->gpsA = 3;
    fop->gpsB = 5;   // fake

    /* set hour, minute, second for OGN reporting of this packet */

    if (! time_sent) {
      fop->hour   = this_aircraft->hour;
      fop->minute = this_aircraft->minute;
      fop->second = this_aircraft->second;
    }

    if (ognrelay_enable) {
      if (!ogn_gnsstime) {
        // >>> do some sanity checks here in case reverse_time is wrong
        //     and thus the decrypted data is random garbage
        // - if we knew what is in the "gps" field could check that
        if (pkt->stealth)   return false;
        if (pkt->no_track)  return false;
        //if (pkt->airborne == 0)  return false;
      }
      /* packet will be relayed in Traffic.cpp using legacy_encode() below */
      return true;
    }

    if (ognrelay_base) {

      if (!ogn_gnsstime && !ognrelay_time) {
        // >>> do some sanity checks here in case NTP time is off by a second
        //     and thus the decrypted data is random garbage
        // - if we knew what is in the "gps" field could check that
        if (pkt->stealth)   return false;
        if (pkt->no_track)  return false;
        //if (pkt->airborne == 0)  return false;
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

    return true;
}

// re-encode packets to be relayed
size_t legacy_encode_data(void* legacy_pkt, ufo_t* fop)
{
    legacy_packet_t* pkt = (legacy_packet_t *) legacy_pkt;

    int      ndx;
    uint8_t  pkt_parity=0;
    uint32_t key[4];
    uint32_t timestamp = (uint32_t) RF_time;

    uint32_t id  = fop->addr;
    float    lat = fop->latitude;
    float    lon = fop->longitude;
    int16_t  alt = (int16_t) (fop->altitude + ogn_geoid_separation);

    float course = fop->course;
    float speedf = fop->speed * _GPS_MPS_PER_KNOT;         /* m/s */
    float vsf    = fop->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

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

    int8_t ns = 0;  // (int8_t) (speed * cosf(radians(course)));
    int8_t ew = 0;  // (int8_t) (speed * sinf(radians(course)));

    pkt->vs = vs10;

    pkt->addr = id & 0x00FFFFFF;

    pkt->addr_type = fop->addr_type;
    pkt->aircraft_type = fop->aircraft_type;
    pkt->stealth  = fop->stealth;
    pkt->no_track = fop->no_track;

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

    pkt->airborne = fop->airborne;

    pkt->_unk1 = 0;
    pkt->_unk2 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

    // do not encrypt yet

    return sizeof(legacy_packet_t);
}

/* special encoding for relayed packets */

size_t legacy_encode(void* buffer, ufo_t* fop)
{
    legacy_packet_t* pkt = (legacy_packet_t *) buffer;

    if (ognrelay_enable && (ogn_gnsstime || have_reverse_time)) {

        // re-encode the data into the old protocol (even if arrived in new protocol)
        legacy_encode_data(buffer, fop);

        pkt->ns[1] = fop->hour;
        pkt->ns[2] = fop->minute;
        pkt->ns[3] = fop->second;

        int rssi = fop->rssi + 101;
        if (rssi <  0)  rssi =  0;
        if (rssi > 63)  rssi = 63;
        int bec = fop->bec;
        if (bec > 3)    bec = 3;
        pkt->ew[1] = (bec << 6) | rssi;

        pkt->msg_type = (fop->msg_type == MSG_TYPE_OLD? MSG_TYPE_ORG : MSG_TYPE_REL);
            /* mark as relayed packet with a timestamp */

        int ndx;
        pkt->ew[2] = 0;
        pkt->ew[3] = 0;
        uint16_t checksum=0;
        for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++)
            checksum += (*(((unsigned char *) pkt) + ndx));
        pkt->ew[2] = (checksum & 0x3F80) >> 7;
        pkt->ew[3] = checksum & 0x007F;

        /* re-encrypt with relay key */
        uint32_t key[4];
        make_relay_key(key);
        uint32_t *p = (uint32_t *) pkt;
        btea(p+1, 5, key);

        return sizeof(legacy_packet_t);

    } else if (fop->msg_type == MSG_TYPE_NEW) {  // new protocol only

        /* do not re-encode, it is still in the original FLARM encryption */

        memcpy(buffer, fop->raw, sizeof(legacy_packet_t));

        /* use bits in the first - unencrypted - word as flags */
        pkt->msg_type = MSG_TYPE_RAW;       /* does not have a timestamp */
        pkt->_unk1 = (ThisAircraft.timestamp > fop->timestamp + 12) ? 1 : 0;

        return sizeof(legacy_packet_t);
    }

    return 0;    // if relay station with no time source, ignore old protocol
}
