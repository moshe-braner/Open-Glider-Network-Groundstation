/*
 * Traffic.cpp
 * Copyright (C) 2018-2020 Linar Yusupov
 *
 * Traffic-relay prioritization code by Moshe Braner, 2022
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

#include "Traffic.h"
#include "EEPROM.h"
#include "RF.h"
#include "Time.h"
#include "GNSS.h"
#include "Web.h"
#include "Protocol_Legacy.h"
#include "OLED.h"
#include "global.h"
#include "Log.h"

#include <math.h>


uint32_t UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
#define MAX_LISTED (3*MAX_TRACKING_OBJECTS)
id_list_entry_t id_list[MAX_LISTED];
id_list_entry_t *id_list_head = NULL;
uint8_t numtracked = 0;
uint16_t numseen_ever = 0;
uint16_t numseen_today = 0;
uint16_t numseen_1hr = 0;
uint16_t numvisible = 0;

/* closed-ring doubly-linked list stored in static array */
void id_list_setup()
{
    id_list_head = &id_list[0];
    for (int i=0; i<MAX_LISTED; i++) {
       id_list[i].addr = 0;
       id_list[i].timestamp = 0;
       id_list[i].invisible = 0;
       id_list[i].prev = (i>0? &id_list[i-1] : &id_list[MAX_LISTED-1]);
       id_list[i].next = (i<MAX_LISTED-1? &id_list[i+1] : &id_list[0]);
    }
}

/* not actually used at this time */
void id_list_clear()
{
    numseen_ever = 0;
    numseen_today = 0;
    numseen_1hr = 0;
    numvisible = 0;
    id_list_setup();
}

/* search for a given aircraft ID in the list */
id_list_entry_t *find_id(uint32_t addr)
{
    if (!id_list_head)  return NULL;   /* should not happen */
    id_list_entry_t *p = id_list_head;
    while (p->addr != addr) {
        if (p->addr == 0)             /* rest of list is empty */
            return NULL;
        if (p->next == id_list_head)  /* end of list, not found */
            return NULL;
        p = p->next;
    }
    return p;
}

/* move an already-listed aircraft to top of list */
void refresh_id(ufo_t *fop)
{
    if (!id_list_head)          /* should not happen */
        return;
    id_list_entry_t *p = fop->listed;
    if (p == NULL)              /* should not happen */
        return;
    if (p->addr != fop->addr)   /* should not happen */
        return;
    p->timestamp = fop->timestamp;
    p->invisible = (/* fop->stealth || */ fop->no_track);
    if (p == id_list_head)      /* already at top of list */
        return;
    // de-link from list
    p->prev->next = p->next;
    p->next->prev = p->prev;
    // re-link at top
    id_list_entry_t *h = id_list_head;
    p->prev = h->prev;
    p->next = h;
    h->prev->next = p;
    h->prev = p;
    // re-point head
    id_list_head = p;
}

/* add a new aircraft as top of list */
void add_id(ufo_t *fop)
{
    if (!id_list_head)     /* should not happen */
        return;
    // overwrite last entry in list - no need to change the linking
    id_list_entry_t *p = id_list_head->prev;
    id_list_head = p;
    fop->listed = p;
    p->addr = fop->addr;
    p->timestamp = fop->timestamp;
    p->invisible = (/* fop->stealth || */ fop->no_track);
}

/* count how many aircraft were seen in the last hour */
/*
 * Some invisibles not counted because remote (if full array) did not
 *  overwrite visible traffic thus did not relay them to base station.
 * But base station adds all new aircraft to this list even if not
 *  adding them into tracking array.
 */
void count_ids_seen()
{
    if (!id_list_head)  return;       /* should not happen */
    numseen_ever = 0;
    numseen_today = 0;
    numseen_1hr = 0;
    numvisible = 0;
    int localtime = ThisAircraft.hour + ogn_timezone;
    if (localtime > 23)  localtime -= 24;
    if (localtime <  0)  localtime += 24;
    time_t day_cutoff = (localtime+1)*3600;
    if (OurTime < day_cutoff)  return;   /* should not happen */
    day_cutoff = OurTime - day_cutoff;
    time_t hour_cutoff = OurTime - 3600;
    id_list_entry_t *p = id_list_head;
    while (p->addr) {
        ++numseen_ever;
        if (p->timestamp > day_cutoff) {
            ++numseen_today;
            if (p->timestamp > hour_cutoff) {
                ++numseen_1hr;
                if (! p->invisible)
                    ++numvisible;
            }
        } else {
            // the rest are from previous days
            // truncate numseen_ever count if list is full
            if (id_list_head->prev->addr != 0) {
                numseen_ever = 999;   // overflow
                return;
            }
            //if (p->timestamp < day_cutoff)
            //    p->addr = 0;
        }
        if (p->next == id_list_head) {  /* end of list */
            /* list is full and all from today */
            numseen_ever  = 999;
            numseen_today = 999;
            return;
        }
        p = p->next;
    }
}

static int8_t (* Alarm_Level)(ufo_t *, ufo_t *);

/*
 * No any alarms issued by the firmware.
 * Rely upon high-level flight management software.
 */
static int8_t Alarm_None(ufo_t* this_aircraft, ufo_t* fop)
{
    return ALARM_LEVEL_NONE;
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(ufo_t* this_aircraft, ufo_t* fop)
{
    int    distance = (int) fop->distance;
    int8_t rval     = ALARM_LEVEL_NONE;
    int    alt_diff = (int) (fop->altitude - this_aircraft->altitude);

    if (abs(alt_diff) < VERTICAL_SEPARATION) /* no warnings if too high or too low */
    {
        if (distance < ALARM_ZONE_URGENT)
            rval = ALARM_LEVEL_URGENT;
        else if (distance < ALARM_ZONE_IMPORTANT)
            rval = ALARM_LEVEL_IMPORTANT;
        else if (distance < ALARM_ZONE_LOW)
            rval = ALARM_LEVEL_LOW;
    }

    return rval;
}

/*
 * EXPERIMENTAL
 *
 * Linear, CoG and GS based collision prediction.
 */
static int8_t Alarm_Vector(ufo_t* this_aircraft, ufo_t* fop)
{
    int8_t rval = ALARM_LEVEL_NONE;

    /* TBD */

    return rval;
}

/*
 * "Legacy" method is based on short history of 2D velocity vectors (NS/EW)
 */
static int8_t Alarm_Legacy(ufo_t* this_aircraft, ufo_t* fop)
{
    int8_t rval = ALARM_LEVEL_NONE;

    /* TBD */

    return rval;
}

/* the following two CAN be different although for now are the same: */
#define relay_interval()   ((time_t)(numtracked <= 4 ? 4 : numtracked))
#define report_interval()  ((time_t)(numtracked <= 4 ? 4 : numtracked))

void calc_distance(ufo_t* fop)
{
    static float coslat = 0;
    if (coslat == 0)  coslat = cosf(ThisAircraft.latitude * 0.0174533);\

//  fop->distance = gnss.distanceBetween(ThisAircraft.latitude,
//                                       ThisAircraft.longitude,
//                                       fop->latitude,
//                                       fop->longitude);
  float dy = 111300.0 * (fop->latitude - ThisAircraft.latitude);     /* meters */
  float dx = 111300.0 * (fop->longitude - ThisAircraft.longitude) * coslat;
  fop->distance = hypot(dx, dy);
}

void ParseData()
{
    size_t rx_size = RF_Payload_Size(ogn_protocol_1);
    if (rx_size > sizeof(fo.raw)) rx_size = sizeof(fo.raw);

    // fo = EmptyFO;   /* all zeros */
    // memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

legacy_packet_t* p = (legacy_packet_t *) fo.raw;
//Serial.printf("parsing packet: %X %06X\r\n", p->msg_type, p->addr);

    if (time_sync_pkt(fo.raw)) {
//Serial.println("handling time-sync packet");

        if (time_client)
            set_our_clock(fo.raw);    /* may set time_synched */

        else if (time_master)
            sync_alive_pkt(fo.raw);   /* may set time_synched */

        return;
    }

    if (ognrelay_enable) {
        if (maybe_remote_reboot(fo.raw)) {
            /* a reboot packet, but did not reboot */
Serial.println("got a reboot packet, but did not reboot");
            return;
        }
        /* else, it was not a remote-reset packet */
        /* fall through to process as a traffic packet */
    }

    /* until time-synched any received packets other than time-sync are ignored */
    if (OurTime < 1000000 || ((ognrelay_time || ognreverse_time) && ! time_synched)) {
        // ++other_packets_recvd;
Serial.println("ignoring non-time-sync packet");
        return;
    }

    /* otherwise fall through to process normal traffic packets */

//    ++traffic_packets_recvd;

//Serial.println("parse calling decode...");

    if (protocol_decode && (*protocol_decode)((void *) fo.raw, &ThisAircraft, &fo))
    {
        int i, age, oldest, oldest_age;
        time_t timenow;
        ufo_t* cip;

        ++traffic_packets_recvd;

//Serial.println("parsing decoded packet...");

        /* timenow = now(); */
        timenow = OurTime;
        // fo.timestamp = timenow;    /* timestamp (and H,M,S) was set in legacy_decode() */

        if (! ognrelay_enable) {
            calc_distance(&fo);
            if (fo.distance /*m*/ > 1200 * ogn_range /*km*/) {
Serial.println("too far, rejected");
                return;               /* too far, or perhaps corrupted data */
            }
            // this is another sanity checks in case of wrong de-cryption
            //  - packet_valid() in APRS reporting is another check
        }

        /* if an already-tracked aircraft, update and report */
        /* - but if reported recently, ignore for now */

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

            cip = &Container[i];
            if (cip->addr == fo.addr) {               /* this aircraft already tracked */

              if (!ognrelay_enable || ogn_gnsstime || ognreverse_time) {   /* packet is de-crypted */
                /* ignore duplicate FLARM packets (there are many!) */
                if (fo.altitude == cip->altitude &&
                    fo.latitude == cip->latitude &&
                    fo.longitude == cip->longitude) {
                       if (timenow - cip->timestamp < 12) {
Serial.println("duplicate packet, rejected");
                              return;
                       }
                }
              }

              if (ognrelay_enable) {  /* remote station */

                  fo.timereported = cip->timereported;   /* last time it was relayed */
                  fo.waiting = (timenow >= fo.timereported + relay_interval());
                  *cip = fo;         /* copy the whole struct, including new timestamp, etc */

              } else {  /* base or standalone station */

                  fo.prevtime = cip->timestamp;      /* keep some data from previous packet...   */
                  fo.prevlat = cip->latitude;        /* ... in order to check data in PVALID.cpp */
                  fo.prevlon = cip->longitude;
                  fo.prevalt = cip->altitude;
                  fo.timereported = cip->timereported;  /* last time it was reported to OGN */
                  fo.waiting = (timenow >= fo.timereported + report_interval());

//Serial.printf("[%d] recvd again, was reported at %d, timest %02d:%02d:%02d, now waiting: %d\r\n",
//   i, fo.timereported, fo.hour, fo.minute, fo.second, fo.waiting);

                  fo.listed = cip->listed;
                  *cip = fo;         /* copy the whole struct, including new timestamp, etc */
                  refresh_id(cip);
              }
//Serial.printf("[%d] received again at %d, now waiting: %d\r\n", i, timenow, cip->waiting);
              return;
            }
        }

        /* new aircraft, not currently in the Container[] array */

        fo.prevtime = 0;              /* no previous data */
        fo.waiting = true;
        fo.timereported = 0;          /* never been relayed or reported */

        if (! ognrelay_enable) {
            id_list_entry_t *ilp = find_id(fo.addr);  /* might still be there from past */
            if (ilp) {
                fo.listed = ilp;
                refresh_id(&fo);
            } else {
                add_id(&fo);
            }
        }

        /* put it in an empty slot if available */
        /* if no empty slot, replace the oldest */

        oldest = 0;
        oldest_age = 0;
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
            cip = &Container[i];
            if (cip->addr == 0) {                     /* empty slot */
                *cip = fo;
//Serial.printf("[%d] was empty at %d, now waiting: %d\r\n", i, timenow, cip->waiting);
                ++numtracked;
                return;
            }
            age = (int) (timenow - cip->timestamp);   /* not ->timereported */
            if (age < ENTRY_EXPIRATION_TIME && fo.timereported == 0)  /* not previously reported */
                age = 0;                              /* do not overwrite */
            if (! cip->waiting)                       /* treat non-waiting as older */
                age += ENTRY_EXPIRATION_TIME;
            if (cip->stealth)
                age += ENTRY_EXPIRATION_TIME;     /* treat stealth as older yet */
            if (cip->no_track)
                age += 2 * ENTRY_EXPIRATION_TIME; /* treat no_track as very old */
            if (age > oldest_age) {
                oldest_age = age;
                oldest = i;
            }
        }

        if (oldest_age < ENTRY_EXPIRATION_TIME && fo.stealth)
            return;   /* drop the new traffic instead of replacing old traffic */
        if (oldest_age < 2*ENTRY_EXPIRATION_TIME && fo.no_track)
            return;   /* drop it */

        if (oldest_age > 0) {
            Container[oldest] = fo;        /* overwrites older data */
//Serial.printf("[%d] overwritten at %d, now waiting: %d\r\n", i, timenow, Container[oldest].waiting);
        }
    } else {
//Serial.println("decode returned false");
    }
}

void Traffic_setup()
{
    switch (settings->alarm)
    {
        case TRAFFIC_ALARM_NONE:
            Alarm_Level = &Alarm_None;
            break;
        case TRAFFIC_ALARM_VECTOR:
            Alarm_Level = &Alarm_Vector;
            break;
        case TRAFFIC_ALARM_LEGACY:
            Alarm_Level = &Alarm_Legacy;
            break;
        case TRAFFIC_ALARM_DISTANCE:
        default:
            Alarm_Level = &Alarm_Distance;
            break;
    }

    id_list_setup();
}

void Traffic_Relay(ufo_t* fop)
{
    if (RF_Transmit(RF_Encode(fop), true)) {   /* success transmitting */
        fop->waiting = false;
//Serial.printf("... relayed at %d, prev report %d, timestamp %02d:%02d:%02d\r\n",
//   OurTime, fop->timereported, fop->hour, fop->minute, fop->second);
        /* set time stamp for next update of same aircraft */
        fop->timereported = OurTime;
        ++traffic_packets_relayed;
Serial.println("... relayed");
    }
}

void Traffic_loop()
{
    int i, waiting, oldest_waiting, age, oldest_age;
    time_t timenow;
    ufo_t* cip;

    timenow = OurTime;
    numtracked = 0;
    waiting = 0;
    oldest_waiting = 0;
    oldest_age = -1;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        cip = &Container[i];
        if (cip->addr) {
            ++numtracked;                      /* count tracked objects */
            if (cip->waiting) {
                ++waiting;                     /* count objects waiting to be reported */
                age = (int) (timenow - cip->timereported);   /* not ->timestamp */
                if (age > oldest_age) {
                   oldest_age = age;
                   oldest_waiting = i;
                }
            }
//if (ognrelay_base)
//Serial.printf("[%d] iswaiting: %d, age: %d\r\n", i, cip->waiting, age);
        }
    }

    if (ognrelay_enable) {
      if (oldest_age >= 0)
        Traffic_Relay(&Container[oldest_waiting]);
    }

    static uint32_t traffic_msg_time = 0;
    if (millis() > traffic_msg_time + 60000) {
        Serial.printf("%d recvd, %d trkd, %d waiting, oldest: %d\r\n",
              traffic_packets_recvd, numtracked, waiting, oldest_age);
        traffic_msg_time = millis();
    }

    if (! ognrelay_enable) {
        static uint32_t count_ids_time = 0;
        if (millis() > count_ids_time + 22500) {
            count_ids_time = millis();
            count_ids_seen();
        }
    }
}

void ClearExpired()
{
    time_t timenow = OurTime;
    numtracked = 0;
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr) {
          if ((timenow - Container[i].timestamp) > ENTRY_EXPIRATION_TIME)
              Container[i].addr = 0;
          else
              ++numtracked;
        }
    }
}
