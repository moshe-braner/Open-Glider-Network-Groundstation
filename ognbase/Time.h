/*
 * Time.h
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

#ifndef TIMEHELPER_H
#define TIMEHELPER_H

extern time_t OurTime;    /* derived from GNSS */
extern uint32_t ref_time_ms;
extern uint16_t have_reverse_time;
extern bool reverse_time_sync;

extern bool time_synched;
extern bool NTP_synched;
extern uint32_t traffic_packets_recvd;
extern uint32_t old_protocol_packets_recvd;
extern uint32_t air_relayed_packets_recvd;
extern uint32_t traffic_packets_relayed;
extern uint32_t traffic_packets_reported;
extern uint32_t other_packets_recvd;
extern uint16_t bad_packets_recvd;
extern uint16_t time_packets_sent;
extern uint16_t time_packets_recvd;
extern uint16_t ack_packets_sent;
extern uint16_t ack_packets_recvd;
extern uint16_t sync_restarts;
extern uint32_t sleep_when, sleep_length;
extern uint32_t when_synched;
extern uint32_t when_sync_sent;
extern uint32_t when_to_switch;
extern bool got_time_ack;

extern uint32_t remote_traffic, remote_other;
extern uint16_t remote_timesent, remote_bad;
extern uint8_t remote_sats, remote_pctrel, remote_ack, remote_restarts, remote_round;
extern int8_t remote_rssi;
extern uint32_t uptime, remote_uptime;
extern uint16_t remote_sleep_length;
extern float remote_voltage;
extern uint16_t packets_per_minute;
extern time_t last_hour;

bool time_sync_pkt(uint8_t*);
void set_our_clock(uint8_t*);
void sync_alive_pkt(uint8_t*);
bool reboot_remote(void);
bool maybe_remote_reboot(uint8_t*);
void Timesync_restart(void);

void Time_setup(void);
void Time_loop(void);
void Poll_NTP(void);

#endif /* TIMEHELPER_H */
