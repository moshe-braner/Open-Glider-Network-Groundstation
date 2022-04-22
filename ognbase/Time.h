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

extern bool time_synched;
extern uint32_t traffic_packets_recvd;
extern uint32_t traffic_packets_relayed;
extern uint32_t traffic_packets_reported;
extern uint16_t bad_packets_recvd;
extern uint16_t other_packets_recvd;
extern uint16_t time_packets_sent;
extern uint16_t ack_packets_recvd;
extern uint16_t sync_restarts;

extern uint32_t remote_traffic;
extern uint16_t remote_timesent, remote_bad, remote_other;
extern uint8_t remote_sats, remote_dropped, remote_noack, remote_restarts, remote_round;

bool time_sync_pkt(uint8_t*);
void set_our_clock(uint8_t*);
void sync_alive_pkt(uint8_t*);

void Time_setup(void);
void Time_loop(void);

#endif /* TIMEHELPER_H */
