/*
 * Web.h
 * Copyright (C) 2020 Manuel Rösel
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

#ifndef WEBHELPER_H
#define WEBHELPER_H

#include "SoC.h"

#if defined(ARDUINO) && !defined(EXCLUDE_WIFI)
#include <WiFiClient.h>
#endif /* ARDUINO */

#include <TinyGPS++.h>

#include "EEPROM.h"
#include "RF.h"

#define BOOL_STR(x) (x ? "true":"false")
#define JS_MAX_CHUNK_SIZE 4096

void Web_setup(ufo_t* this_aircraft);

void Web_loop(void);

void Web_fini(void);

void Web_start(void);
void Web_stop(void);

#if DEBUG
void Hex2Bin(String, byte *);
#endif

extern uint32_t tx_packets_counter, rx_packets_counter;
//extern byte TxBuffer[PKT_SIZE];
extern String TxDataTemplate;

#if defined(ARDUINO) && !defined(EXCLUDE_WIFI)
extern WiFiClient client;
#endif /* ARDUINO */

extern TinyGPSPlus gps;

#endif /* WEBHELPER_H */
