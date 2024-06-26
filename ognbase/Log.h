/*
 * Log.h
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

#ifndef LOGHELPER_H
#define LOGHELPER_H

#include "SoftRF.h"

void Logger_send_udp(String *);
void Logger_send_enc_udp(String *);

#include "SPIFFS.h"
extern File DebugLog;
extern bool DebugLogOpen;
void OpenDebugLog();
void DebugLogWrite(const char *s);
void LogDate();

#if LOGGER_IS_ENABLED

#include <FS.h>

extern File LogFile;
void Logger_setup(void);

void Logger_loop(void);

void Logger_fini(void);

#endif /* LOGGER_IS_ENABLED */

#endif /* LOGHELPER_H */
