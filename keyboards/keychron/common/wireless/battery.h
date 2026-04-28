/* Copyright 2022 ~ 2025 @ lokher (https://www.keychron.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "config.h"

enum {
    BAT_NOT_CHARGING = 0,
    BAT_CHARGING,
    BAT_FULL_CHARGED,
};

#ifndef FULL_VOLTAGE_VALUE
#    define FULL_VOLTAGE_VALUE 4100
#endif

#ifndef EMPTY_VOLTAGE_VALUE
#    define EMPTY_VOLTAGE_VALUE 3500
#endif

#ifndef SHUTDOWN_VOLTAGE_VALUE
#    define SHUTDOWN_VOLTAGE_VALUE 3300
#endif

#ifndef VOLTAGE_MEASURE_INTERVAL
#    define VOLTAGE_MEASURE_INTERVAL 3000
#endif

#ifndef VOLTAGE_POWER_ON_MEASURE_COUNT
#    define VOLTAGE_POWER_ON_MEASURE_COUNT 15
#endif

#ifndef BACKLIGHT_OFF_VOLTAGE_MEASURE_INTERVAL
#    define BACKLIGHT_OFF_VOLTAGE_MEASURE_INTERVAL 200
#endif

void battery_init(void);
void battery_stop(void);

void     battery_measure(void);
void     battery_calculate_voltage(bool vol_src_bt, uint16_t value);
void     battery_set_voltage(uint16_t value);
uint16_t battery_get_voltage(void);
uint8_t  battery_get_percentage(void);
bool     battery_is_empty(void);
bool     battery_is_critical_low(void);

/* Charge state machine, exposed over HID (KC_GET_BATTERY_LEVEL byte 2). */
enum {
    BAT_CHARGE_STATE_DISCHARGING = 0, /* no USB power source */
    BAT_CHARGE_STATE_CHARGING    = 1, /* USB power present, GPIO says actively charging */
    BAT_CHARGE_STATE_FULL        = 2, /* USB power present, GPIO says topped off */
};
uint8_t  battery_get_charge_state(void);

/* Push notification (KC_PUSH_BATTERY_NOTIFY = 0xAD) — host doesn't poll,
 * keyboard reports changes asynchronously. battery_push_check() runs
 * every battery_task() tick: leading-edge fire on state change, plus
 * trailing-edge flush after a debounce window for bursty transitions.
 * No heartbeat — host gets liveness from the dongle's link events.
 * battery_push_force() resets the "last sent" cache so the next check
 * resends current state — used on EVT_CONNECTED to re-prime the host
 * after the keyboard reconnects to the dongle. */
void     battery_push_check(void);
void     battery_push_force(void);

bool     battery_power_on_sample(void);
void     battery_timer_reset(void);
void     battery_task(void);
