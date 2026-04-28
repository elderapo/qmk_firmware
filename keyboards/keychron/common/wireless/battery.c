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

#include "quantum.h"
#include "wireless.h"
#include "battery.h"
#include "transport.h"
#if defined(LK_WIRELESS_ENABLE)
#    include "lkbt51.h"
#elif defined(KC_BLUETOOTH_ENABLE)
#    include "ckbt51.h"
#endif
#include "lpm.h"
#include "indicator.h"
#include "rtc_timer.h"
#include "analog.h"
#include "backlit_indicator.h"
#include "config.h"
#include "keychron_raw_hid.h"

#define BATTERY_EMPTY_COUNT 10
#define CRITICAL_LOW_COUNT 20

/* Battery voltage resistive voltage divider setting of MCU */
#ifndef RVD_R1
#    define RVD_R1 10 // Upper side resitor value (uint: KΩ)
#endif
#ifndef RVD_R2
#    define RVD_R2 10 // Lower side resitor value (uint: KΩ)
#endif

/* Battery voltage resistive voltage divider setting of Bluetooth */
#ifndef LKBT51_RVD_R1
#    define LKBT51_RVD_R1 560
#endif
#ifndef LKBT51_RVD_R2
#    define LKBT51_RVD_R2 499
#endif

#ifndef VOLTAGE_TRIM_LED_MATRIX
#    define VOLTAGE_TRIM_LED_MATRIX 30
#endif

#ifndef VOLTAGE_TRIM_RGB_MATRIX
#    define VOLTAGE_TRIM_RGB_MATRIX 60
#endif

static uint32_t bat_monitor_timer_buffer = 0;
static uint16_t voltage                  = FULL_VOLTAGE_VALUE;
static uint8_t  bat_empty                = 0;
static uint8_t  critical_low             = 0;
static uint8_t  bat_state;
static uint8_t  power_on_sample = 0;

void battery_init(void) {
    bat_state = BAT_NOT_CHARGING;
#if defined(BAT_CHARGING_PIN)
#    if (BAT_CHARGING_LEVEL == 0)
    palSetLineMode(BAT_CHARGING_PIN, PAL_MODE_INPUT_PULLUP);
#    else
    palSetLineMode(BAT_CHARGING_PIN, PAL_MODE_INPUT_PULLDOWN);
#    endif
#endif

#ifdef BAT_ADC_ENABLE_PIN
    palSetLineMode(BAT_ADC_ENABLE_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    gpio_write_pin(BAT_ADC_ENABLE_PIN, 1);
#endif
#ifdef BAT_ADC_PIN
    palSetLineMode(BAT_ADC_PIN, PAL_MODE_INPUT_ANALOG);
#endif
}

void battery_stop(void) {
#if (HAL_USE_ADC)
#    ifdef BAT_ADC_ENABLE_PIN
    gpio_write_pin(BAT_ADC_ENABLE_PIN, 0);
#    endif
#    ifdef BAT_ADC_PIN
    palSetLineMode(BAT_ADC_PIN, PAL_MODE_INPUT_ANALOG);
    analog_stop(BAT_ADC_PIN);
#    endif
#endif
}

__attribute__((weak)) void battery_measure(void) {
#if defined(LK_WIRELESS_ENABLE)
    lkbt51_read_state_reg(0x05, 0x02);
#elif defined(KC_BLUETOOTH_ENABLE)
    ckbt51_read_state_reg(0x05, 0x02);
#endif
}

/* Calculate the voltage */
__attribute__((weak)) void battery_calculate_voltage(bool src_bt, uint16_t value) {
    uint16_t voltage;

#if defined(LK_WIRELESS_ENABLE)
    if (src_bt)
        voltage = ((uint32_t)value) * (LKBT51_RVD_R1 + LKBT51_RVD_R2) / LKBT51_RVD_R2;
    else
        voltage = (uint32_t)value * 3300 / 1024 * (RVD_R1 + RVD_R2) / RVD_R2;
#elif defined(KC_BLUETOOTH_ENABLE)
    voltage = ((uint32_t)value) * 2246 / 1000;
#else
    return;
#endif

#ifdef LED_MATRIX_ENABLE
    if (led_matrix_is_enabled()) {
        /* We assumpt it is linear relationship*/
        uint32_t compensation = (VOLTAGE_TRIM_LED_MATRIX * led_matrix_driver.get_total_duty_ratio());
        voltage += compensation;
    }
#endif
#ifdef RGB_MATRIX_ENABLE
    if (rgb_matrix_is_enabled()) {

        /* We assumpt it is linear relationship*/
        uint32_t compensation = (VOLTAGE_TRIM_LED_MATRIX * rgb_matrix_driver.get_total_duty_ratio());
        voltage += compensation;
    }
#endif

    battery_set_voltage(voltage);
}

void battery_set_voltage(uint16_t value) {
    voltage = value;
}

uint16_t battery_get_voltage(void) {
    return voltage;
}

uint8_t battery_get_percentage(void) {
    if (voltage > FULL_VOLTAGE_VALUE) return 100;

    if (voltage > EMPTY_VOLTAGE_VALUE) {
        return ((uint32_t)voltage - EMPTY_VOLTAGE_VALUE) * 80 / (FULL_VOLTAGE_VALUE - EMPTY_VOLTAGE_VALUE) + 20;
    }

    if (voltage > SHUTDOWN_VOLTAGE_VALUE) {
        return ((uint32_t)voltage - SHUTDOWN_VOLTAGE_VALUE) * 20 / (EMPTY_VOLTAGE_VALUE - SHUTDOWN_VOLTAGE_VALUE);
    } else
        return 0;
}

bool battery_is_empty(void) {
    return bat_empty > BATTERY_EMPTY_COUNT;
}

bool battery_is_critical_low(void) {
    return critical_low > CRITICAL_LOW_COUNT;
}

/* GPIO-pin reading that battery_task() also uses for the LKBT51 charge LED,
 * exposed for HID consumers (KC_GET_BATTERY_LEVEL byte 2). Without this,
 * hosts cannot distinguish discharging from charging via external USB-C
 * charger — the wireless battery query carries only the percentage, and
 * the wired sibling PID does not enumerate when the keyboard charges from
 * a wall adapter rather than the host. */
uint8_t battery_get_charge_state(void) {
#if defined(BAT_CHARGING_PIN)
    if (!usb_power_connected())
        return BAT_CHARGE_STATE_DISCHARGING;
    return (gpio_read_pin(BAT_CHARGING_PIN) == BAT_CHARGING_LEVEL)
               ? BAT_CHARGE_STATE_CHARGING
               : BAT_CHARGE_STATE_FULL;
#else
    return BAT_CHARGE_STATE_DISCHARGING;
#endif
}

/* --- Push notification path (KC_PUSH_BATTERY_NOTIFY = 0xAD) ---
 *
 * State sent to the host: percentage + charge_state (same layout as the
 * 0xAC query response, just on a different opcode).
 *
 * Two triggers, both routing through battery_push_check():
 *   1. Leading-edge: state changed *and* we're outside the debounce
 *      window since the previous send → fire immediately.
 *   2. Trailing-edge: state changed *during* the debounce window → defer
 *      the value into `pending_*`; once the window closes (DEBOUNCE_MS
 *      after the *most recent* change with no further change), send the
 *      latest value. Guarantees that bursty transitions never silently
 *      drop the final state.
 *
 * No heartbeat: the host driver gets liveness from the LKBT51 dongle's
 * own connect/disconnect events (the `54 e2 01 XX` reports on dongle
 * intf 1), and from the absence of those events. A periodic push would
 * just wake the keyboard from STOP mode every interval for no
 * additional information — battery percentage drift naturally fires a
 * push on every 1% change anyway.
 *
 * Two transport paths, both attempted on every push:
 *   - Wireless tunnel via lkbt51 → dongle → host. Gated by
 *     WT_CONNECTED, payload XOR-masked with 0x28 to survive the
 *     LKBT51 wireless module's mishandling of toxic bytes.
 *   - USB raw HID via the wired interface (0x0b30). Gated by the
 *     USB driver being the active host driver — true in switch=wired
 *     mode (firmware delivers HID input over cable). Plain bytes,
 *     no XOR (the wired path doesn't have the LKBT51 quirk).
 *
 * Both paths are tried every push so the host listener works in
 * whichever mode the user has the switch in, including 2.4G+cable
 * where both transports are simultaneously alive.
 *
 * battery_push_force() invalidates the "last sent" cache so the next
 * check re-emits regardless of whether the value actually changed —
 * called on EVT_CONNECTED so the host gets a fresh state right after
 * the keyboard reconnects. */
#define BATTERY_PUSH_DEBOUNCE_MS  500

static uint8_t  last_pushed_pct    = 0xFF;  /* 0xFF = never pushed */
static uint8_t  last_pushed_charge = 0xFF;
static uint32_t last_push_t        = 0;

static bool     pending_active = false;
static uint8_t  pending_pct;
static uint8_t  pending_charge;
static uint32_t pending_t;

static void battery_do_push(uint8_t pct, uint8_t charge) {
    uint8_t buf[32] = {0};
    buf[0] = KC_PUSH_BATTERY_NOTIFY;
    buf[1] = pct;
    buf[2] = charge;

    bool sent = false;

#if defined(LK_WIRELESS_ENABLE)
    if (wireless_get_state() == WT_CONNECTED) {
        extern wt_func_t wireless_transport;
        if (wireless_transport.send_raw_hid) {
            /* Don't mutate `buf` — the USB branch below needs it
             * unmodified. Build a separate XOR'd copy. */
            uint8_t xor_buf[32];
            for (uint8_t i = 0; i < 32; i++) {
                xor_buf[i] = buf[i] ^ WIRELESS_RAW_HID_XOR_KEY;
            }
            wireless_transport.send_raw_hid(xor_buf, 32);
            sent = true;
        }
    }
#endif

    /* USB cable path. host_get_driver() == &chibios_driver means USB
     * is the currently-active HID transport — true in switch=wired
     * mode and in the brief pre-pairing window after wireless mode
     * is selected. In switch=2.4G + cable plugged we deliberately
     * still send via wireless (the if above) and skip USB to avoid
     * delivering the same payload twice on the same host. */
    extern host_driver_t chibios_driver;
    if (host_get_driver() == &chibios_driver && chibios_driver.send_raw_hid) {
        chibios_driver.send_raw_hid(buf, 32);
        sent = true;
    }

    if (sent) {
        last_pushed_pct    = pct;
        last_pushed_charge = charge;
        last_push_t        = rtc_timer_read_ms();
    }
}

void battery_push_force(void) {
    /* Invalidate the cache; next battery_push_check() re-emits. */
    last_pushed_pct    = 0xFF;
    last_pushed_charge = 0xFF;
    pending_active     = false;
}

void battery_push_check(void) {
    uint32_t now    = rtc_timer_read_ms();
    uint8_t  pct    = battery_get_percentage();
    uint8_t  charge = battery_get_charge_state();
    bool     changed = (pct != last_pushed_pct) || (charge != last_pushed_charge);

    if (!changed) {
        /* Same state. Only reason to still emit: pending trailing-edge
         * push (state changed during the debounce window and we
         * deferred sending the final value). */
        if (pending_active && (now - pending_t) >= BATTERY_PUSH_DEBOUNCE_MS) {
            battery_do_push(pending_pct, pending_charge);
            pending_active = false;
        }
        return;
    }

    /* State changed. */
    if ((now - last_push_t) < BATTERY_PUSH_DEBOUNCE_MS) {
        /* Within debounce — defer. Overwrite any previous pending value
         * with the latest (we always want the *most recent* state at the
         * trailing edge, not the first one seen mid-burst). */
        pending_active = true;
        pending_pct    = pct;
        pending_charge = charge;
        pending_t      = now;
    } else {
        /* Past debounce — fire immediately (leading edge). */
        battery_do_push(pct, charge);
        pending_active = false;
    }
}

void battery_check_empty(void) {
    if (voltage < EMPTY_VOLTAGE_VALUE) {
        if (bat_empty <= BATTERY_EMPTY_COUNT) {
            if (++bat_empty > BATTERY_EMPTY_COUNT) {
                indicator_battery_low_enable(true);
                power_on_sample = VOLTAGE_POWER_ON_MEASURE_COUNT;
            }
        }
    }
}

void battery_check_critical_low(void) {
    if (voltage < SHUTDOWN_VOLTAGE_VALUE) {
        if (critical_low <= CRITICAL_LOW_COUNT) {
            if (++critical_low > CRITICAL_LOW_COUNT) wireless_low_battery_shutdown();
        }
    } else if (critical_low <= CRITICAL_LOW_COUNT) {
        critical_low = 0;
    }
}

bool battery_power_on_sample(void) {
    return power_on_sample < VOLTAGE_POWER_ON_MEASURE_COUNT;
}

void battery_timer_reset(void) {
    bat_monitor_timer_buffer = rtc_timer_read_ms();
}

void battery_task(void) {
    uint32_t t = rtc_timer_elapsed_ms(bat_monitor_timer_buffer);
    if ((get_transport() & TRANSPORT_WIRELESS) && (wireless_get_state() == WT_CONNECTED || battery_power_on_sample())) {
#if defined(BAT_CHARGING_PIN)
        if (usb_power_connected() && t > VOLTAGE_MEASURE_INTERVAL) {
            if (gpio_read_pin(BAT_CHARGING_PIN) == BAT_CHARGING_LEVEL)
                lkbt51_update_bat_state(BAT_CHARGING);
            else
                lkbt51_update_bat_state(BAT_FULL_CHARGED);
        }
#endif

        if ((battery_power_on_sample()
#if defined(LED_MATRIX_ENABLE) || defined(RGB_MATRIX_ENABLE)
             && !indicator_is_enabled()
#endif
             && t > BACKLIGHT_OFF_VOLTAGE_MEASURE_INTERVAL) ||
            t > VOLTAGE_MEASURE_INTERVAL) {

            battery_check_empty();
            battery_check_critical_low();

            bat_monitor_timer_buffer = rtc_timer_read_ms();
            if (bat_monitor_timer_buffer > RTC_MAX_TIME) {
                bat_monitor_timer_buffer = 0;
                rtc_timer_clear();
            }

            battery_measure();
            if (power_on_sample < VOLTAGE_POWER_ON_MEASURE_COUNT) power_on_sample++;
        }
    }

    if ((bat_empty || critical_low) && usb_power_connected()) {
        bat_empty    = false;
        critical_low = false;
        indicator_battery_low_enable(false);
    }

    /* Push check runs every battery_task tick. Internal debounce /
     * heartbeat timers gate whether anything actually goes on the
     * wire — outside of state changes this is a few comparisons and
     * a timer read, cheap enough to call unconditionally. */
    battery_push_check();
}
