/* Low-power-mode breadcrumbs in the RTC backup registers.
 *
 * The backup domain survives STOP mode and system resets (incl. the
 * HardFault handler's NVIC_SystemReset below), so after a hang that is
 * recovered by a wake or a reset, the host can read where the keyboard
 * was and how it woke up (raw HID KC_GET_LPM_DIAG = 0xAE). It does NOT
 * survive a power cut (VBAT tied to VDD on this board): a recovery by
 * battery disconnect loses the trail.
 *
 * Register map (RTC->BKPnR):
 *   10  magic 0x4C50 << 16 | stage at last boot << 8 | current stage
 *   11  STOP attempts (lpm_standby entered)
 *   12  WFI returns (real wake-ups)
 *   13  skipped sleeps (wake condition already present before WFI)
 *   14  last 4 wake sources, newest in the low byte: EXTI line number of
 *       the lowest pending line, 0xFE = skipped, 0xFF = no line pending
 *   15  boot count << 16 | HardFault count
 *   16  last HardFault stacked PC
 *   17  RCC_CSR reset flags at last boot << 8 | stage at last HardFault
 *   18  EXTI pending mask at the last wake
 *   19  last HardFault stacked LR
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    LPM_STAGE_NONE    = 0,
    LPM_STAGE_RUN     = 1, /* awake, main loop */
    LPM_STAGE_ENTER   = 2, /* tearing down peripherals for STOP */
    LPM_STAGE_WFI     = 3, /* inside lpm_standby, about to WFI */
    LPM_STAGE_WOKE    = 4, /* WFI returned / skipped */
    LPM_STAGE_HALINIT = 5, /* halInit() + timer_restore() done */
    LPM_STAGE_MATRIX  = 6, /* lpm_wakeup() finished re-init */
    LPM_STAGE_BOOT    = 7, /* lpm_init() at power-up */
};

void    lpm_diag_boot(void);
void    lpm_diag_stage(uint8_t stage);
void    lpm_diag_standby_enter(void);
void    lpm_diag_standby_exit(bool skipped, uint32_t exti_pending);
uint8_t lpm_diag_read(uint8_t page, uint8_t *out, uint8_t max_len);
