/* Low-power-mode breadcrumbs + HardFault auto-recovery. See lpm_diag.h. */

#include "hal.h"
#include "lpm_diag.h"

#if defined(QMK_MCU_SERIES_STM32F4XX) && (HAL_USE_RTC == TRUE)

#    define DIAG_MAGIC 0x4C50U /* "LP" */
#    define BKP(n) ((&RTC->BKP0R)[(n)])

enum {
    R_STAGE  = 10,
    R_STOPS  = 11,
    R_WAKES  = 12,
    R_SKIPS  = 13,
    R_SRCS   = 14,
    R_COUNTS = 15,
    R_HF_PC  = 16,
    R_FLAGS  = 17,
    R_PR     = 18,
    R_HF_LR  = 19,
};

static inline uint8_t cur_stage(void) {
    return BKP(R_STAGE) & 0xFFU;
}

void lpm_diag_stage(uint8_t stage) {
    BKP(R_STAGE) = (BKP(R_STAGE) & 0xFFFFFF00U) | stage;
}

void lpm_diag_boot(void) {
    /* ChibiOS leaves backup-domain write access open after init; be
     * explicit anyway, it costs one store. */
    PWR->CR |= PWR_CR_DBP;

    uint32_t reg = BKP(R_STAGE);
    if ((reg >> 16) != DIAG_MAGIC) {
        for (uint8_t n = R_STAGE; n <= R_HF_LR; n++)
            BKP(n) = 0;
        reg = DIAG_MAGIC << 16;
    }
    uint8_t stage_at_boot = reg & 0xFFU;

    /* RCC_CSR: bits 25..31 = BOR/PIN/POR/SFT/IWDG/WWDG/LPWR reset flags. */
    uint8_t flags = (RCC->CSR >> 24) & 0xFEU;
    RCC->CSR |= RCC_CSR_RMVF;

    BKP(R_STAGE)  = (DIAG_MAGIC << 16) | ((uint32_t)stage_at_boot << 8) | LPM_STAGE_BOOT;
    BKP(R_COUNTS) = BKP(R_COUNTS) + (1U << 16);
    BKP(R_FLAGS)  = (BKP(R_FLAGS) & 0xFFU) | ((uint32_t)flags << 8);
}

void lpm_diag_standby_enter(void) {
    BKP(R_STOPS) = BKP(R_STOPS) + 1;
    lpm_diag_stage(LPM_STAGE_WFI);
}

void lpm_diag_standby_exit(bool skipped, uint32_t exti_pending) {
    uint8_t src;
    if (skipped) {
        BKP(R_SKIPS) = BKP(R_SKIPS) + 1;
        src          = 0xFE;
    } else {
        BKP(R_WAKES) = BKP(R_WAKES) + 1;
        src          = exti_pending ? (uint8_t)__builtin_ctz(exti_pending) : 0xFF;
    }
    BKP(R_SRCS) = (BKP(R_SRCS) << 8) | src;
    BKP(R_PR)   = exti_pending;
    lpm_diag_stage(LPM_STAGE_WOKE);
}

uint8_t lpm_diag_read(uint8_t page, uint8_t *out, uint8_t max_len) {
    uint8_t first = page == 0 ? R_STAGE : R_FLAGS;
    uint8_t last  = page == 0 ? R_HF_PC : R_HF_LR;
    uint8_t len   = 0;
    for (uint8_t n = first; n <= last && len + 4 <= max_len; n++) {
        uint32_t v = BKP(n);
        out[len++] = v & 0xFF;
        out[len++] = (v >> 8) & 0xFF;
        out[len++] = (v >> 16) & 0xFF;
        out[len++] = (v >> 24) & 0xFF;
    }
    return len;
}

/* HardFault (MemManage/BusFault/UsageFault escalate here, they are not
 * enabled separately): record where it happened, then reset instead of
 * spinning in ChibiOS' _unhandled_exception forever. A dead keyboard
 * that needs the mode switch or the battery cut to come back becomes a
 * one-second reboot, and the trail survives for the host to read. */
void lpm_diag_hardfault(uint32_t *frame) {
    BKP(R_HF_PC)  = frame[6];
    BKP(R_HF_LR)  = frame[5];
    BKP(R_COUNTS) = (BKP(R_COUNTS) & 0xFFFF0000U) | ((BKP(R_COUNTS) + 1) & 0xFFFFU);
    BKP(R_FLAGS)  = (BKP(R_FLAGS) & 0xFFFFFF00U) | cur_stage();
    NVIC_SystemReset();
}

__attribute__((naked, used)) void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4          \n"
        "ite eq              \n"
        "mrseq r0, msp       \n"
        "mrsne r0, psp       \n"
        "b lpm_diag_hardfault\n");
}

#else

void lpm_diag_boot(void) {}
void lpm_diag_stage(uint8_t stage) {
    (void)stage;
}
void lpm_diag_standby_enter(void) {}
void lpm_diag_standby_exit(bool skipped, uint32_t exti_pending) {
    (void)skipped;
    (void)exti_pending;
}
uint8_t lpm_diag_read(uint8_t page, uint8_t *out, uint8_t max_len) {
    (void)page;
    (void)out;
    (void)max_len;
    return 0;
}

#endif
