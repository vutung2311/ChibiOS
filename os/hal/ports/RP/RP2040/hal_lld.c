/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    RP2040/hal_lld.c
 * @brief   RP2040 HAL subsystem low level driver source.
 * @note    The Core 1 launch sequence follows the multicore launch protocol
 *          documented in RP2040 Datasheet 2.8.2 "Launching Code On
 *          Processor Core 1".
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"
#include "rp_clocks.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_rp2040.h.
 */
uint32_t SystemCoreClock = RP_CLK_SYS_FREQ;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if RP_CORE1_START == TRUE
static void start_core1(void) {
  extern uint32_t RP_CORE1_STACK_END;
  extern uint32_t RP_CORE1_VECTORS_TABLE;
  extern void RP_CORE1_ENTRY_POINT(void);
  static const uint32_t cmd_sequence[] = {0, 0, 1,
                             (uint32_t)&RP_CORE1_VECTORS_TABLE,
                             (uint32_t)&RP_CORE1_STACK_END,
                             (uint32_t)RP_CORE1_ENTRY_POINT};
  unsigned seq;

  /* Force-reset core1 via PSM before starting the FIFO launch handshake.
   *
   * If core1 survived a preceding soft reset (e.g. a bootloader that uses
   * BX to jump to the application rather than SYSRESETREQ), it may still
   * be executing application code from a previous run.  In that case the
   * ROM FIFO handshake protocol will deadlock: core0 sends the sync
   * sequence but core1 is not in the ROM wait-for-launch loop so it never
   * echoes back.
   *
   * PSM FRCE_OFF holds core1 in reset (powered-off state).  Clearing it
   * releases core1 into the ROM boot-path which immediately enters the
   * wait-for-launch loop.  The PSM DONE bit falls while the core is held
   * off, and rises again once it has been released and is running.*/

  /* QMK/Vial Modification:
     We cherry-pick the safe Core 1 reset sequence from the Raspberry Pi Pico SDK.
     Instead of busy-waiting on the hardware-status PSM->DONE bit (which deadlocks
     on warm reboots/flashing on this keyboard controller), we assert the Core 1
     reset bit, read once to let the bus write propagate, and immediately clear it.
     This safely forces Core 1 back to its ROM wait-for-launch loop on warm resets
     without any risk of infinite deadlocks. */
  PSM->SET.FRCE_OFF = PSM_ANY_PROC1;         /* force Core 1 into reset */
  (void)PSM->FRCE_OFF;                       /* read once to settle write */
  PSM->CLR.FRCE_OFF = PSM_ANY_PROC1;         /* release Core 1 to enter ROM */

  /* Starting core 1.*/
  seq = 0;
  do {
    uint32_t response;
    uint32_t cmd = cmd_sequence[seq];

    /* Flushing the FIFO state before sending a zero.*/
    if (!cmd) {
      fifoFlushRead();
    }
    fifoBlockingWrite(cmd);
    response = fifoBlockingRead();
    /* Checking response, going forward or back to first step.*/
    seq = cmd == response ? seq + 1U : 0U;
  } while (seq < 6U);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

#if RP_NO_INIT == FALSE
  /* QMK/Vial Modification:
     We restore the safe clock initialization block here.
     Since we commented out the dangerous early peripheral resets in board.c
     to avoid XIP flash/bus deadlocks, we initialize system clocks (PLLs/XOSC)
     safely here before unresetting the bus fabric. */
  rp_clock_init();

  rp_peripheral_unreset(RESETS_ALLREG_BUSCTRL);
  rp_peripheral_unreset(RESETS_ALLREG_SYSINFO);
  rp_peripheral_unreset(RESETS_ALLREG_SYSCFG);
#endif /* RP_NO_INIT */

  /* NVIC initialization.*/
  nvicInit();

  /* IRQ subsystem initialization.*/
  irqInit();
#if defined(RP_DMA_REQUIRED)
  dmaInit();
#endif
#if defined(RP_PIO_REQUIRED)
  pioInit();
#endif

  /* Bind the system timer IRQ to this core for tickless mode.*/
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  stBind();
#endif

#if RP_CORE1_START == TRUE
  start_core1();
#endif
}

/** @} */
