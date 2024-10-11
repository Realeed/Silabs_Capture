/***************************************************************************//**
 * @file main.c
 *
 * @brief This project demonstrates high frequency single pulse capture using
 * the TIMER module.  A periodic input signal is routed to a capture/compare
 * channel, and a single pulse width is captured and stored.  Connect a periodic
 * signal to the GPIO pin specified in the readme.txt for input.  The minimum
 * duration measurable via this method of input capture is around 700 ns.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "sl_board_control.h"
#include "sl_board_control_config.h"
#include "sl_iostream_init_usart_instances.h"
#include <stdio.h>
#include "sl_udelay.h"

// Stored edges from interrupt
uint32_t capture = 0;

bool captureSet = false;
bool freqCapture = false;

/**************************************************************************//**
 * @brief
 *    HFXO initialization
 *****************************************************************************/
void initHFXO(void)
{
    // Initialize the HFXO with the board specific settings
    CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;
    CMU_HFXOInit(&hfxoInit);

    // Enable HFXO and wait for it to stabilize
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
}

/**************************************************************************//**
 * @brief
 *    CMU initialization
 *****************************************************************************/
void initCMU(void)
{
    // Select the HFXO as the EM01GRPACLK source (clock for all TIMERs)
    CMU_ClockSelectSet(cmuClock_SYSCLK, cmuSelect_HFXO);
    CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFXO);

    /*
     * Enable the GPIO and TIMER0 bus clocks.
     *
     * Note: On EFR32xG21 devices, calls to CMU_ClockEnable() have no
     * effect as clocks are automatically turned on/off in response to
     * on-demand requests from the peripherals.  CMU_ClockEnable() is
     * a dummy function on EFR32xG21 present for software compatibility.
     */
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_TIMER0, true);
}

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
    // Configure PA6 as input for TIMER0 CC0
    GPIO_PinModeSet(gpioPortD, 2, gpioModeInput, 0);
    GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0);

    GPIO_PinModeSet(gpioPortB, 1, gpioModeInput, 1);

    GPIO_ExtIntConfig(gpioPortB, 1, 1, false, true, 1);

    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/
void initTIMER0(void)
{
    // Initialize the timer
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

    // Configure TIMER0 for input capture mode
    TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

    timerCCInit.mode = timerCCModeCapture;
    timerCCInit.edge = timerEdgeBoth;                 // Input capture on every edge
    timerCCInit.eventCtrl = timerEventEveryEdge;      // Interrupt on every other edge
    timerInit.riseAction = timerInputActionReloadStart;
    timerInit.enable = false;

    TIMER_Init(TIMER0, &timerInit);

    // Route TIMER0 CC0 input from PA6
    GPIO->TIMERROUTE[0].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
            | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

    TIMER_InitCC(TIMER0, 0, &timerCCInit);

    // Enable TIMER0 interrupts
    TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Now enable the TIMER
    //TIMER_Enable(TIMER0, true);
}

/**************************************************************************//**
 * @brief
 *    Interrupt handler for TIMER0
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
    // Acknowledge the interrupt
    TIMER_IntClear(TIMER0, TIMER_IntGet(TIMER0));
    capture = TIMER_CaptureGet(TIMER0, 0);
    captureSet = true;
    if (freqCapture) {
        freqCapture = false;
    } else {
        freqCapture = true;
    }
}


uint32_t capture_getPwmFreq()
{
  uint32_t freq = 0;
  uint8_t cnt = 0;
  while (cnt != 5) {
      if (captureSet) {
          captureSet = false;
          if (freqCapture) {
              freq = 38400000 / capture;
              cnt++;
          }
      }
  }
  return freq;
}

uint32_t capture_getPwmDutyCycle()
{
  uint32_t dutyCycle;
  uint32_t freq = capture_getPwmFreq();
  while (1) {
      if (captureSet) {
          captureSet = false;
          if (!freqCapture) {
              dutyCycle = capture * freq / 3840;
              if (dutyCycle % 10 >= 5) {
                  return dutyCycle / 10 + 1;
              }
              else {
                  return dutyCycle / 10;
              }
          }
      }
  }
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
    CHIP_Init();

    initHFXO();
    initCMU();
    initGPIO();
    initTIMER0();
    while (1) {
        if (capture_getPwmFreq() != 1000 || capture_getPwmDutyCycle() != 100) {
            GPIO_PinOutSet(gpioPortB, 0);
        }
    }
}
