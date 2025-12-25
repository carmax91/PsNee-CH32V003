#include "ch32fun.h"
#include <stdio.h>

#define SYSCLK_FREQ_48MHZ_HSI 48000000
#define millis() (systick_millis)
#define micros() (SysTick->CNT / DELAY_US_TIME)
// PsNee / psxdev.net version
// Porting to Arduino CH32V003 by Carmax91 rev1 heavly based on kalymos PsNee_v7/v8 (https://github.com/kalymos/PsNee)
//
// There are some pictures in the development thread ( http://www.psxdev.net/forum/viewtopic.php?f=47&t=1262&start=120 )
// Beware to use the PSX 3.5V / 3.3V power, *NOT* 5V! The installation pictures include an example.
//
// Only for WCH CH32V003 mcu @ 48Mhz internal clock.
//
// Coded using Ch32fun library! https://github.com/cnlohr/ch32fun
//
// PAL PM-41 consoles BIOS patching is NOT supported.

// PINOUT for wch ch32v003j4m6:
/*
                   ___ ___
       LED (PD6) -|   U   |- PD1 (SWIO)
             GND -|       |- SUBQ (PC4)
 WFCK_GATE (PA2) -|       |- SQCK (PC2)
             VCC -|       |- DATA (PC1)
                   -------

*/
// DON'T forget to set the region of modchip via #define X_BIT ('e' for PAL, 'a' for USA, 'i' for JAP region) predirectives.

/* Global define */
//------REGION SELECT-------
 #define X_Bit 'e' //PAL
// #define X_Bit 'a' //USA
// #define X_Bit 'i' // JAP
//----------------------------
#define bits_delay 4000     // 250 bits/s (microseconds)
#define injections_delay 90 // 72 in oldcrow. PU-22+ work best with 80 to 100 (milliseconds)
#define HYSTERESIS_MAX 17
#define NOP __asm__ __volatile__("nop\n\t")
/* Global Variable */
// Setup() detects which (of 2) injection methods this PSX board requires, then stores it in wfck_mode.
uint8_t wfck_mode = 0;

volatile uint32_t systick_millis;

/*
 * Initialises the SysTick to trigger an IRQ with auto-reload, using HCLK/1 as
 * its clock source
 */
void systick_init(void)
{
  // Reset any pre-existing configuration
  SysTick->CTLR = 0x0000;

  // Set the compare register to trigger once per millisecond
  SysTick->CMP = DELAY_MS_TIME - 1;

  // Reset the Count Register, and the global millis counter to 0
  SysTick->CNT = 0x00000000;
  systick_millis = 0x00000000;

  // Set the SysTick Configuration
  // NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
  // busywait delay funtions used by ch32v003_fun.

  SysTick->CTLR |= SYSTICK_CTLR_STE |  // Enable Counter
                   SYSTICK_CTLR_STIE | // Enable Interrupts
                   SYSTICK_CTLR_STCLK; // Set Clock Source to HCLK/1

  // Enable the SysTick IRQ
  NVIC_EnableIRQ(SysTick_IRQn);
}

// *****************************************************************************************
// Function: readBit
// Description:
// Reads a specific bit from an array of bytes.
// This function helps retrieve SCEX data efficiently while working within
// the constraints of Harvard architecture.
//
// Parameters:
// - index: The bit position to read within the byte array.
// - ByteSet: A pointer to the byte array containing the data.
//
// Return:
// - Returns 1 if the specified bit at the given index is set (1).
// - Returns 0 if the specified bit is cleared (0).
//
// Explanation:
// - The function determines which byte contains the requested bit using (index / 8).
// - It then calculates the bit position within that byte using (index % 8).
// - A bitwise AND operation extracts the bit's value, and the double NOT (!!) operator
//   ensures a clean boolean return value (1 or 0).
//
// *****************************************************************************************

uint8_t readBit(uint8_t index, const uint8_t *ByteSet)
{
  return !!(ByteSet[index / 8] & (1 << (index % 8))); // Return true if the specified bit is set in ByteSet[index]
}

// *****************************************************************************************
// Function: inject_SCEX
// Description:
// Injects SCEX data corresponding to a given region ('e' for Europe, 'a' for America,
// 'i' for Japan). This function is used for modulating the SCEX signal to bypass
// region-locking mechanisms.
//
// Parameters:
// - region: A character ('e', 'a', or 'i') representing the target region.
//
// *****************************************************************************************

void inject_SCEX(const char region)
{
  // SCEX data patterns for different regions (SCEE, SCEA, SCEI)
  static const uint8_t SCEEData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11101010,
      0b00000010};

  static const uint8_t SCEAData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11111010,
      0b00000010};

  static const uint8_t SCEIData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11011010,
      0b00000010};

  // Iterate through 44 bits of SCEX data
  for (uint8_t bit_counter = 0; bit_counter < 44; bit_counter++)
  {
    // Check if the current bit is 0
    if (readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData
                                                                      : SCEIData) == 0)
    {
      // PC1 -> Data Output ---------------------------------------
      GPIOC->CFGLR &= ~(0xf << (4 * 1));
      GPIOC->CFGLR |= (3 | 0) << (4 * 1);
      //----------------------------------------------------------
      GPIOC->BSHR = (1 << 16 + 1); // PC1 Data Low
      Delay_Us(bits_delay);        // Wait for specified delay between bits
    }
    else
    {
      // modulate DATA pin based on WFCK_READ
      if (wfck_mode) // WFCK mode (pu22mode enabled): synchronize PIN_DATA with WFCK clock signal
      {
        // PC1 -> Data Output ---------------------------------------
        GPIOC->CFGLR &= ~(0xf << (4 * 1));
        GPIOC->CFGLR |= (3 | 0) << (4 * 1);
        //----------------------------------------------------------
        unsigned long now = micros(); // Start microsec timer
        do
        {
          // Read the WFCK pin and set or clear DATA accordingly
          uint8_t wfckSample = (GPIOA->INDR >> (2 & 0xf) & 1); // funDigitalRead(PA2);
          if (wfckSample == 1)
          {
            GPIOC->BSHR = (1 << 1); // //PC1 -> Data High
          }

          else
          {
            GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
          }
        }

        while ((micros() - now) < bits_delay);
      }

      // PU-18 or lower mode: simply set PIN_DATA as input with a delay
      else
      {

        // PC1 <- Data Input -----------------------------------------------
       //funPinMode(PC1, GPIO_CNF_IN_FLOATING);
       GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf<<(4*(1&0xf))))) | (4<<(4*(1&0xf)));
        //-----------------------------------------------------------------
        Delay_Us(bits_delay);
      }
    }
  }

  // After injecting SCEX data, set DATA pin as output and clear (low)
  // PC1 -> Data Output ---------------------------------------
  GPIOC->CFGLR &= ~(0xf << (4 * 1));
  GPIOC->CFGLR |= (3 | 0) << (4 * 1);
  //----------------------------------------------------------
  GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
  Delay_Ms(injections_delay);
}

int main()
{
  SystemInit();
  systick_init();

  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

  // PC1 <- Data Input -----------------------------------------------
  //funPinMode(PC1, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf<<(4*(1&0xf))))) | (4<<(4*(1&0xf)));
  //-----------------------------------------------------------------
  // PA2 <- Wfck Input -----------------------------------------------
  //funPinMode(PA2, GPIO_CNF_IN_FLOATING);
  GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf<<(4*(2&0xf))))) | (4<<(4*(2&0xf)));
  //-----------------------------------------------------------------
  // PC4 <- Subq Input-----------------------------------------------
  //funPinMode(PC4, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf<<(4*(4&0xf))))) | (4<<(4*(4&0xf)));
  //-----------------------------------------------------------------
  // PC2 <- Sqck Input-------------------------------------------------
  //funPinMode(PC2, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf<<(4*(2&0xf))))) | (4<<(4*(2&0xf)));
  //------------------------------------------------------------------
  // PD6 -> LED Output ------------------------------------------------
  GPIOD->CFGLR &= ~(0xF << (4 * 6));
  GPIOD->CFGLR |= (3 | 0) << (4 * 6);
  //------------------------------------------------------------------
  uint8_t hysteresis = 0;
  uint8_t scbuf[12] = {0}; // SUBQ bit storage
  uint16_t timeout_clock_counter = 0;
  uint8_t bitbuf = 0;
  uint8_t bitpos = 0;
  uint8_t scpos = 0; // scbuf position
  uint16_t lows = 0;

  GPIOD->BSHR = (1 << 6); // PD6 -> LED High  //Setup begin!:

  while (!(GPIOC->INDR >> (2 & 0xf) & 1))
    ; // Sqck read==0
  while (!(GPIOA->INDR >> (2 & 0xf) & 1))
    ; // Wfck read==0

  unsigned long now = millis(); // Timer_Start
  //************************************************************************
  // Board detection
  //
  // WFCK: __-----------------------  // this is a PU-7 .. PU-20 board!
  //
  // WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!
  // typical readouts PU-22: highs: 2449 lows: 2377
  // The detection performed here allows us to determine the code's behavior.
  //
  // In the case of older generations like PU-7 through PU-20, we are connected to the output of an operational amplifier used as a buffer
  //(this part of the motherboard was detailed in the official schematics; by convention, this point was labeled GATE).
  // In normal operation, this line is always high, but pulling it down frees the DATA line, allowing the SCEX code to be injected without any further issues.
  //
  // If we are dealing with PU-22 or newer models, it is connected to a WFCK clock output. This clock signal will be used to synchronize the SCEX injection.
  //************************************************************************

  do
  {
    if ((GPIOA->INDR >> (2 & 0xf) & 1) == 0) // Wfck read == 0)
      lows++;                                // good for ~5000 reads in 1s
    Delay_Us(200);
  } while ((millis() - now) < 1000); // sample 1s
  systick_millis = 0; // Timer_Stop and reset variable

  if (lows > 100)
  {
    wfck_mode = 1; // flag pu22mode
  }
  else
  {
    wfck_mode = 0; // flag oldmod
  }
  GPIOD->BSHR = (1 << 16 + 6); // PD6 -> LED Low  //Setup done!

  while (1)
  {
    Delay_Ms(1); /* Start with a small delay, which can be necessary in cases where the MCU loops too quickly and picks up the laster SUBQ trailing end*/

    __disable_irq(); // start critical section

    do
    {
      for (bitpos = 0; bitpos < 8; bitpos++)
      {
        while ((GPIOC->INDR >> (2 & 0xf) & 1) != 0) // Sqck read == 1  // wait for clock to go low
        {
          timeout_clock_counter++;
          // a timeout resets the 12 byte stream in case the PSX sends malformatted clock pulses, as happens on bootup
          if (timeout_clock_counter > 1000)
          {
            scpos = 0;
            timeout_clock_counter = 0;
            bitbuf = 0;
            bitpos = 0;
            continue;
          }
        }

        // Wait for clock to go high
        while ((GPIOC->INDR >> (2 & 0xf) & 1) == 0)
          ; // Sqck read == 0

        if ((GPIOC->INDR >> (4 & 0xf) & 1) == 1) // Subq == 1 // If clock pin high
        {
          bitbuf |= 1 << bitpos; // Set the bit at position bitpos in the bitbuf to 1. Using OR combined with a bit shift
        }

        timeout_clock_counter = 0; // no problem with this bit
      }

      scbuf[scpos] = bitbuf; // One byte done
      scpos++;
      bitbuf = 0;
    }

    while (scpos < 12); // Repeat for all 12 bytes

    __enable_irq(); // End critical section

    //************************************************************************
    // Check if read head is in wobble area
    // We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
    // We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
    // All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
    // Hysteresis is used because older drives exhibit more variation in read head positioning.
    // While the laser lens moves to correct for the error, they can pick up a few TOC sectors.
    //************************************************************************

    // This variable initialization macro is to replace (0x41) with a filter that will check that only the three most significant bits are correct. 0x001xxxxx
    uint8_t isDataSector = (((scbuf[0] & 0x40) == 0x40) && (((scbuf[0] & 0x10) == 0) && ((scbuf[0] & 0x80) == 0)));

    if (
        (isDataSector && scbuf[1] == 0x00 && scbuf[6] == 0x00) &&      // [0] = 41 means psx game disk. the other 2 checks are garbage protection
        (scbuf[2] == 0xA0 || scbuf[2] == 0xA1 || scbuf[2] == 0xA2 ||   // if [2] = A0, A1, A2 ..
         (scbuf[2] == 0x01 && (scbuf[3] >= 0x98 || scbuf[3] <= 0x02))) // .. or = 01 but then [3] is either > 98 or < 02
    )
    {
      hysteresis++;
    }

    // This CD has the wobble into CD-DA space. (started at 0x41, then went into 0x01)
    else if (hysteresis > 0 && ((scbuf[0] == 0x01 || isDataSector) && (scbuf[1] == 0x00 /*|| scbuf[1] == 0x01*/) && scbuf[6] == 0x00))
    {
      hysteresis++;
    }

    // None of the above. Initial detection was noise. Decrease the counter.
    else if (hysteresis > 0)
    {
      hysteresis--;
    }

    // hysteresis value "optimized" using very worn but working drive on ATmega328 @ 16Mhz
    // should be fine on other MCUs and speeds, as the PSX dictates SUBQ rate
    if (hysteresis >= HYSTERESIS_MAX)
    {
      // If the read head is still here after injection, resending should be quick.
      // Hysteresis naturally goes to 0 otherwise (the read head moved).
      hysteresis = 11;

      //************************************************************************
      // Executes the region code patch injection sequence.
      //************************************************************************
      GPIOD->BSHR = (1 << 6); // PD6 -> LED High //Start Injection

      // PC1 -> Data Output ---------------------------------------
      GPIOC->CFGLR &= ~(0xf << (4 * 1));
      GPIOC->CFGLR |= (3 | 0) << (4 * 1);
      //----------------------------------------------------------
      GPIOC->BSHR = (1 << 16 + 1); // PC1 Data Low

      if (!wfck_mode) // If wfck_mode is fals (oldmode)
      {
        // PA2 -> Wfck Output -----------------------------------------------
        GPIOA->CFGLR &= ~(0xf << (4 * 2));
        GPIOA->CFGLR |= (3 | 0) << (4 * 2);
        //-----------------------------------------------------------------
        GPIOA->BSHR = (1 << 16 + 2); // PA2 Wfck Low
      }
      Delay_Ms(injections_delay); // HC-05 waits for a bit of silence (pin low) before it begins decoding.

      // inject symbols now. 2 x 3 runs seems optimal to cover all boards
      for (uint8_t scex = 0; scex < 2; scex++)
      {
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
      }

      if (!wfck_mode)
      {
        // PA2 <- Wfck Input ---------------------------------------------------
        //funPinMode(PA2, GPIO_CNF_IN_FLOATING);
        GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf<<(4*(2&0xf))))) | (4<<(4*(2&0xf)));
        //----------------------------------------------------------------------
      }
      // PC1 <- Data Input ----------------------------------------------------
      //funPinMode(PC1, GPIO_CNF_IN_FLOATING);
      GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf<<(4*(1&0xf))))) | (4<<(4*(1&0xf)));
      //-----------------------------------------------------------------------

      GPIOD->BSHR = (1 << 16 + 6); // PD6 -> LED Low //Injection done!
    }
  }
}

/*
 * SysTick ISR - must be lightweight to prevent the CPU from bogging down.
 * Increments Compare Register and systick_millis when triggered (every 1ms)
 * NOTE: the `__attribute__((interrupt))` attribute is very important
 */

void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
  // Increment the Compare Register for the next trigger
  // If more than this number of ticks elapse before the trigger is reset,
  // you may miss your next interrupt trigger
  // (Make sure the IQR is lightweight and CMP value is reasonable)
  SysTick->CMP += DELAY_MS_TIME;

  // Clear the trigger state for the next IRQ
  SysTick->SR = 0x00000000;

  // Increment the milliseconds count
  systick_millis++;
}