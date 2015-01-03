/*
 * Copyright (c) 2015, Derek King
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/* 
 * Getting compiler and tools (in Ubuntu)
 *  sudo apt-get update
 *  sudo apt-get install binutils-avr gcc-avr avrdude avr-libc
 */


/*
 * Fuses : want to run at 8Mhz off of internal oscillator
 * 
 * Extended Fuse Byte : 0xFF
 * unused [7:1]   
 * SELFPRGEN 0    : 1   : Self-prgromming enabled   : no
 *
 * High Fuse Byte : 11011111b = 0xDF
 * RSTDISBLE 7    : 1   : External reset disabled   : no
 * DWEN      6    : 1   : DebugWIRE active          : no
 * SPIEN     5    : 0   : Serial Program Enabled    : yes
 * WDTON     4    : 1   : Watchdog timer always on  : no
 * EESAVE    3    : 1   : EEPROM preserver on erase : no EEPROM not presevered
 * BODLEVEL [2:1] : 111 : Brownout Dectector level  : ?
 *
 * Low Fuse Byte : 11100010b = 0xE2
 * CKDIV8   7     : 1   : Clock Divided By 8        : don't divide by 8 (8Mhz internal osc)
 * CKOUT    6     : 1   : Clock output enabled      : no
 * SUT      [5:4] : 10  : Start-up time             : 14Clock cycles + 64ms
 * CLKSEL   [3:0] : 0010: Clock select              : calibrated internal osc
 *
 * http://www.engbedded.com/fusecalc/
 *  -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
 */


/* 
 * Notes
 *   avr-gcc -print-file-name=include 
 *   g/usr/lib/gcc/avr/4.5.3/include
 *   /usr/lib/avr/include/avr/iotn25.h
 */


/* Pin numbering defs */
enum {
  REV_PIN=0,  // reverse output pin
  FWD_PIN=1,  // forward output pin 
  PWM_PIN=2,  // server pwm input pin
  LED_PIN=3,  
  TEMP_PIN=4  // connected to temperature thermistor
};



/**
 * Set duty of motor output
 *    value of 255 = full forward
 *    value of -255 = full reverse
 *    value of 0 = stop/brake
 */
void setMotorOutput(int16_t duty)
{
  // Enable Timer/Counter0 outputs 
  enum { T0_COMv = 2 }; //non-inverting (for fast PWM)
  TCCR0A |= (T0_COMv<<COM0A0) | (T0_COMv<<COM0B0);

  if (duty>0xFF)
  {
    duty = 0xFF;
  }
  else if (duty < -0xFF)
  {
    duty = -0xFF;
  }

  if (duty < 0)
  {
    OCR0A = duty;
    OCR0B = 0xFF; // 0xFF high always
  }
  else 
  {    
    OCR0A = 0xFF; // 0xFF high always
    OCR0B = 0xFF-duty;
  }
}


/**
 * Disables motor output : effectivly high-Z outputs
 *  When both REV and FWD inputs to motor are low the motor driver disables its outputs
 *  Because of how the PWM signals are generated, a always low output cannot be generated,
 *  So 
 */
void disableMotor()
{
  // disable both timer outputs by setting botCOM0A/B to 0
  TCCR0A &= 0x0F;
}

/// Updated by times
volatile uint8_t g_pwm_pulse_width = 0;

/// number of times PWM overflow between getting pulses
uint8_t g_pwm_overflow_count = 0;

/// Used by main loop for flashing LED
volatile char g_overflow_flag = 0;

ISR(TIMER1_OVF_vect)
{
  g_overflow_flag = 1;
  if (g_pwm_overflow_count < 250)
  {
    ++g_pwm_overflow_count;
  }
  else 
  {
    // if the timer has overflowed for more than 200 times 
    // then it has been 0.5seconds since the last good pwm pulse was recieved, 
    // and the motors output should be disabled
    g_pwm_pulse_width = 0;
  }
}

ISR(INT0_vect)
{
  static uint8_t pwm_rising_edge = 0;
  // Flash LED
  if (PINB & (1<<PWM_PIN))
  {    
    // rising edge of PWM
    pwm_rising_edge = TCNT1;
    g_pwm_overflow_count = 0;
  }
  else
  {
    // falling edge of PWM
    if (g_pwm_overflow_count < 2)
    {
      // if overflow count is more than 1 then this was not a valid pulse width
      g_pwm_pulse_width = TCNT1 - pwm_rising_edge;
    }    
  }
}


int main(void)
{
  // Enable PWM mode on timer 0 without actually enabling outputs
  enum { T0_WGMv = 3 }; //fast PWM mode use 0xFF as TOP
  TCCR0A = (T0_WGMv&3);
  enum { T0_CSv = 1 };  //use internal clock with no prescalling
  TCCR0B = ((T0_WGMv>>2)<<WGM02) | T0_CSv;

  // Initialize Timer/Counter1 as free-running 0 to 0xFF
  // Pin change interrupt will read timer value on rising and falling edges 
  // and compare values to get pulse width. 
  // 
  // Have timer prescaled so that its period is longer than the longest
  // valid servo WPM pulse width that needs to be measured (2ms).
  //   2ms * 8Mhz / 256 = 62.5 --> set clock divider to 64
  enum { T1_CSv = 7 };  //use devide clock by 64
  TCCR1 = T1_CSv;  // all other bits are 0
  GTCCR = 0;
  PLLCSR = 0; // don't use external clock for timer1

  // Timer interrupt mask (for timer0 and timer1)
  // Use timer1 overflow interrupt to detect loss of servo PWM input signal
  TIMSK = (1<<TOIE1);  
  
  // PORTB data direction
  // 0 : REV  : output
  // 1 : FWD  : output
  // 2 : PWM  : input w/ pullup
  // 3 : LED  : output
  // 4 : TEMP : input
  DDRB = (1<<REV_PIN) | (1<<FWD_PIN) | (1<<LED_PIN);
  PORTB = (1<<LED_PIN) | (1<<PWM_PIN);

  // Enable int0 interrupt, on both rising and falling edges
  MCUCR = (MCUCR & 0xFC) | 1; 
  GIMSK = (1<<INT0);

  // Enable interrupts
  sei();


  uint8_t flash_count = 0;
  while (1)
  {
    // Count 2ms overflows to determine when to flash LED
    if (g_overflow_flag)
    {
      g_overflow_flag = 0;
      ++flash_count;
    }

    // pwm pulse width is volatile (changed by interrupt) so read it only once before using it
    int16_t pulse_width = g_pwm_pulse_width; 
    if (pulse_width == 0)
    {
      PORTB |= (1<<LED_PIN);
      disableMotor();
    }
    else 
    {
      // Scale and offset PWM pulse with so that 1.9us pulse produces duty of 255, 
      //   and 1.1ms pulse produces duty of -255
      // 1 LSB of pulse width is nominally 8uSec ( 1/8Mhz * 64 = 8usec)
      // zero throttle is about 1.5ms, which is 187.5 (1.5msec/8usec = 187.5)
      int16_t duty = (int16_t)pulse_width - 187;
      // Range on PWM pulse is 1.9-1.1 = 0.8ms = 100.
      // rescale PWM so that pulse of -50 becomes -255 and +50 becomes 255
      //  We want to multiply duty by 5.1 (255/50 = 5.1)
      //  To get same result multiple duty by 5.1*8 and shift left by 3 to divide by 8
      //    50*41>>3 = 256 
      duty = (duty*41) >> 3;
      setMotorOutput(duty);

      // When getting good signal, flash LED slowly
      if (flash_count > 100)
      {
        flash_count = 0;
        PORTB ^= (1<<LED_PIN);
      }
    }
  }

  return 0;
}
