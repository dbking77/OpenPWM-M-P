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


// TODO have template class for this?
#define OUT1_PORT PORTB
#define OUT1_DDR DDRB
#define OUT1_PIN 2

#define OUT2_PORT PORTA
#define OUT2_DDR DDRA
#define OUT2_PIN 7

#define OUT3_PORT PORTA
#define OUT3_DDR DDRA
#define OUT3_PIN 5

#define OUT4_PORT PORTA
#define OUT4_DDR DDRA
#define OUT4_PIN 6

#define LED1_PORT PORTA
#define LED1_DDR DDRA
#define LED1_PIN 4

#define TOUT_PORT PORTA
#define TOUT_DDR DDRA
#define TOUT_PIN 0

#define PWM1_PORT PORTA
#define PWM1_DDR DDRA
#define PWM1_PIN 1

#define PWM2_PORT PORTA
#define PWM2_DDR DDRA
#define PWM2_PIN 2

#define LED2_PORT PORTA
#define LED2_DDR DDRA
#define LED2_PIN 3

#define LED1_PORT PORTA
#define LED1_DDR DDRA
#define LED1_PIN 4


void gpioInit()
{
  DDRA = 
    (0<<TOUT_PIN) |
    (0<<PWM1_PIN) |
    (0<<PWM2_PIN) |
    (1<<LED2_PIN) | 
    (1<<LED1_PIN) |
    (1<<OUT3_PIN) |
    (1<<OUT4_PIN) |
    (1<<OUT2_PIN) ;
  
  PORTA =
    (1<<PWM1_PIN) |
    (1<<PWM2_PIN) |
    (1<<LED2_PIN) |
    (1<<LED1_PIN) ;
  
  DDRB = 
    (1<<OUT1_PIN);
  
  PORTB = 0;
}

/*
 * Mapping of Timers to Outputs
 *  output1 and output2 are a pair
 *  output2 and output4 are a pair
 *  
 *  output1 : PB2 : OC0A
 *  output2 : PA7 : OC0B
 *  output3 : PA5 : OC1B
 *  output4 : PA6 : OC1A
 */


/**
 * Set duty for motor1 (out1 and out2)
 *    value of 255 = full forward
 *    value of -255 = full reverse
 *    value of 0 = stop/brake
 * 
 * out1 = PB2 = OC0A
 * out2 = PA7 = OC0B
 */
void setMotor1(int16_t duty)
{
  // Enable outputs, by enable PWM control of pins
  // set A & B outputs to non-inverting (for fast PWM)
  // in non-inverting mode pins are SET at BOTTOM
  enum { T0_COMv = 2 }; 
  TCCR0A |= (T0_COMv<<COM0A0) | (T0_COMv<<COM0B0);

  // Staturate duty between -0xFF and 0xFF
  if (duty > 0xFF)
  {
    duty = 0xFF;
  }
  else if (duty < -0xFF)
  {
    duty = -0xFF;
  }

  if (duty < 0)
  {
    OCR0A = duty-1;
    OCR0B = 0xFF; // 0xFF high always
  }
  else 
  {    
    OCR0A = 0xFF; // 0xFF high always
    OCR0B = -duty-1;
  }
}

/**
 * Set duty for motor2 (out3 and out4)
 *    value of 255 = full forward
 *    value of -255 = full reverse
 *    value of 0 = stop/brake
 * 
 * out3 = PA5 = OC1B
 * out4 = PA6 = OC1A
 */
void setMotor2(int16_t duty)
{
  // Enable outputs, by enable PWM control of pins
  // set A & B outputs to non-inverting (for fast PWM)
  // in non-inverting mode pins are set at BOTTOM, and cleared on compare match
  enum { T1_COMv = 2 }; 

  TCCR1A |= (T1_COMv<<COM0A0) | (T1_COMv<<COM0B0);

  // Staturate duty between -0xFF and 0xFF
  if (duty > 0xFF)
  {
    duty = 0xFF;
  }
  else if (duty < -0xFF)
  {
    duty = -0xFF;
  }

  if (duty < 0)
  {
    OCR1A = duty-1;
    OCR1B = 0xFF; // 0xFF high always
  }
  else 
  {    
    OCR1A = 0xFF; // 0xFF high always
    OCR1B = -duty-1;
  }
}


void disableMotors()
{
  // Disable outputs be switching pins to normal operation.
  // Out1 and out2 will PORT are already set to 0, so output will
  // high-Z and motor will coast
  TCCR0A &= 0x0F;  // Clears COM0A and COM0B (top 4 bits)
  TCCR1A &= 0x0F;
}


int main(void)
{
  gpioInit();

  // Enable PWM mode on timer 0 without actually enabling outputs
  enum { T0_WGMv = 3 }; //fast PWM mode use 0xFF as TOP
  enum { T0_CSv = 1 };  //use internal clock with no prescalling
  // Enable Timer/Counter0 outputs 
  TCCR0A = (T0_WGMv&3);
  TCCR0B = ((T0_WGMv>>2)<<WGM02) | T0_CSv;

  // Enable PWM mode on timer 1.  Have timer saturate at 0xFF
  enum { T1_WGMv = 5 }; //fast PWM mode use 0xFF as TOP
  enum { T1_CSv = 1 };  //use internal clock with no prescalling
  TCCR1A = (T1_WGMv&3);
  TCCR1B = ((T1_WGMv>>2)<<WGM12) | T1_CSv;

  // Have timers run 180degrees out-of-phase to reduce supply ripple
  TCNT0 = 0;
  TCNT1 = 0x84;

  // Enable interrupts
  //sei();

  int16_t duty = 0;
  int8_t direction = -1;
  while (1)
  {
    setMotor1(duty);
    setMotor2(duty);
    duty+=direction;
    if ((direction > 0) && (duty > 300))
    {
      duty = 0;
      //direction = -1;
    }
    else if ((direction < 0) && (duty <= -300))
    {
      duty = 0;
      //direction = 1;
    }
    _delay_ms(20);
  }

  return 0;
}
