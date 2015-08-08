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
 * High Fuse Byte : 11011111b = 0xDD
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
 *   /usr/lib/avr/include/avr/iotn24a.h
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

#define TOUT_PORT PORTA
#define TOUT_DDR DDRA
#define TOUT_PIN 0

#define PWM1_PORT PORTA
#define PWM1_DDR DDRA
#define PWM1_PIN 1

#define PWM2_PORT PORTA
#define PWM2_DDR DDRA
#define PWM2_PIN 2

// Used to indicate error has occurred
#define LED2_PORT PORTA
#define LED2_DDR DDRA
#define LED2_PIN 3

// Used to indicate that input signal is being recv'ed
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
  TCCR1A |= (T1_COMv<<COM1A0) | (T1_COMv<<COM1B0);

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
    OCR1A = 0xFF&(duty-1);
    OCR1B = 0xFF; // 0xFF high always
  }
  else 
  {    
    OCR1A = 0xFF; // 0xFF high always
    OCR1B = 0xFF&(-duty-1);
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



volatile uint8_t tim0_ovf_count = 0;
volatile char tim0_ovf_flag;
ISR(TIM0_OVF_vect)
{
  tim0_ovf_flag = 1;
  ++tim0_ovf_count;
}



// Prevent compiler from moving memory access operations across this code
#define MEMBAR() __asm volatile( "" ::: "memory" );

/** This struct allows use to create 16bit value from two 8bit values.
 *  It assumes that avrgcc organizes memory in little-endian order
 *
 * AVR uses 8bit registers, so 16bit value is stored two 8bit register
 * The typical code to concatinate two 8bit values into 16bit values is:
 *  
 *  uint8_t lo,hi;
 *  uint16_t result = (hi<<8) | lo;
 * 
 * The above code really shouldn't produce any assembly, because its just
 * shuffling around registers but it does:
 *    166:	83 2f       	mov	r24, r19
 *    168:	90 e0       	ldi	r25, 0x00	; 0
 *    16a:	92 2b       	or	r25, r18
 *    16c:	90 93 6d 00 	sts	0x006D, r25
 *    170:	80 93 6c 00 	sts	0x006C, r24
 *
 * If the union is used instead, the assembly looks like
 *    176:	c3 2f       	mov	r28, r19
 *    178:	d2 2f       	mov	r29, r18
 *    17a:	d0 93 6d 00 	sts	0x006D, r29
 *    17e:	c0 93 6c 00 	sts	0x006C, r2
 */
union Timer16
{
  uint16_t val;
  struct 
  {
    uint8_t lo,hi;
  };
};



/**
 * Returns 16bit timestamp generated from TCNT0
 */
uint16_t getTimestamp()
{
  union Timer16 cnt;
  cnt.lo = TCNT0;
  MEMBAR();
  cnt.hi = tim0_ovf_count;
  MEMBAR();
  uint8_t tifr = TIFR0;
  MEMBAR();

  // A servo PWM pulse is typically between 1.0 and 2.0 ms long.
  // Since clock is 8Mhz, this a 2.0ms pulse is 16000 cycles.
  // Unfortunately TIM0 is only 8bit (and TIM1 is set to only count to 8bit).
  // Just using the TCNT0 value to measure pulse width is not enough.
  // 
  // To generate top 8bits of 16bit time value, the timer overflow interrupt to
  // increment an 8bit overflow counter.  
  // Typically concatinating the overflow counter 8bit TCNT0 value produces 
  // will prodcue the correct overflow count.  
  // However, the is a race condition where the TNCT timer has overflowed
  // but the ISR has not run to increment the counter.
  // In this case the timer overflow flag will be set and TCNT will have a low
  // value.  In this case, the code should add 1 to the counter before using it.
  // There is is also the possibility that the timer overflows just after TCNT
  // is read, in this case the overflow flag will be set, and TCNT will be high,
  // in this case the overflow counter is correct.
  if ((tifr & (1<<TOV0)) && (cnt.lo & 0x80))
  {
    ++cnt.hi;
    // debug toggle LED everytime this occurs
    //LED2_PORT ^= (1<<LED2_PIN);
  }
  return cnt.val;
}


struct ServoPwm
{
  // pulse width measurement value of 8000 = 1ms.
  // value of zero is special and indicates valid value
  volatile uint16_t pulse_width;
  int16_t average_pulse_width;
  uint16_t rising_edge_time;
  volatile uint8_t no_signal_cycles;
  char valid_rising_edge; // true if rising edge was captured recently
} pwm1, pwm2;

// Clock frequency is 8Mhz, centered pulse width is usually 1.5ms
//   1.5ms = 1500us :  1500us * 8Mhz = 1500*8
//   DX5e output varies from 1.1 to 1.9ms
enum 
{
  MIN_VALID_SERVO_PULSE_WIDTH = 800*8, 
  CENTER_SERVO_PULSE_WIDTH    = 1500*8,
  MAX_VALID_SERVO_PULSE_WIDTH = 2200*8,
  SERVO_DEADZONE_WIDTH        = 100*8,
};

// Number of 2ms cycles before servo signal is assumed to be *lost*
enum {LOST_SERVO_SIGNAL_CYCLES=100};

/**
 * Should be called from mainloop about every 2milliseconds, in order to 
 * detect bad pulse-widths or a loss of servo signal
 * 
 * returns true if servo signal seems to be lost
 */
char timeoutServoPwm(struct ServoPwm* pwm, uint16_t time)
{
  // Since clock runs at 8Mhz, every 8.192ms the 16bit timestamp will wrap around.
  // Any pulse longer than 8.192ms will not be measured properly because
  // the difference between rising and falling edges will be large than a 16bit value
  //
  // Instead a long pulse width will alias as a shorter measurement.
  // For example a 9.192ms pulse will be alias to a 1ms pulse.
  // This means invalid pulse widths might seems to be valid.  
  // To prevent longer pulse from aliasing to shorter values, this code will
  // measure the difference between the current time, and last rising edge time,
  // if the time difference is larger than the MAX_VALID_SERVO_PULSE_WIDTH, 
  // it will clear the valid_rising_edge flag
  if (pwm->valid_rising_edge) 
  {
    uint16_t width = time - pwm->rising_edge_time;
    if (width > MAX_VALID_SERVO_PULSE_WIDTH)
    {
      pwm->valid_rising_edge = 0;
    }
  }

  // every time this is called to increment no signal cycles
  cli();
  uint8_t no_signal_cycles = pwm->no_signal_cycles+1;
  char lost_signal = (no_signal_cycles >= LOST_SERVO_SIGNAL_CYCLES);
  if (!lost_signal)
  {
    // only write value back if it has not overran
    pwm->no_signal_cycles = no_signal_cycles;
  }
  sei();
      
  return lost_signal;
}


/** 
 * Called from pin-change interrupt for rising or falling edge of servo pwm
 */
void updateServoPwm(struct ServoPwm* pwm, uint16_t time, char rising)
{
  if (rising)
  {
    // rising edge
    pwm->valid_rising_edge = 1;
    pwm->rising_edge_time = time;    
  }
  else if (pwm->valid_rising_edge)
  {
    pwm->valid_rising_edge = 0;
    // falling edge with valid rising edge, determine pulse width
    uint16_t width = time - pwm->rising_edge_time;
    if ((width >= MIN_VALID_SERVO_PULSE_WIDTH) && (width <= MAX_VALID_SERVO_PULSE_WIDTH))
    {
      // pulse width seems valid
      pwm->pulse_width = width;
      pwm->no_signal_cycles = 0;
    }
    else
    {
      // debug toggle LED when bad pulse width is seen
      // LED1_PORT ^= (1<<LED1_PIN);
    }
  }
}

/** Get servo pulse width 
 * pulse width is rescaled and has a dead-zone removed
 */
int16_t getAvgServoPulseWidth(struct ServoPwm *pwm)
{
  // make sure read of pulse width is atomic
  cli();
  int16_t new_width = pwm->pulse_width;
  sei();

  int16_t width = pwm->average_pulse_width;
  width += (new_width-width)>>4;
  pwm->average_pulse_width = width;

  // offset pulse widths so values so output   
  width-=CENTER_SERVO_PULSE_WIDTH;
  
  if (width > SERVO_DEADZONE_WIDTH)
  {
    width -= SERVO_DEADZONE_WIDTH;
  }
  else if (width < -SERVO_DEADZONE_WIDTH)
  {
    width += SERVO_DEADZONE_WIDTH;
  }
  else
  {
    width = 0;
  }
  return width;
}



/**
 * Use pin change interrupt and TIM0 to measure input PWM pulse width
 * this uses a combination of TCNT0 and TIM0 overflow interrupt to
 * generate a 16bit timestamp value that alows a servo pulse between
 * 1 and 2ms to be measured.
 * 
 * PA1 : PWM1 : PCINT1
 * PA2 : PWM2 : PCINT2
 */
uint8_t pcint_last_in;
ISR(PCINT0_vect)
{
  // Read hardware registers immediately and in exact order specified
  uint8_t in = PINA; // read pin A
  MEMBAR();
  uint16_t time = getTimestamp();

  // The pin change interrupt could be triggered by
  // and change of one or both PWM pins, use difference between
  // last and current pin values to determine which pins changed
  uint8_t changed = in ^ pcint_last_in;
  pcint_last_in = in;  

  if (changed & (1<<PWM1_PIN))
  {
    char rising_edge = (in & (1<<PWM1_PIN));
    updateServoPwm(&pwm1, time, rising_edge);
  }

  if (changed & (1<<PWM2_PIN))
  {
    char rising_edge = (in & (1<<PWM2_PIN));
    updateServoPwm(&pwm2, time, rising_edge);
  }
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

  // Timer interrupt mask for timer0
  // Use timer0 overflow interrupt along with timer 0 value to
  // measure servo pwm pulse
  TIMSK0 = (1<<TOIE0);  

  // Enabled PCINT1 and PCINT2 for PA1 and PA2 pin change interrupts
  PCMSK0 = (1<<PCINT1) | (1<<PCINT2);
  GIMSK = (1<<PCIE0);
  pcint_last_in = PINA;

  // Enable interrupts
  sei();

  uint8_t stable_counter = 0;
  uint8_t ovf_counter = 0;
  while (1)
  {
    if (tim0_ovf_flag)
    {
      tim0_ovf_flag = 0;
      ++ovf_counter;      
      if (ovf_counter > 62)
      {
        ovf_counter = 0;
        //LED1_PORT ^= (1<<LED1_PIN);
        // when overflow counter reaches 62 it means about 
        // 62*256/8Mhz = 1.984ms have elapsed
        uint16_t time = getTimestamp();
        if (timeoutServoPwm(&pwm1, time) || timeoutServoPwm(&pwm2, time))
        {
          // servo signals stopped comming in, disable motors
          LED1_PORT &= ~(1<<LED1_PIN);
          LED2_PORT |= (1<<LED2_PIN);
          disableMotors();
          stable_counter = 0;
        }
        else 
        {
          LED1_PORT |= (1<<LED1_PIN);
          LED2_PORT &= ~(1<<LED2_PIN);

          // calculate new duty values from servo values
          // assume:
          //  pwm2 = forward/reverse
          //  pwm1 = rotate
          int16_t forward = getAvgServoPulseWidth(&pwm2);
          int16_t rotate  = getAvgServoPulseWidth(&pwm1);

          // range on forward and reverse values is typically 
          // -400*8 to 400*8 for Spektrum DX5 and 8Mhz clock

          // scale forward value  so that max forward value 400*8 is now 1024
          forward = (forward * 5)>>4;
          
          // scale rotate so that that max value 400 is now 1024
          rotate = (rotate * 5)>>4;

          int16_t duty1 = forward + rotate;
          int16_t duty2 = forward - rotate;

          if (stable_counter < 100)
          {
            ++stable_counter;
          }
          else
          {
            setMotor1(-duty1>>2);
            setMotor2(duty2>>2);
          }
        }
      } // end if (ovf_counter > 62)
    } // end if (tim0_ovf_flag)

  } // end while(1)

  return 0;
}
