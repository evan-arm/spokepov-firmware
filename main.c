/******************************************
SpokePOV V1.0 firmware

SpokePOV firmware is distributed under CC license. 
For more info on CC go to www.creativecommons.org
For more info on SpokePOV go to www.ladyada.net/make/spokepov

Creative Commons Deed
Attribution-ShareAlike 2.5
You are free:
    * to copy, distribute, display, and perform the work
    * to make derivative works

Under the following conditions:
Attribution.
   You must attribute the work in the manner specified by the 
   author or licensor.
Share Alike. 
   If you alter, transform, or build upon this work, you may 
   distribute the resulting work only under a license identical to this one.

    * For any reuse or distribution, you must make clear to others 
      the license terms of this work.
    * Any of these conditions can be waived if you get permission 
      from the copyright holder.

Your fair use and other rights are in no way affected by the above.

A more detailed version of this license is available at:
http://creativecommons.org/licenses/by-nc-sa/2.5/legalcode
******************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/signal.h>
#include "main.h"
#include "eeprom.h"

#define EEMWE   2
#define EEWE    1

#define ANIMATE 1
uint8_t animation_time = 6;
volatile uint16_t anim_timer = 0;
volatile uint16_t anim_eeprom_offset = 0;

#define HALL_DEBOUNCE_THRESH 4  
#define BUTTON_DEBOUNCE  100  // in ms

#define NUM_PIXELS 256     // max is 255 (since it is 8 bit value)

#define STANDBY_TIMEOUT 5       // in seconds
#define POWEROFF_TIMEOUT 2*60   // in seconds

// address that stores the rotation offset
#define EEPROM_ROTATION_OFFSET 0x00
// address that stores the mirror flag
#define EEPROM_MIRROR 0x01
#define EEPROM_ANIMATION 0x02

uint8_t mirror;

uint8_t fleds[4], bleds[4];  // front and back LEDs

volatile uint8_t hall_debounce;
volatile uint16_t sensor_timer;

volatile uint8_t stopcomputertx = 0;

ISR (TIMER0_OVF_vect) {
  if (hall_debounce != 0xFF)
    hall_debounce++;
  
  if (sensor_timer != 0xFFFF)
    sensor_timer++;

}

volatile uint16_t curr_eeprom_addr = 0;
// gets called once per pixel


ISR (TIMER1_COMPA_vect) {
  uint16_t eepromaddr;
  uint16_t chase;

  sei();

  eepromaddr = curr_eeprom_addr;

  if (sensor_timer < ((F_CPU/NUM_PIXELS)/256 * STANDBY_TIMEOUT)) {    
    // less than ~5 seconds since last sensor
    eepromaddr %= NUM_PIXELS * 4;
    spieeprom_read_into_leds(eepromaddr + anim_eeprom_offset, FRONT);
    if (mirror) {
      spieeprom_read_into_leds(anim_eeprom_offset + (1024UL-eepromaddr), BACK);
    } else {
      LATCH_SELECT_PORT |= _BV(BACK);
      NOP; NOP; NOP; NOP;
      LATCH_SELECT_PORT &= ~_BV(BACK);
    }
      
    /*
    fleds[0] = fleds[1] = fleds[2] = fleds[3] = 0xFF;
    fleds[eepromaddr / 8] = ~(_BV(eepromaddr % 8));
    clock_leds(FRONT);
    eepromaddr++;
    if (eepromaddr > 32)
      eepromaddr = 0;
    */

    // make sure no other interrupt tries to write to this address at the same
    // time, shut off interrupts. this is very quick, though.   
    cli();
    
    if (eepromaddr == (curr_eeprom_addr%(NUM_PIXELS*4))) {
      curr_eeprom_addr = eepromaddr+4;
      }
    
      //curr_eeprom_addr = eepromaddr;
    sei();
    
  } else {

    // no signal for a while, go into chill mode
    // turn off this pixel timer
    cli();
    TCCR1B &= ~0x7;
    sei();


    // start LED chaser sequence for ~90 sec, then turn all off
    clear_both_leds();

    for (chase = 10; chase < 1800; chase++) {
        fleds[((chase%30)/8)] &= ~(_BV((chase%30)%8));
        fleds[(((chase-10)%30)/8)] |= _BV(((chase-10)%30)%8);
        clock_both_leds();
        delay_ms(50);
    }
    clear_both_leds();

  }

}

ISR (INT0_vect) {
  uint16_t timer;

  timer = 0;
  while (! (BUTTON_PIN & _BV(BUTTON))) {
    // while button is still held down
    timer++;
    delay_ms(1);
  }
  // button must be held down for a bit
  if (timer > BUTTON_DEBOUNCE) {
    if (timer < 500UL) {
      // a short button press (less than 1/2 second) resets
      // set up watchdog timer that will reset the device for us
      WDTCSR = _BV(WDE);
      while (1);
    } else {
      // long button press means shutdown
      sensor_timer = 0xFFFF;
    }
  }
}

ISR (INT1_vect) {

  if (hall_debounce > HALL_DEBOUNCE_THRESH) {
    stopcomputertx = 1;

#ifdef ANIMATE
  if (anim_timer != animation_time) {
    anim_timer++;
  } else {
    anim_timer = 0;
    anim_eeprom_offset += 1024;
  }
#endif

    // we know the number of ms since the last hall sensor trigger
    // and there are 128 radial 'pixels' per sweep so divide to get
    // the necessary ms between the pixel interrupts
    
    // now just make timer1 trigger at that rate!
    
    TCNT1 = 0;
    
    if ((sensor_timer < 0xFF) && (sensor_timer > 0x3)) {
      OCR1A = (sensor_timer << 8) | TCNT0;
      TCNT0 = 0;
      curr_eeprom_addr = 
	(internal_eeprom_read(EEPROM_ROTATION_OFFSET) % NUM_PIXELS) * 4;
      mirror = internal_eeprom_read(EEPROM_MIRROR);
      TCCR1B |= _BV(CS10);
      TIMSK |= _BV(OCIE1A);
    } else {
      set_led(2, FRONT);
      set_led(2, BACK);
      TCCR1B &= ~_BV(CS10);
    }   
    sensor_timer = 0;
  }
  hall_debounce = 0;
}

void ioinit(void) {
  DDRD = 0x73; // input on PD2 (button), PD3 (sensor), all other output
  DDRB = 0xDF; // input on MOSI/DI (for SPI), all others output

  // deselect EEPROM
  PORTB = _BV(SPIEE_CS);

  // turn on pullup for button// and power and pullup for sensor
  PORTD = (_BV(BUTTON) | _BV(SENSOR) | _BV(SENSORPOWER))  
    & ~_BV(FRONT) & ~_BV(BACK);  // and deselect latches

  // interrupt on INT1 pin falling edge (sensor triggered) and
  // interrupt on INT0 pin falling edge (button press!)
  MCUCR = _BV(ISC11) & ~_BV(ISC01) & ~_BV(ISC00) &  ~_BV(ISC10);

  // turn on interrupts!
  GIMSK = _BV(INT1) | _BV(INT0);

  TCCR0A = 0; // normal, overflow (count up to 256 == num pixels)
  //TCCR0B = _BV(CS00); // no clock div
  TCCR0B = _BV(CS02); // clk/256
  TIMSK |= _BV(TOIE0);  // turn on overflow interrupt
  
  // create pixel timer (T1)
  TCCR1A = 0;
  TCCR1B = _BV(WGM12);

  hall_debounce = 0;
  sensor_timer = 0;
}


void delay_ms(unsigned char ms)
{
  unsigned short delay_count = F_CPU / 4000;
  
  unsigned short cnt;
  asm volatile ("\n"
		"L_dl1%=:\n\t"
		"mov %A0, %A2\n\t"
		"mov %B0, %B2\n"
		"L_dl2%=:\n\t"
		"sbiw %A0, 1\n\t"
		"brne L_dl2%=\n\t"
		"wdr\n\t"
		"dec %1\n\t" "brne L_dl1%=\n\t":"=&w" (cnt)
		:"r"(ms), "r"((unsigned short) (delay_count))
		);
}

void clock_leds(uint8_t select) {
  uint8_t *leds;

  if (select == FRONT)
    leds = fleds;
  else
    leds = bleds;

  spi_transfer(leds[3]);
  spi_transfer(leds[2]);
  spi_transfer(leds[1]);
  spi_transfer(leds[0]);
  LATCH_SELECT_PORT |= _BV(select);
  NOP; NOP; NOP; NOP;
  LATCH_SELECT_PORT &= ~_BV(select);
}

void clock_both_leds() {
  clock_leds(FRONT);
  LATCH_SELECT_PORT |= _BV(BACK);
  NOP; NOP; NOP; NOP;
  LATCH_SELECT_PORT &= ~_BV(BACK);
}

void clear_both_leds() {
  fleds[0] = fleds[1] = fleds[2] = fleds[3] = 0xFF;
  clock_both_leds();
}

void set_led(uint8_t led, uint8_t side) {
  uint8_t *leds;

  if (side == FRONT)
    leds = fleds;
  else
    leds = bleds;

  leds[0] = leds[1] = leds[2] = leds[3] = 0xFF;
  leds[led/8] = ~_BV(led%8);

  clock_leds(side);
}
void test_leds(void) {
  uint8_t i;

  for(i=0; i< 33; i++) {
    set_led(i, FRONT);
    set_led(33-i, BACK);
    delay_ms(50);
  }
}
 
 
int main(void) {
  uint8_t buff[16];
  uint8_t i, n;
  uint8_t cmd;
  uint16_t addr;

  cmd = MCUSR;  // find out why we reset
  // clear reset flags immediately
  MCUSR = 0;
  // turn on watchdog timer immediately, this protects against
  // a 'stuck' system by resetting it
  WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // 1 second
  
  ioinit();
  if ((cmd & _BV(PORF)) != 0)
    test_leds();  // test the LEDs only on power reset


  set_led(cmd+2, FRONT);  // show the reason for the last reset
  //set_led(2, FRONT);
  set_led(2, BACK);

  animation_time = internal_eeprom_read(EEPROM_ANIMATION);

  sei();
  
  // look for computer commands  
  while (1) {
    sei();
    cmd = tx_computer_byte(0);

    if (cmd == 0)
      break;
    else 
      cli();

    sensor_timer = 0;      // overloaded to reset communications timeout

    switch (cmd) {
    case COMP_CMD_RDEEPROM:
    case COMP_CMD_RDEEPROM16:
      if  (cmd == COMP_CMD_RDEEPROM16)
	n = 16;
      else
	n = 1;

      addr = tx_computer_byte(0);
      addr <<= 8;
      addr |= tx_computer_byte(0);
      
      if ((addr & 0x8000) != 0) {
      	tx_computer_byte(internal_eeprom_read(addr & 0xFF));
      } else {
	
	spieeprom_read(addr, buff, n);
	for (i=0; i<n; i++) {
	  tx_computer_byte(buff[i]);
	}
      }

      tx_computer_byte(COMP_SUCCESS);
      break;

    case COMP_CMD_WREEPROM:
    case COMP_CMD_WREEPROM16:
      if (cmd == COMP_CMD_WREEPROM16)
	n = 16;
      else
	n = 1;

      addr = tx_computer_byte(0);
      addr <<= 8;
      addr |= tx_computer_byte(0);
      set_led((addr/16)%32,FRONT); 
      for (i=0; i<n; i++) {
	buff[i] = tx_computer_byte(0);
      }

      tx_computer_byte(COMP_SUCCESS);

      if ((addr & 0x8000) != 0) {
	internal_eeprom_write(addr & 0xFF, buff[0]);
      } else {
	spieeprom_write(addr, buff, n);
      }
      break;
    }
  }

  //WDTCSR = _BV(WDE) | _BV(WDP2); // 1/4 second
  while (1) {
    PORTD |= 0x1;
    asm("wdr");
    PORTD &= ~0x1;

    if (sensor_timer == 0xFFFF) {
      // ~3 minutes since last sensor
      // no interrupts until we're done
      cli();
      // turn off all LEDs
      clear_both_leds();
      
      // turn off sensor
      SENSOR_PORT &= ~_BV(SENSORPOWER);
      
      // deselect EEPROM
      SPIEE_CS_PORT |= _BV(SPIEE_CS);      // pull CS high to deselect
      
      //go into sleep mode
      // turn off WDT (use interrupt to reset!)
      WDTCSR |= _BV(WDCE) | _BV(WDE);
      WDTCSR = 0;
      MCUCR |= _BV(SM1) | _BV(SM0) | _BV(SE);
      sei();
      asm("sleep");
    }
  }
  PORTD |= 0x2;
}


uint8_t tx_computer_byte(uint8_t b) {
  uint8_t v;

  DDRA = 0x0;           // outputs
  DDRB = 0x5F;          // input on MOSI/DI (for SPI), all others output
  USICR = _BV(USIWM0) | _BV(USICS1);
  USIDR = b;                 // transfer 0x0
  USISR = _BV(USIOIF);       // ready
  while (! (USISR & _BV(USIOIF)) ) {
    asm("wdr");

    if (sensor_timer == 0xFFFF) { 
      stopcomputertx = 1;
    }
    if (stopcomputertx) {
      break;
    }
  }

  v = USIDR;

  DDRB = 0xDF; 
  USICR = 0;

  return v;
}

uint8_t spi_transfer(uint8_t c) {
  USIDR = c;
  USISR = _BV(USIOIF);
  while (! (USISR & _BV(USIOIF))) {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
    //NOP;  // slow down so that the eeprom isnt overwhelmed
  }
  return USIDR;
}

uint8_t internal_eeprom_read(uint8_t addr) {
  loop_until_bit_is_clear(EECR, EEWE); // wait for last write to finish
  EEAR = addr;
  EECR |= _BV(EERE);        // start EEPROM read
  return EEDR;            // takes only 1 cycle
}

void internal_eeprom_write(uint8_t addr, uint8_t data) {
  loop_until_bit_is_clear(EECR, EEWE); // wait for last write to finish
  EEAR = addr;
  EEDR = data;
  cli();                // turn off interrupts 
  EECR |= _BV(EEMWE);     // these instructions must happen within 4 cycles
  EECR |= _BV(EEWE);
  sei();                // turn on interrupts again
}
 
