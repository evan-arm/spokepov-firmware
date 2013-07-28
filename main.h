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

#define BACK 5
#define LATCH_SELECT_PORT PORTD
#define FRONT 4

#define BUTTON_PIN PIND
#define BUTTON 2

#define SENSOR_PORT PORTD
#define SENSOR_PIN PIND
#define SENSOR 3

#define SENSORPOWER 6
#define SPI_CLK1 _BV(USIWM0) | _BV(USICS0) | _BV(USITC)
#define SPI_CLK2 _BV(USIWM0) | _BV(USICS0) | _BV(USITC) | _BV(USICLK)

uint8_t spi_transfer(uint8_t c);
#define NOP asm("nop");


// all commands have the highest bit set
#define COMP_SUCCESS 0x80

#define COMP_CMD_SETFLED 0x81
#define COMP_CMD_SETBLED 0x82
#define COMP_CMD_CLRFLED 0x83
#define COMP_CMD_CLRBLED 0x84

#define COMP_CMD_RDEEPROM 0x85
#define COMP_CMD_WREEPROM 0x86
#define COMP_CMD_RDEEPROM16 0x87
#define COMP_CMD_WREEPROM16 0x88

uint8_t tx_computer_byte(uint8_t);

uint8_t internal_eeprom_read(uint8_t addr);
void internal_eeprom_write(uint8_t addr, uint8_t data);

void clock_leds(uint8_t select);
void set_led(uint8_t led, uint8_t side);

void go_to_sleep(void);

void delay_ms(unsigned char ms);
