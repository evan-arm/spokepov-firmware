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
#include "eeprom.h"
#include "main.h"

uint8_t spieeprom_write(uint16_t addr, uint8_t *data, uint8_t len) {
  uint8_t i;

  /* check if there is a write in progress, wait */
  do {
    asm("wdr");
    SPIEE_CS_PORT &= ~_BV(SPIEE_CS);       // pull CS low
    NOP; NOP; NOP; NOP;                    // wait 500 ns, CS setup time
    
    spi_transfer(SPI_EEPROM_RDSR);         // write "READ STATUS REG" cmd
    i = spi_transfer(0);              // read status
    
    SPIEE_CS_PORT |= _BV(SPIEE_CS);        // pull CS high
    
  } while ((i & 0x1) != 0);
	/* set the spi write enable latch */
	
  SPIEE_CS_PORT &= ~_BV(SPIEE_CS);         // pull CS low
  NOP; NOP; NOP; NOP;                      // wait 500 ns, CS setup time
  
  spi_transfer(SPI_EEPROM_WREN);           // send command
  
  SPIEE_CS_PORT |= _BV(SPIEE_CS);          // pull CS high
  NOP; NOP; NOP; NOP;                      // 500ns hold time
  SPIEE_CS_PORT &= ~_BV(SPIEE_CS);         // pull CS low
  
  spi_transfer(SPI_EEPROM_WRITE);          // send command
  spi_transfer(addr >> 8);                 // send high addr 
  spi_transfer(addr & 0xFF);               // send low addr

  for (i = 0; i< len; i++)
    spi_transfer(data[i]);                      // send data

  NOP; NOP; NOP; NOP;                      // 500ns hold time

  SPIEE_CS_PORT |= _BV(SPIEE_CS);          // pull CS high

  return 0;
}

void spieeprom_read(uint16_t addr, uint8_t *buff, uint8_t len) {
  uint8_t i;

  SPIEE_CS_PORT &= ~_BV(SPIEE_CS); // pull CS low
  NOP; NOP; NOP; NOP;

  spi_transfer(SPI_EEPROM_READ);           // send READ command
  spi_transfer(addr >> 8);                 // send high addr 
  spi_transfer(addr & 0xFF);               // send low addr

  for (i=0; i<len; i++)
    buff[i] = spi_transfer(0);

  SPIEE_CS_PORT |= _BV(SPIEE_CS);      // pull CS high
}


void spieeprom_read_into_leds(uint16_t addr, uint8_t side) {
  uint8_t x;
  
  SPIEE_CS_PORT &= ~_BV(SPIEE_CS); // pull CS low
  NOP; NOP; NOP; NOP;

  spi_transfer(SPI_EEPROM_READ);           // send READ command
  spi_transfer(addr >> 8);                 // send high addr 
  spi_transfer(addr & 0xFF);               // send low addr

  x = spi_transfer(0);
  x = spi_transfer(x);
  x = spi_transfer(x);
  x = spi_transfer(x);
  spi_transfer(x);

  SPIEE_CS_PORT |= _BV(SPIEE_CS);      // pull CS high

  
  LATCH_SELECT_PORT |= _BV(side);
  NOP; NOP; NOP; NOP;
  LATCH_SELECT_PORT &= ~_BV(side);
}
