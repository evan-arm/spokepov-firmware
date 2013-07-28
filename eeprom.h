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

#define SPIEE_CS_PORT PORTB
#define SPIEE_CS 4

#define SPI_EEPROM_READ 0x3
#define SPI_EEPROM_WRITE 0x2
#define SPI_EEPROM_WREN 0x6
#define SPI_EEPROM_RDSR 0x5
#define SPI_EEPROM_WRSR 0x1

void spieeprom_read(uint16_t addr, uint8_t *buff, uint8_t len);
uint8_t spieeprom_write(uint16_t addr, uint8_t* data, uint8_t len);

void spieeprom_read_into_leds(uint16_t addr, uint8_t side);

