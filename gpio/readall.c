/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2024 Gordon Henderson and contributors
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://github.com/WiringPi/WiringPi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
  int pin ;

  printf ("+------+---------+--------+\n") ;
  printf ("|  Pin | Digital | Analog |\n") ;
  printf ("+------+---------+--------+\n") ;

  for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
    printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

  printf ("+------+---------+--------+\n") ;
}


/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

#define MAX_ALTS 15
static const char unknown_alt[] = " - ";
static const char *alts [MAX_ALTS+1] =
{
  "IN", "OUT", "ALT1", "ALT2", "ALT3", "ALT4", "ALT5", "ALT6", "ALT7", "ALT8", "ALT9", "ALT10", "ALT11", "ALT12", "ALT13", "ALT14",
} ;


static const char* GetAltString(int alt) {

  if (alt>=0 && alt<=MAX_ALTS) {
    return alts[alt];
  }

  return unknown_alt;
}


static int physToWpi [64] =
{
  -1,           // 0
  -1, -1,       // 1, 2
   0, -1,
   1, -1,
   2,  3,
  -1,  4,
   5,  6,
   7, -1,
   8,  9,
  -1, 10,
  11, -1,
  12, 13,
  14, 15,
  -1, 16,       // 25, 26
  17, 18,
  19, -1,
  20, 21,
  22, -1,
  23, 24,
  25, 26,
  -1, 27,

  // Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
} ;

static char *physNames [64] =
{
  NULL,

  "    3.3V", "5V      ",
  "   SDA.8", "5V      ",
  "   SCL.8", "0V      ",
  "GPIO1_A4", "TXD.1   ",
  "      0v", "RXD.1   ",
  "GPIO1_A6", "GPIO4_A1",
  "GPIO1_A7", "0V      ",
  "GPIO1_B0", "GPIO1_A0",
  "    3.3V", "GPIO1_A1",
  "    MOSI", "0V      ",
  "    MISO", "GPIO3_B4",
  "    SCLK", "CE0     ",
  "      0V", "CE1     ",
  "   SDA.4", "SCL.4   ",
  "GPIO4_A3", "0V      ",
  "GPIO4_A4", "GPIO4_B2",
  "GPIO4_B3", "0V      ",
  "GPIO4_A0", "GPIO4_A2",
  "GPIO4_B0", "GPIO4_A5",
  "      0V", "GPIO4_B1",

    // Padding:
  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
} ;


/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| BCM | wPi |   Name  | Mode | Val| Physical |Val | Mode | Name    | wPi | BCM |
 *********************************************************************************
 */

static void readallPhys (int physPin)
{
  int pin ;

  if (physPinToGpio (physPin) == -1)
    printf (" |      |    ") ;
  else
    printf (" | %3d  | %3d", physPinToGpio (physPin), physToWpi [physPin]) ;

  printf (" | %s", physNames [physPin]) ;

  if (physToWpi [physPin] == -1)
    printf (" |       |  ") ;
  else
  {
    if      (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" | %5s", GetAltString(getAlt (pin))) ;
    printf (" | %d", digitalRead (pin)) ;
  }

// Pin numbers:

  printf (" | %2d", physPin) ;
  ++physPin ;
  printf (" || %-2d", physPin) ;

// Same, reversed

  if (physToWpi [physPin] == -1)
    printf (" |   |      ") ;
  else
  {
    if      (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" | %d", digitalRead (pin)) ;
    printf (" | %-5s", GetAltString(getAlt (pin))) ;
  }

  printf (" | %-5s", physNames [physPin]) ;

  if (physToWpi     [physPin] == -1)
    printf (" |     |     ") ;
  else
    printf (" | %-3d |  %-3d", physToWpi [physPin], physPinToGpio (physPin)) ;

  printf (" |\n") ;
}

/*
 * piPlusReadall:
 *	Read all the pins on the model A+ or the B+ or actually, all 40-pin Pi's
 *********************************************************************************
 */
const char piModelNamesShort[PI_MODELS_MAX][11] =
{
  "  HBox 5A ",	//  0
} ;

static void plus2header (int model)
{
  if (model<PI_MODELS_MAX && piModelNamesShort[model][0]!='\0') {
    printf (" +------+-----+----------+-------+---+%s+---+-------+----------+-----+------+\n", piModelNamesShort[model]);
  } else {
    printf (" +------+-----+----------+-------+---+---- ? ---+---+-------+----------+-----+------+\n");
  }
}


static void piPlusReadall (int model)
{
  int pin ;

  plus2header (model) ;

  printf (" | GPIO | wPi |   Name   |  Mode | V | Physical | V |  Mode |   Name   | wPi | GPIO |\n") ;
  printf (" +------+-----+----------+-------+---+----++----+---+-------+----------+-----+------+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin) ;
  printf (" +------+-----+----------+-------+---+----++----+---+-------+----------+-----+------+\n") ;
  printf (" | GPIO | wPi |   Name   |  Mode | V | Physical | V | Mode  |   Name   | wPi | GPIO |\n") ;

  plus2header (model) ;
}


/*
 * doReadall:
 *	Generic read all pins called from main program. Works out the Pi type
 *	and calls the appropriate function.
 *********************************************************************************
 */

void doReadall (void)
{
  int model;

  if (wiringPiNodes != NULL)	// External readall
  {
    doReadallExternal () ;
    return ;
  }

  piBoardId (&model) ;

  if ((model == PI_MODEL_HAIBOX_5A))
    piPlusReadall (model) ;
  else
    printf ("Oops - unable to determine board type... model: %d\n", model) ;
}


/*
 * doQmode:
 *	Query mode on a pin
 *********************************************************************************
 */

void doQmode (int argc, char *argv [])
{
  int pin ;

  if (argc != 3)
  {
    fprintf (stderr, "Usage: %s qmode pin\n", argv [0]) ;
    exit (EXIT_FAILURE) ;
  }

  pin = atoi (argv [2]) ;
  printf ("%s\n", GetAltString(getAlt (pin))) ;
}
