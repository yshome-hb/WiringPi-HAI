/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2025 Gordon Henderson and contributors
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *    https://github.com/WiringPi/WiringPi
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/utsname.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "../version.h"

#include "rk3588_soc.h"

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"
#define	ENV_GPIOMEM	"WIRINGPI_GPIOMEM"

// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure

struct wiringPiNodeStruct *wiringPiNodes = NULL ;

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:
//
// Updates in September 2015 - all now static variables (and apologies for the caps)
//	due to the Pi v2, v3, etc. and the new /dev/gpiomem interface

const char* gpiomem_global = "/dev/mem";

static int wiringPiSetuped = FALSE ;

const char *piModelNames [PI_MODELS_MAX] =
{
  "HaiBox 5A",	//  0
} ;

// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;

// Function

static struct wiringPiFuncStruct piFunc = { 0 };

// Debugging & Return codes

int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;


// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.

static int *pinToGpio ;

static int pinToGpio_HaiBox_5A [64] =
{
   63, 62, // 0, 1
   36, 46, // 2, 3
   47, 38, // 4, 5
  129, 39, // 6, 7
   40, 32, // 8, 9
   33, 41, //10,11
   42,108, //12,13
   43, 44, //14,15
   45, 34, //16,17
   35,131, //18,19
  132,138, //20,21
  139,128, //22,23
  130,136, //24,25
  133,137, //26,27,

// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;


// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;

static int physToGpio_HaiBox_5A [64] =
{
   -1,		 // 0
   -1, -1, // 1, 2
   63, -1,
   62, -1,
   36, 46,
   -1, 47,
   38,129,
   39, -1,
   40, 32,
   -1, 33,
   41, -1,
   42,108,
   43, 44,
   -1, 45,
   34, 35,
  131, -1,
  132,138,
  139, -1,
  128,130,
  136,133,
   -1,137, //39,40

  // Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;


int ToBCMPin(int* pin) {
  if (*pin<0 || *pin>63) {
    return FALSE;
  }
  switch(wiringPiMode) {
    case WPI_MODE_PINS:
      *pin = pinToGpio[*pin];
      break;
    case WPI_MODE_PHYS:
      *pin = physToGpio[*pin];
      break;
    case WPI_MODE_GPIO:
      return TRUE;
    default:
      return FALSE;
  }
  return TRUE;
}

/*
 * Functions
 *********************************************************************************
 */


/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * setupCheck
 *	Another sanity check because some users forget to call the setup
 *	function. Mosty because they need feeding C drip by drip )-:
 *********************************************************************************
 */

static void setupCheck (const char *fName)
{
  if (!wiringPiSetuped)
  {
    fprintf (stderr, "%s: You have not called one of the wiringPiSetup\n"
	"  functions, so I'm aborting your program before it crashes anyway.\n", fName) ;
    exit (EXIT_FAILURE) ;
  }
}


void PrintSystemStdErr () {
  struct utsname sys_info;
  if (uname(&sys_info) == 0) {
    fprintf (stderr, "      WiringPi    : %d.%d\n", VERSION_MAJOR, VERSION_MINOR);
    fprintf (stderr, "      system name : %s\n", sys_info.sysname);
    //fprintf (stderr, "  node name   : %s\n", sys_info.nodename);
    fprintf (stderr, "      release     : %s\n", sys_info.release);
    fprintf (stderr, "      version     : %s\n", sys_info.version);
    fprintf (stderr, "      machine     : %s\n", sys_info.machine);
    if (strstr(sys_info.machine, "arm") == NULL && strstr(sys_info.machine, "aarch")==NULL) {
      fprintf (stderr, " -> This is not an ARM architecture; it cannot be a Raspberry Pi.\n") ;
    }
  }
}


void piFunctionOops (const char *function, const char* suggestion, const char* url)
{
  fprintf (stderr, "Oops: Function %s is not supported\n", function) ;
  PrintSystemStdErr();
  if (suggestion) {
    fprintf (stderr, " -> Please %s\n", suggestion) ;
  }
  if (url) {
    fprintf (stderr, " -> See info at %s\n", url) ;
  }
  fprintf (stderr, " -> Check at https://github.com/wiringpi/wiringpi/issues.\n\n") ;
  exit (EXIT_FAILURE) ;
}

void ReportDeviceError(const char *function, int pin, const char *mode, int ret) {
  fprintf(stderr, "wiringPi: ERROR: ioctl %s of %d (%s) returned error '%s' (%d)\n", function, pin, mode, strerror(errno), ret);
}


/*
 * piGpioLayout:
 *	Return a number representing the hardware revision of the board.
 *	This is not strictly the board revision but is used to check the
 *	layout of the GPIO connector - and there are 2 types that we are
 *	really interested in here. The very earliest Pi's and the
 *	ones that came after that which switched some pins ....
 *
 *	Revision 1 really means the early Model A and B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *		... and the Pi 2 - which is a B+ ++  ...
 *		... and the Pi 0 - which is an A+ ...
 *
 *	The main difference between the revision 1 and 2 system that I use here
 *	is the mapping of the GPIO pins. From revision 2, the Pi Foundation changed
 *	3 GPIO pins on the (original) 26-way header - BCM_GPIO 22 was dropped and
 *	replaced with 27, and 0 + 1 - I2C bus 0 was changed to 2 + 3; I2C bus 1.
 *
 *	Additionally, here we set the piModel2 flag too. This is again, nothing to
 *	do with the actual model, but the major version numbers - the GPIO base
 *	hardware address changed at model 2 and above (not the Zero though)
 *
 *********************************************************************************
 */
 const char* revfile = "/etc/haibox-release";

void piGpioLayoutOops (const char *why)
{
  fprintf (stderr, "Oops: Unable to determine board revision from %s\n", revfile) ;
  PrintSystemStdErr();
  fprintf (stderr, " -> %s\n", why) ;
	fprintf (stderr, " ->  You'd best google the error to find out why.\n");
  exit (EXIT_FAILURE) ;
}


/*
 * piBoardId:
 *	Return the real details of the board we have.
 *
 *	This is undocumented and really only intended for the GPIO command.
 *	Use at your own risk!
 *
 *	Seems there are some boards with 0000 in them (mistake in manufacture)
 *	So the distinction between boards that I can see is:
 *
 *		0000 - Error
 *		0001 - Not used
 *
 *	Original Pi boards:
 *		0002 - Model B,  Rev 1,   256MB, Egoman
 *		0003 - Model B,  Rev 1.1, 256MB, Egoman, Fuses/D14 removed.
 *
 *	Newer Pi's with remapped GPIO:
 *		0004 - Model B,  Rev 1.2, 256MB, Sony
 *		0005 - Model B,  Rev 1.2, 256MB, Egoman
 *		0006 - Model B,  Rev 1.2, 256MB, Egoman
 *
 *		0007 - Model A,  Rev 1.2, 256MB, Egoman
 *		0008 - Model A,  Rev 1.2, 256MB, Sony
 *		0009 - Model A,  Rev 1.2, 256MB, Egoman
 *
 *		000d - Model B,  Rev 1.2, 512MB, Egoman	(Red Pi, Blue Pi?)
 *		000e - Model B,  Rev 1.2, 512MB, Sony
 *		000f - Model B,  Rev 1.2, 512MB, Egoman
 *
 *		0010 - Model B+, Rev 1.2, 512MB, Sony
 *		0013 - Model B+  Rev 1.2, 512MB, Embest
 *		0016 - Model B+  Rev 1.2, 512MB, Sony
 *		0019 - Model B+  Rev 1.2, 512MB, Egoman
 *
 *		0011 - Pi CM,    Rev 1.1, 512MB, Sony
 *		0014 - Pi CM,    Rev 1.1, 512MB, Embest
 *		0017 - Pi CM,    Rev 1.1, 512MB, Sony
 *		001a - Pi CM,    Rev 1.1, 512MB, Egoman
 *
 *		0012 - Model A+  Rev 1.1, 256MB, Sony
 *		0015 - Model A+  Rev 1.1, 512MB, Embest
 *		0018 - Model A+  Rev 1.1, 256MB, Sony
 *		001b - Model A+  Rev 1.1, 256MB, Egoman
 *
 *	A small thorn is the olde style overvolting - that will add in
 *		1000000
 *
 *	The Pi compute module has an revision of 0011 or 0014 - since we only
 *	check the last digit, then it's 1, therefore it'll default to not 2 or
 *	3 for a	Rev 1, so will appear as a Rev 2. This is fine for the most part, but
 *	we'll properly detect the Compute Module later and adjust accordingly.
 *
 * And then things changed with the introduction of the v2...
 *
 * For Pi v2 and subsequent models - e.g. the Zero:
 *
 *   [USER:8] [NEW:1] [MEMSIZE:3] [MANUFACTURER:4] [PROCESSOR:4] [TYPE:8] [REV:4]
 *   NEW          23: will be 1 for the new scheme, 0 for the old scheme
 *   MEMSIZE      20: 0=256M 1=512M 2=1G
 *   MANUFACTURER 16: 0=SONY 1=EGOMAN 2=EMBEST
 *   PROCESSOR    12: 0=2835 1=2836
 *   TYPE         04: 0=MODELA 1=MODELB 2=MODELA+ 3=MODELB+ 4=Pi2 MODEL B 5=ALPHA 6=CM
 *   REV          00: 0=REV0 1=REV1 2=REV2
 *********************************************************************************
 */

void piBoardId (int *model)
{
	FILE *recv_fd;
  const int maxlength = 120;
  char line [maxlength+1];
  char revision[maxlength+1];
  char *c;
	unsigned int i = 0;

  //piGpioLayoutOops ("this is only a test case");

	if ((recv_fd = fopen (revfile, "r")) == NULL) {
			piGpioLayoutOops ("Unable to open /etc/haibox-release.");
  }

	while (fgets (line, maxlength, recv_fd) != NULL) {
	  if (strncmp (line, "BOARD=", 6) == 0)
		  break;
  }

	fclose (recv_fd) ;

	if (strncmp (line, "BOARD=", 6) != 0)
		piGpioLayoutOops ("No \"Revision\" line") ;

	if (wiringPiDebug)
		printf ("piBoardId: Revision string: %s\n", line) ;

  // Chomp trailing CR/NL
	for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
		*c = 0 ;

  // Need to work out if it's using the new or old encoding scheme:
	// Scan to the first character of the revision number
	for (c = line ; *c ; ++c)
    if (*c == '=')
      break ;

	if (*c != '=')
    piGpioLayoutOops ("Revision line (no equal)");

  c++;
	for (i = 0; *c ; c++)
		revision[i++] = *c;
	revision[i] = '.';

	if (wiringPiDebug)
		printf ("piBoardId: Board string: %s\n", revision) ;

	/**/ if (strncmp(revision, "haibox-5a.", 10) == 0) { *model = PI_MODEL_HAIBOX_5A; }
  else { piGpioLayoutOops ("Unkown board model"); }

	if (wiringPiDebug)
		printf("piBoardId: model = %d\n", *model);
}


/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
  return pinToGpio [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
  return physToGpio [physPin & 63] ;
}


/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrivePin (int pin, int value)
{
  (void)pin;
  (void)value;
}


void setPadDrive (int group, int value)
{
  (void)group;
  (void)value;
}


/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int getAlt (int pin)
{
  if (!ToBCMPin(&pin)) {
    return 0;
  }

  if (!piFunc.getPinMode) {
    return 0;
  }

  return piFunc.getPinMode(pin);
}


/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */

void pwmSetMode (int mode)
{
  (void)mode;
}


/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */

void pwmSetRange (int pin, unsigned int range)
{
  if (wiringPiDebug)
    printf ("pwmSetRange: pin:%d range:%d\n", pin, range) ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    if (!ToBCMPin(&pin)) {
      return;
    }

    if (piFunc.getPinMode) {
      piFunc.pwmRange(pin, range);
    }
  }
}


/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 *********************************************************************************
 */

void pwmSetClock (int pin, unsigned int divisor)
{
  if (wiringPiDebug)
    printf ("pwmSetClock: pin:%d divisor:%d\n", pin, divisor) ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    if (!ToBCMPin(&pin)) {
      return;
    }

    if (piFunc.pwmClock) {
      piFunc.pwmClock(pin, divisor);
    }
  }
}


/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static void pinModeDummy         (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int mode)  { return ; }
static void pullUpDnControlDummy (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int pud)   { return ; }
static  int digitalReadDummy     (UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return LOW ; }
static void digitalWriteDummy    (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static void pwmWriteDummy        (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static  int analogReadDummy      (UNU struct wiringPiNodeStruct *node, UNU int pin)                { return 0 ; }
static void analogWriteDummy     (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
  int    pin ;
  struct wiringPiNodeStruct *node ;

// Minimum pin base is 64

  if (pinBase < 64)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

// Check all pins in-case there is overlap:

  for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
    if (wiringPiFindNode (pin) != NULL)
      (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

  node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
  if (node == NULL)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

  node->pinBase          = pinBase ;
  node->pinMax           = pinBase + numPins - 1 ;
  node->pinMode          = pinModeDummy ;
  node->pullUpDnControl  = pullUpDnControlDummy ;
  node->digitalRead      = digitalReadDummy ;
  node->digitalWrite     = digitalWriteDummy ;
  node->pwmWrite         = pwmWriteDummy ;
  node->analogRead       = analogReadDummy ;
  node->analogWrite      = analogWriteDummy ;
  node->next             = wiringPiNodes ;
  wiringPiNodes          = node ;

  return node ;
}


/*
 *********************************************************************************
 * Core Functions
 *********************************************************************************
 */

/*
 * pinModeAlt:
 *	This is an un-documented special to let you set any pin to any mode
 *********************************************************************************
 */

void pinModeAlt (int pin, int alt)
{
  int origPin = pin ;

  setupCheck ("pinModeAlt") ;

  if ((origPin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (!ToBCMPin(&origPin)) {
      return;
    }

    softPwmStop  (origPin) ;
    softToneStop (origPin) ;

    if (piFunc.setPinAlt)
      piFunc.setPinAlt(origPin, alt);
  }
}


/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */

void pinMode (int pin, int mode)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int origPin = pin ;

  if (wiringPiDebug)
    printf ("pinMode: pin:%d mode:%d\n", pin, mode) ;

  setupCheck ("pinMode") ;

  if ((origPin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    if (!ToBCMPin(&origPin)) {
      return;
    }

    softPwmStop  (origPin) ;
    softToneStop (origPin) ;

    if (INPUT==mode  || PM_OFF==mode) {
      if (piFunc.setPinMode)
        piFunc.setPinMode(origPin, INPUT);
    } else if (mode == OUTPUT) {
      if (piFunc.setPinMode)
        piFunc.setPinMode(origPin, OUTPUT);
    } else if (mode == SOFT_PWM_OUTPUT) {
      softPwmCreate (origPin, 0, 100) ;
    } else if (mode == SOFT_TONE_OUTPUT) {
      softToneCreate (origPin) ;
    } else if (mode == PWM_TONE_OUTPUT) {
      pinMode (origPin, PWM_OUTPUT) ;	// Call myself to enable PWM mode
      pwmSetMode (PWM_MODE_MS) ;
    }
    else if (PWM_OUTPUT==mode || PWM_MS_OUTPUT==mode || PWM_BAL_OUTPUT==mode) {
      if (piFunc.setPinMode)
        piFunc.setPinMode(origPin, PWM_OUTPUT);
    }
    else if (mode == GPIO_CLOCK) {

    }
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pinMode (node, pin, mode) ;
    return ;
  }
}


/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  setupCheck ("pullUpDnControl") ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (!ToBCMPin(&pin)) {
      return;
    }

    if (piFunc.setPullUpDn)
      piFunc.setPullUpDn(pin, pud);
  }
  else						// Extension module
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pullUpDnControl (node, pin, pud) ;
    return ;
  }
}

//*********************************************


/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */

int digitalRead (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (!ToBCMPin(&pin)) {
      return 0;
    }

    if (!piFunc.digitalRead) {
      return 0;
    }

    return piFunc.digitalRead(pin);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}


/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (!ToBCMPin(&pin)) {
      return;
    }

    if (piFunc.digitalWrite)
      piFunc.digitalWrite(pin, value);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value) ;
  }
}


void pwmWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  setupCheck ("pwmWrite") ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (!ToBCMPin(&pin)) {
      return;
    }

    if (piFunc.pwmWrite)
      piFunc.pwmWrite(pin, value);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pwmWrite (node, pin, value) ;
  }
}


/*
 * analogRead:
 *	Read the analog value of a given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return 0 ;
  else
    return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return ;

  node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
  setupCheck ("pwmToneWrite") ;

  if (freq == 0)
    pwmWrite (pin, 0) ;             // Off
  else
  {
    int range = 600000 / freq ;
    pwmSetRange (pin, range) ;
    pwmWrite    (pin, freq / 2) ;
  }
}


/*
 * digitalWriteByte:
 * digitalReadByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change
 *	to set the outputs bits to zero, then another change to set the 1's
 *	Reading is just bit fiddling.
 *	These are wiringPi pin numbers 0..7, or BCM_GPIO pin numbers
 *	17, 18, 22, 23, 24, 24, 4 on a Pi v1 rev 0-3
 *	17, 18, 27, 23, 24, 24, 4 on a Pi v1 rev 3 onwards or B+, 2, 3, zero
 *********************************************************************************
 */

void digitalWriteByte (const unsigned int value)
{
  int pin ;
  uint32_t mask = 0x01 ;

  if (wiringPiMode == WPI_MODE_GPIO_SYS)
  {
    return ;
  }
  else
  {
    for (pin = 0 ; pin < 32 ; ++pin)
    {
      if ((value & mask) == 0)
        digitalWrite(pin, 0);
      else
        digitalWrite(pin, 1);

      mask <<= 1 ;
    }
  }
}

unsigned int digitalReadByte (void)
{
  int pin, x ;
  uint32_t data = 0 ;

  if (wiringPiMode == WPI_MODE_GPIO_SYS)
  {
    return 0;
  }
  else
  {
    for (pin = 0 ; pin < 32 ; ++pin)
    {
      x = digitalRead(pin);
      data = (data << 1) | x ;
    }
  }
  return data ;
}

/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
#else
  struct timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000L) ;
  epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec /    1000L) ;
#endif
}


/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int ms)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(ms / 1000) ;
  sleeper.tv_nsec = (long)(ms % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int us)
{
  struct timeval tNow, tLong, tEnd ;

  gettimeofday (&tNow, NULL) ;
  tLong.tv_sec  = us / 1000000 ;
  tLong.tv_usec = us % 1000000 ;
  timeradd (&tNow, &tLong, &tEnd) ;

  while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int us)
{
  struct timespec sleeper ;
  unsigned int uSecs = us % 1000000 ;
  unsigned int wSecs = us / 1000000 ;

  if      (us ==   0)
    return ;
  else if (us  < 100)
    delayMicrosecondsHard (us) ;
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *	Wraps at 49 days.
 *********************************************************************************
 */

unsigned int millis (void)
{
  uint64_t now ;

#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
#endif

  return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *	Wraps after 71 minutes.
 *********************************************************************************
 */

unsigned int micros (void)
{
  uint64_t now ;
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;
#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
#endif


  return (uint32_t)(now - epochMicro) ;
}


unsigned long long piMicros64(void) {
  struct  timespec ts;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  uint64_t now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
  return (now - epochMicro) ;
}

/*
 * wiringPiVersion:
 *	Return our current version number
 *********************************************************************************
 */

void wiringPiVersion (int *major, int *minor)
{
  *major = VERSION_MAJOR ;
  *minor = VERSION_MINOR ;
}

/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
  int   fd ;
  int   model ;

  if (wiringPiSetuped)
    return 0 ;

  wiringPiSetuped = TRUE ;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetup called\n") ;

// Get the board ID information. We're not really using the information here,
//	but it will give us information like the GPIO layout scheme (2 variants
//	on the older 26-pin Pi's) and the GPIO peripheral base address.
//	and if we're running on a compute module, then wiringPi pin numbers
//	don't really mean anything, so force native BCM mode anyway.

  piBoardId (&model) ;

  wiringPiMode = WPI_MODE_PINS ;

	switch (model) {
    case PI_MODEL_HAIBOX_5A:
      pinToGpio =  pinToGpio_HaiBox_5A;
      physToGpio = physToGpio_HaiBox_5A;
      piFunc.setupReg = rk3588_setupReg;
      piFunc.setPinMask = rk3588_setPinMask;
      piFunc.getPinMode = rk3588_getPinMode;
      piFunc.setPinMode = rk3588_setPinMode;
      piFunc.setPullUpDn = rk3588_setPullUpDn;
      piFunc.digitalRead = rk3588_digitalRead;
      piFunc.digitalWrite = rk3588_digitalWrite;
      piFunc.pwmWrite = rk3588_pwmWrite;
      piFunc.pwmClock = rk3588_pwmClock;
      piFunc.pwmRange = rk3588_pwmRange;
      piFunc.setPinMask(model);
      break;
    default:
      wiringPiFailure(WPI_ALMOST, "Oops - unable to determine board type... model: %d\n", model);
      break;
  }

// Open the master /dev/ memory control device
// Device strategy: December 2016:
//	Try /dev/mem. If that fails, then
//	try /dev/gpiomem. If that fails then game over.

  // const char* gpiomemGlobal = gpiomem_global;

  if ((fd = open (gpiomem_global, O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open %s: %s.\n"
	"  Aborting your program because if it can not access the GPIO\n"
	"  hardware then it most certianly won't work\n"
	"  Try running with sudo?\n", gpiomem_global, strerror (errno)) ;
  }
  if (wiringPiDebug) {
    printf ("wiringPi: access to %s succeded %d\n", gpiomem_global, fd) ;
  }

  piFunc.setupReg(fd);

  initialiseEpoch () ;

  return 0 ;
}


/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupGpio called\n") ;

  wiringPiMode = WPI_MODE_GPIO ;

  return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupPhys called\n") ;

  wiringPiMode = WPI_MODE_PHYS ;

  return 0 ;
}

