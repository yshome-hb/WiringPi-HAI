/*
 * wiringPi.h:
 *	Arduino like Wiring library for the Raspberry Pi.
 *	Copyright (c) 2012-2025 Gordon Henderson
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

#ifndef	__WIRING_PI_H__
#define	__WIRING_PI_H__

// C doesn't have true/false by default and I can never remember which
//	way round they are, so ...
//	(and yes, I know about stdbool.h but I like capitals for these and I'm old)

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(!TRUE)
#endif

// GCC warning suppressor

#define	UNU	__attribute__((unused))

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

// Handy defines

// wiringPi modes

#define	WPI_MODE_PINS		          0
#define	WPI_MODE_GPIO		          1
#define	WPI_MODE_GPIO_SYS	        2  // deprecated since 3.2
#define	WPI_MODE_PHYS		          3
#define	WPI_MODE_PIFACE		        4
#define	WPI_MODE_GPIO_DEVICE_BCM  5  // BCM pin numbers like WPI_MODE_GPIO
#define	WPI_MODE_GPIO_DEVICE_WPI  6  // WiringPi pin numbers like WPI_MODE_PINS
#define	WPI_MODE_GPIO_DEVICE_PHYS 7  // Physic pin numbers like WPI_MODE_PHYS
#define	WPI_MODE_UNINITIALISED   -1

// Pin modes

#define	INPUT			         0
#define	OUTPUT			       1
#define	PWM_OUTPUT		     2
#define	PWM_MS_OUTPUT	     8
#define	PWM_BAL_OUTPUT     9
#define	GPIO_CLOCK		     3
#define	SOFT_PWM_OUTPUT		 4
#define	SOFT_TONE_OUTPUT	 5
#define	PWM_TONE_OUTPUT		 6
#define	PM_OFF		         7   // to input / release line

#define	LOW				0
#define	HIGH			1

// Pull up/down/none

#define	PUD_OFF   0
#define	PUD_DOWN  1
#define	PUD_UP    2

// PWM

#define	PWM_MODE_MS		0
#define	PWM_MODE_BAL	1

// Interrupt levels

#define	INT_EDGE_SETUP		0
#define	INT_EDGE_FALLING	1
#define	INT_EDGE_RISING		2
#define	INT_EDGE_BOTH			3

// Pi model types and version numbers
//	Intended for the GPIO program Use at your own risk.

#define PI_MODEL_UNKOWN		-1

/* Rockchip RK3588(s) */
#define PI_MODEL_HAIBOX_5A	0

#define PI_MODELS_MAX			1

extern const char *piModelNames    [PI_MODELS_MAX] ;


//	Intended for the GPIO program Use at your own risk.

// Threads

#define	PI_THREAD(X)	void *X (UNU void *dummy)

// Failure modes

#define	WPI_FATAL		(1==1)
#define	WPI_ALMOST	(1==2)


// wiringPiNodeStruct:
//	This describes additional device nodes in the extended wiringPi
//	2.0 scheme of things.
//	It's a simple linked list for now, but will hopefully migrate to
//	a binary tree for efficiency reasons - but then again, the chances
//	of more than 1 or 2 devices being added are fairly slim, so who
//	knows....

struct wiringPiNodeStruct
{
  int     pinBase ;
  int     pinMax ;

  int          fd ;	// Node specific
  unsigned int data0 ;	//  ditto
  unsigned int data1 ;	//  ditto
  unsigned int data2 ;	//  ditto
  unsigned int data3 ;	//  ditto

  void (*pinMode)         (struct wiringPiNodeStruct *node, int pin, int mode) ;
  void (*pullUpDnControl) (struct wiringPiNodeStruct *node, int pin, int mode) ;
  int  (*digitalRead)     (struct wiringPiNodeStruct *node, int pin) ;
  void (*digitalWrite)    (struct wiringPiNodeStruct *node, int pin, int value) ;
  void (*pwmWrite)        (struct wiringPiNodeStruct *node, int pin, int value) ;
  int  (*analogRead)      (struct wiringPiNodeStruct *node, int pin) ;
  void (*analogWrite)     (struct wiringPiNodeStruct *node, int pin, int value) ;

  struct wiringPiNodeStruct *next ;
} ;

extern struct wiringPiNodeStruct *wiringPiNodes ;

// Function prototypes
//	c++ wrappers thanks to a comment by Nick Lott
//	(and others on the Raspberry Pi forums)

#ifdef __cplusplus
extern "C" {
#endif

// Data
struct wiringPiFuncStruct
{
  int (*setupReg)(int fd) ;
  int (*setPinMask)(int model) ;
  int (*getPinMode)(int pin) ;
  int (*setPinMode)(int pin, int mode);
  int (*setPinAlt)(int pin, int alt);
  int (*setPullUpDn)(int pin, int pud);
  int (*digitalRead)(int pin) ;
  int (*digitalWrite)(int pin, int value) ;
  int (*pwmWrite)(int pin, unsigned int value) ;
  int (*pwmClock)(int pin, unsigned int divisor) ;
  int (*pwmRange)(int pin, unsigned int range) ;
} ;

// Internal
extern void piGpioLayoutOops (const char *why);
extern int wiringPiFailure (int fatal, const char *message, ...) ;

// Core wiringPi functions

extern struct wiringPiNodeStruct *wiringPiFindNode (int pin) ;
extern struct wiringPiNodeStruct *wiringPiNewNode  (int pinBase, int numPins) ;

extern void wiringPiVersion	(int *major, int *minor) ;
extern int  wiringPiSetup       (void) ;
extern int  wiringPiSetupSys    (void) ;
extern int  wiringPiSetupGpio   (void) ;
extern int  wiringPiSetupPhys   (void) ;

extern void pinModeAlt          (int pin, int alt) ;
extern void pinMode             (int pin, int mode) ;
extern void pullUpDnControl     (int pin, int pud) ;
extern int  digitalRead         (int pin) ;
extern void digitalWrite        (int pin, int value) ;
extern void pwmWrite            (int pin, int value) ;
extern int  analogRead          (int pin) ;
extern void analogWrite         (int pin, int value) ;

// On-Board Raspberry Pi hardware specific stuff

extern          void piBoardId           (int *model) ;
extern          int  wpiPinToGpio        (int wpiPin) ;   // please don't use outside 0-63 and on RP1
extern          int  physPinToGpio       (int physPin) ;  // please don't use outside 0-63 and on RP1
extern          void setPadDrive         (int group, int value) ;
extern          void setPadDrivePin      (int pin, int value);     // Interface V3.0
extern          int  getAlt              (int pin) ;
extern          void pwmToneWrite        (int pin, int freq) ;
extern          void pwmSetMode          (int mode) ;
extern          void pwmSetRange         (int pin, unsigned int range) ;
extern          void pwmSetClock         (int pin, unsigned int divisor) ;
extern unsigned int  digitalReadByte     (void) ;
extern          void digitalWriteByte    (unsigned int value) ;

// Interrupts
// status returned from waitForInterruptV2    V3.16
struct WPIWfiStatus {
  int statusOK;               // -1: error (return of 'poll' command), 0: timeout, 1: irq processed, next data values are valid if needed
  unsigned int pinGPIO;       // gpio as soc pin
  int edge;                   // INT_EDGE_FALLING or INT_EDGE_RISING
  long long int timeStamp_us; // time stamp in microseconds
};

extern int  wiringPiISR         (int pin, int mode, void (*function)(void)) ;
extern struct WPIWfiStatus  waitForInterrupt2(int pin, int edgeMode, int ms, unsigned long debounce_period_us) ;   // V3.16
extern int  wiringPiISR2       (int pin, int edgeMode, void (*function)(struct WPIWfiStatus wfiStatus, void* userdata), unsigned long debounce_period_us, void* userdata) ;  // V3.16
extern int  wiringPiISRStop     (int pin) ;  //V3.2
extern int  waitForInterruptClose(int pin) ; //V3.2 legacy use wiringPiISRStop

// Threads

extern int  piThreadCreate      (void *(*fn)(void *)) ;
extern void piLock              (int key) ;
extern void piUnlock            (int key) ;

// Extras from arduino land

extern void         delay             (unsigned int ms) ;
extern void         delayMicroseconds (unsigned int us) ;
extern unsigned int millis            (void) ;
extern unsigned int micros            (void) ;

extern unsigned long long piMicros64(void);   // Interface V3.7

#ifdef __cplusplus
}
#endif

#endif
