// WiringPi test program: Kernel char device interface / sysfs successor
// Compile: gcc -Wall wiringpi_test62_isr_wpin.c -o wiringpi_test62_isr_wpin -lwiringPi

#include "wpi_test.h"
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>

//WPI pin numbers
int GPIO = 28;
int GPIOIN = 29;

static volatile int globalCounter;


static void wfiup(void) { 

  globalCounter++;
  printf("I[%d] ", globalCounter);
  fflush(stdout);
}

static void wfidown(void) { 

  globalCounter--;
  printf("I[%d] ", globalCounter);
  fflush(stdout);
}


void StartSequence (int Enge, int OUTpin, int count,int expected) {
  /*
  int expected = up ? count : 0;
  globalCounter = up ? 0 : count;
  */
  globalCounter = 0;

  printf("Start: ");
  fflush(stdout);
  for (int loop=0; loop<count; ++loop) {
    printf("H");
    digitalWrite(OUTpin, HIGH); 
    printf("."); fflush(stdout);
    delay(200);
    printf("L");
    digitalWrite(OUTpin, LOW);
    delay(100); 
    printf("."); fflush(stdout);
  }
  printf("\n");

  CheckSame("IRQ count", globalCounter, expected);
}



int main (void) {

	int major, minor;

	wiringPiVersion(&major, &minor);

	printf("WiringPi GPIO test program 6.2 (using GPIO%d (output) and GPIO%d (input))\n", GPIO, GPIOIN);
	printf(" testing irq with WPI pin numbers > 28\n");

	printf("\nWiringPi %d.%d\n", major, minor);

  //printf("Error check - next call create fatal error with exit!\n");
  //CheckSame("wiringPiISRStop with wrong pin, result code:", wiringPiISRStop(5), 0);

  CheckSame("wiringPiSetupPinType(WPI_PIN_WPI)", wiringPiSetupPinType(WPI_PIN_WPI), 0);

  int rev, mem, maker, overVolted, RaspberryPiModel;
  piBoardId(&RaspberryPiModel, &rev, &mem, &maker, &overVolted);

  CheckNotSame("piBoardId", RaspberryPiModel, 0);

  int IRQpin = GPIOIN;
  int OUTpin = GPIO;
  
  if (RaspberryPiModel==PI_MODEL_4B) {
    pinMode(IRQpin, INPUT);
    pinMode(OUTpin, OUTPUT);
    digitalWrite (OUTpin, LOW) ;
  
    printf("\nTesting IRQ @ WPI-GPIO%d with trigger @ WPI-GPIO%d rising\n", IRQpin, OUTpin);
    CheckSame("wiringPiISR",  wiringPiISR(IRQpin, INT_EDGE_RISING, &wfiup), 0);
    sleep(1);
    StartSequence(INT_EDGE_RISING, OUTpin, 3 , 3);
    sleep(1);
    CheckSame("wiringPiISRStop", wiringPiISRStop(IRQpin), 0);
    printf("\n.IRQ off\n");
    sleep(1);
    StartSequence(INT_EDGE_RISING, OUTpin, 2, 0);

    printf("\nTesting IRQ @ WPI-GPIO%d with trigger @ WPI-GPIO%d falling\n", IRQpin, OUTpin);
    CheckSame("wiringPiISR",  wiringPiISR(IRQpin, INT_EDGE_RISING, &wfidown), 0);
    sleep(1);
    StartSequence(INT_EDGE_FALLING, OUTpin, 4, -4);
    sleep(1);
    CheckSame("wiringPiISRStop", wiringPiISRStop(IRQpin), 0);
    printf("\n.IRQ off\n");
    sleep(1);
    StartSequence(INT_EDGE_RISING, OUTpin, 2, 0);

    printf("Error check - next call must be wrong!\n");
    CheckSame("wiringPiISRStop with wrong pin, result code:", wiringPiISRStop(5555), EINVAL);

    pinMode(OUTpin, INPUT);
  }

	return UnitTestState();
}
