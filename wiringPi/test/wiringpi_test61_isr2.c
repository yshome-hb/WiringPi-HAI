// WiringPi test program: 3.16 new ISR2 function with Kernel char device interface / sysfs successor 
// Compile: gcc -Wall wiringpi_test1_device.c -o wiringpi_test1_device -lwiringPi

#include "wpi_test.h"
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>


int GPIO = 19;
int GPIOIN = 26;
const int ToggleValue = 4;
float irq_timstamp_duration_ms = 0;
float accuracy = 0.01;

static volatile int globalCounter;
volatile long long gStartTime, gEndTime;
struct WPIWfiStatus wfiStatusOld;

static void ISR(void) { 
  struct timeval now;

  gettimeofday(&now, 0);
  if (0==gStartTime) {
    gStartTime = now.tv_sec*1000000LL + now.tv_usec;
  } else {
    gEndTime = now.tv_sec*1000000LL + now.tv_usec;
  }
  globalCounter++;
}


static void ISR2(struct WPIWfiStatus wfiStatus) { 
  struct timeval now;
  char strEdge[3][10] = {"unknown", "falling", "raising"};
  int strEdgeidx = 0; //default

  gettimeofday(&now, 0);
  if (0==gStartTime) {
    gStartTime = now.tv_sec*1000000LL + now.tv_usec;
  } else {
    gEndTime = now.tv_sec*1000000LL + now.tv_usec;
  }
  globalCounter++;
  switch(wfiStatus.edge) {
    case INT_EDGE_FALLING: strEdgeidx = INT_EDGE_FALLING; break;
    case INT_EDGE_RISING:  strEdgeidx = INT_EDGE_RISING;  break;
  }
  if (wfiStatusOld.statusOK>0 && wfiStatusOld.pinBCM==wfiStatus.pinBCM) {
    irq_timstamp_duration_ms = (wfiStatus.timeStamp_us - wfiStatusOld.timeStamp_us) / 1000.0f;
  } else {
    irq_timstamp_duration_ms = 0.0f;
  }
  printf("ISR occured @ %lld us: statusOK=%d, pin=%u, edge=%s (%d), duration=%g ms\n", 
    wfiStatus.timeStamp_us, wfiStatus.statusOK, wfiStatus.pinBCM, strEdge[strEdgeidx], wfiStatus.edge, irq_timstamp_duration_ms);
  wfiStatusOld = wfiStatus;
}


void digitalWriteBounce(int OUTpin, int value, int bounce) {
  digitalWrite(OUTpin, value);
  if (bounce>0) {
    delay(1);
    digitalWrite(OUTpin, !value);
    delay(1);
    digitalWrite(OUTpin, value);
  }
}


void DelayAndSumDuration(int ms, float* irq_timstamp_sum, const int doSum) {
  delay(20);
  if (doSum) {
    *irq_timstamp_sum += irq_timstamp_duration_ms;
  }
  delay(ms-20);
}


double StartSequence2(int Edge, int OUTpin, int INpin, int bounce) {
  int expected;
  float timeExpected_ms;
  float irq_timstamp_sum = 0.0f;

  gStartTime = 0;
  gEndTime = 0;
  globalCounter = 0;
  printf("Start\n");
  digitalWriteBounce(OUTpin, HIGH, bounce);
  delay(200);

  digitalWriteBounce(OUTpin, LOW, bounce);
  DelayAndSumDuration(100, &irq_timstamp_sum, INT_EDGE_BOTH == Edge);

  digitalWriteBounce(OUTpin, HIGH, bounce);
  DelayAndSumDuration(200, &irq_timstamp_sum, INT_EDGE_RISING == Edge || INT_EDGE_BOTH == Edge);

  digitalWriteBounce(OUTpin, LOW, bounce);
  DelayAndSumDuration(100, &irq_timstamp_sum, INT_EDGE_FALLING == Edge || INT_EDGE_BOTH == Edge);

  printf("Stop\n");
  int globalCounterCopy = globalCounter; 

  if (INT_EDGE_BOTH == Edge) {
    expected = 4;  
    timeExpected_ms = bounce ? 508: 500;
  } else {
    expected = 2;  
    timeExpected_ms = bounce ? 304: 300;
  }

  if (globalCounter==expected) {
    char str[1024];
    float fTime = (gEndTime - gStartTime) / 1000.0;
    sprintf(str, "IRQ measured  %g msec (~%g expected)", fTime, timeExpected_ms);
    CheckSameFloat(str, fTime, timeExpected_ms, timeExpected_ms*accuracy);
    sprintf(str, "IRQ timestamp %g msec (~%g expected)", irq_timstamp_sum, timeExpected_ms);
    CheckSameFloat(str, irq_timstamp_sum, timeExpected_ms, timeExpected_ms*accuracy);
    // new data struct
    CheckSame("GPIO IRQ pin", wfiStatusOld.pinBCM, INpin);
    if (INT_EDGE_FALLING==Edge || INT_EDGE_RISING==Edge) {
      CheckSame("GPIO IRQ edge", wfiStatusOld.edge, Edge);
    }
    CheckSame("GPIO IRQ status", wfiStatusOld.statusOK, 1);
    return fTime;
  } else {
    printf("IRQ not worked got %d iterations (%d exprected)\n\n", globalCounterCopy, expected);
    return 0;
  }
}


double DurationTime(int Enge, int OUTpin, int IRQpin, int new) {
  struct timeval now;
  double fTime = 0.0;

  gStartTime = 0;
  gEndTime = 0;
  globalCounter = 0;
  printf("Start\n");

  if (INT_EDGE_RISING == Enge) {
    digitalWrite(OUTpin, LOW);
    if (new) { 
      wiringPiISR2(IRQpin, INT_EDGE_RISING, &ISR2, 0);
    } else {
      wiringPiISR(IRQpin, INT_EDGE_RISING, &ISR);
    }
    sleep(1);
    gettimeofday(&now, 0);
    gStartTime = now.tv_sec*1000000LL + now.tv_usec;
    digitalWrite(OUTpin, HIGH);  
    delay(20);
    digitalWrite(OUTpin, LOW);  
   } else if (INT_EDGE_FALLING == Enge) {
    digitalWrite(OUTpin, HIGH); 
    if (new) {
      wiringPiISR2(IRQpin, INT_EDGE_FALLING, &ISR2, 0);
    } else {
      wiringPiISR(IRQpin, INT_EDGE_FALLING, &ISR);
    }
    sleep(1);
    gettimeofday(&now, 0);
    gStartTime = now.tv_sec*1000000LL + now.tv_usec;    
    digitalWrite(OUTpin, LOW);  
  }

  sleep(1);
  fTime = (gEndTime - gStartTime);
  printf("IRQ detect time %g usec", fTime);
  if (fTime<2000 && fTime>0) {
    printf("                          -> %spassed%s\n", COLORGRN, COLORDEF);
  } else {
    printf("                          -> %sfailed%s\n", COLORRED, COLORDEF);
  }
  wiringPiISRStop(IRQpin);

  return fTime;
}


int main (void) {

	int major=0, minor=0;

	wiringPiVersion(&major, &minor);

	printf("WiringPi GPIO test program 6b (using GPIO%d (output) and GPIO%d (input))\n", GPIO, GPIOIN);
	printf("ISR and ISR2 test (WiringPi %d.%d)\n", major, minor);

	wiringPiSetupGpio();

  int RaspberryPiModel, rev, mem, maker, overVolted;
  piBoardId(&RaspberryPiModel, &rev, &mem, &maker, &overVolted);
  CheckNotSame("Model: ", RaspberryPiModel, -1);
  switch(RaspberryPiModel) {
    case PI_MODEL_A:
    case PI_MODEL_B:     //ARM=800MHz
    case PI_MODEL_BP:
    case PI_MODEL_AP:
    case PI_MODEL_CM:
      accuracy = 0.02;
      break;
    default:
      accuracy = 0.01;
      break;
  }
	if (!piBoard40Pin()) {
		GPIO = 23;
		GPIOIN = 24;
	}
	int IRQpin = GPIOIN;
	int OUTpin = GPIO;

  //wiringPiISR2(13, INT_EDGE_RISING, &ISR2, 0); // next pin
	
  memset(&wfiStatusOld, 0, sizeof(wfiStatusOld));

	pinMode(IRQpin, INPUT);
	pinMode(OUTpin, OUTPUT);
	digitalWrite (OUTpin, LOW) ;

  for (int bounce=0; bounce<=1; bounce++) {
    unsigned long bouncetime = 0;
    if (0==bounce) {
      printf("! --- default test (no bounce) --- !\n");
    } else {
      bouncetime = 2000; //2 ms
      printf("! --- test with bounce on input --- !\n");
    }

    printf("\nTesting IRQ @ GPIO%d with trigger @ GPIO%d rising\n", IRQpin, OUTpin);
    wiringPiISR2(IRQpin, INT_EDGE_RISING, &ISR2, bouncetime);
    sleep(1);
    StartSequence2(INT_EDGE_RISING, OUTpin, IRQpin, bounce);
    printf("Stopp IRQ\n");
    wiringPiISRStop(IRQpin);

    printf("\nTesting IRQ @ GPIO%d with trigger @ GPIO%d falling\n", IRQpin, OUTpin);
    wiringPiISR2(IRQpin, INT_EDGE_FALLING, &ISR2, bouncetime);
    sleep(1);
    StartSequence2(INT_EDGE_FALLING, OUTpin, IRQpin, bounce);
    printf("Stopp IRQ\n");
    wiringPiISRStop(IRQpin);

    printf("\nTesting IRQ @ GPIO%d with trigger @ GPIO%d both\n", IRQpin, OUTpin);
    wiringPiISR2(IRQpin, INT_EDGE_BOTH, &ISR2, bouncetime);
    sleep(1);
    StartSequence2(INT_EDGE_BOTH, OUTpin, IRQpin, bounce);
    printf("Stopp IRQ\n");
    wiringPiISRStop(IRQpin);
  }

	for (int count=0; count<2; count++) {
	  printf("\nclassic function:\n");
    printf("=================\n");
    printf("Measuring duration IRQ @ GPIO%d with trigger @ GPIO%d rising\n", IRQpin, OUTpin);
	  DurationTime(INT_EDGE_RISING, OUTpin, IRQpin, 0);
	  printf("Measuring duration IRQ @ GPIO%d with trigger @ GPIO%d falling\n", IRQpin, OUTpin);
	  DurationTime(INT_EDGE_FALLING, OUTpin, IRQpin, 0);

    printf("\nnew function:\n");
    printf("===============\n");
    printf("Measuring duration IRQ @ GPIO%d with trigger @ GPIO%d rising\n", IRQpin, OUTpin);
	  DurationTime(INT_EDGE_RISING, OUTpin, IRQpin, 1);
	  printf("Measuring duration IRQ @ GPIO%d with trigger @ GPIO%d falling\n", IRQpin, OUTpin);
	  DurationTime(INT_EDGE_FALLING, OUTpin, IRQpin, 1);
	}
	pinMode(OUTpin, INPUT);

	return UnitTestState();
}
