// encoder.c
// Test code for incremental rotary encoder on Raspberry Pi 
//tested on B R2 Linux raspberrypi 3.10.25+ #622 PREEMPT Fri Jan 3 18:41:00 GMT 2014 armv6l GNU/Linux

// http://www.sparkfun.com/products/10982
// No licence, no warranty, free to use for any purpose
// Credit to joan for pigpio http://abyz.co.uk/rpi/pigpio/
// Build with gcc -o encoder encoder.c -lpigpio -lpthread -lrt
// Run with sudo ./encoder
// Turn shaft to vary RGB value
// Press shaft to cycle R-G-B control

#include <stdio.h>
#include <pigpio.h>

/*
   Rotary encoder connections:

   http://www.hobbytronics.co.uk/rotary-encoder-rgb

   Encoder A      - gpio 24
   Encoder B      - gpio 23
   Encoder Common - Pi ground
   Red LED        - gpio 22
   Green LED      - gpio 27
   Blue  LED      - gpio 17
   Button         - gpio 25
   Buttom common  and LED + to 3v3 via 100R

   Turn to adjust value and R,G or B LED level (bu PWM)
   Press to cycle adjust R, G, B

   Bugs:        Button press does not manage PWM value update correctly

*/

#define ENCODER_A 24
#define ENCODER_B 23
#define BUTTON 25
#define RED 22
#define GREEN 27
#define BLUE 17

static volatile int encoderPos;
static volatile int colourRed;
static volatile int colourGreen;
static volatile int colourBlue;
static volatile int adjustMode;

/* forward declaration */

void encoderPulse(int gpio, int lev, uint32_t tick);
void buttonPress(int gpio, int lev, uint32_t tick);

int main(int argc, char * argv[])
{
   int pos=0;

   colourRed = 80;
   colourGreen = 180;
   colourBlue = 190;
   adjustMode = 0;
   pos = colourRed;
   encoderPos = pos;

   if (gpioInitialise()<0) return 1;

   gpioSetMode(ENCODER_A, PI_INPUT);
   gpioSetMode(ENCODER_B, PI_INPUT);
   gpioSetMode(BUTTON, PI_INPUT);

   gpioSetMode(RED, PI_OUTPUT);
   gpioSetMode(GREEN, PI_OUTPUT);
   gpioSetMode(BLUE, PI_OUTPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(ENCODER_A, PI_PUD_UP);
   gpioSetPullUpDown(ENCODER_B, PI_PUD_UP);
   gpioSetPullUpDown(BUTTON, PI_PUD_DOWN);

  gpioPWM(RED,255-colourRed);
  gpioPWM(GREEN,155-colourGreen);
  gpioPWM(BLUE,155-colourBlue);


   /* monitor encoder level changes */

   gpioSetAlertFunc(ENCODER_A, encoderPulse);
   gpioSetAlertFunc(ENCODER_B, encoderPulse);
   gpioSetAlertFunc(BUTTON, buttonPress);

   while (1)
   {
      if (pos != encoderPos)
      {
         pos = encoderPos;
         switch(adjustMode){
        case 0:
                colourRed = pos;
                break;
        case 1:
                colourGreen = pos;
                break;
        case 2:
                colourBlue = pos;
                break;
        default:
                adjustMode=0;
        }
         printf("pos[%c]=%d\n",(adjustMode==0)?'R':(adjustMode==1)?'G':'B', pos);
         gpioPWM(RED,255-colourRed);
         gpioPWM(GREEN,255-colourGreen);
         gpioPWM(BLUE,255-colourBlue);
      }
      gpioDelay(20000); /* check pos 50 times per second */
   }

   gpioTerminate();
}

void buttonPress(int gpio, int level, uint32_t tick)
{
        adjustMode++;

         switch(adjustMode){
        case 0:
                encoderPos = colourRed ;
                break;
        case 1:
                encoderPos = colourGreen;
                break;
        case 2:
                encoderPos = colourBlue;
                break;
        default:
                adjustMode=0;
        }
}


void encoderPulse(int gpio, int level, uint32_t tick)
{
   /*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

level will be 1 for rising edge
 count up on rising edge of A if B is high
 or down if B is low
 Falling edge is reverse

 Same logic but reverse of transitions on B
 count down on rising B if A is high
 or up is A is low
 Falling edge is reverse

   */
   int inc = (level!=0)?1:-1;
   int other = 0;

   if (gpio == ENCODER_A){
        other = gpioRead(ENCODER_B);
   }
   else if (gpio == ENCODER_B){
        other = gpioRead(ENCODER_A);
        inc = -inc;
   }

   if(other>0)
        encoderPos += inc;
  if(other>0)
        encoderPos += inc;
   else
        encoderPos -= inc;

   if(encoderPos<0) encoderPos = 0;
   if(encoderPos>255) encoderPos = 255;
}