#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <avr/eeprom.h>
#include "Arduino.h"
#include "EEPROM.h"

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskOmniDemo( void *pvParameters );


typedef int TaskProfiler;

TaskProfiler BlinkProfilier;
TaskProfiler AnalogProfiler;

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);


const uint16_t* LEDptr = (uint16_t*)(LED_BUILTIN);
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.31,0.01,0,10);

  // Now set up two tasks to run independently.
  xTaskCreate(
      TaskBlink
      ,  "Blink"   // A name just for humans
      ,  100  // This stack size can be checked & adjusted by reading the Stack Highwater
//      ,  const_cast<void *>(reinterpret_cast<const void*>(LEDptr))
      , (void*)LEDptr
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

  xTaskCreate(
      TaskAnalogRead
      ,  "AnalogRead"
      ,  100  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  NULL );


  xTaskCreate(
      TaskOmniDemo
      ,  "OmniDemo"
      ,  100  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  NULL );

}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care
  of use the correct LED pin whatever is the board used.

  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.

  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald

  modified 2 Sep 2016
  by Arturo Guadalupi
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite((uint8_t)pvParameters, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite((uint8_t)pvParameters, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // print out the value you read:
    Serial.println(sensorValue);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskOmniDemo(void *pvParameters)  // This is a task.
{
  (void) pvParameters;


  for (;;)
  {
    Omni.demoActions(30,1500,500,false);
  }
}