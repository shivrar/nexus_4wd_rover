#include <Arduino.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include "SoftTimer.h"
#include <MotorWheel.h>
#include <Omni4WD.h>

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);

void callBack1(Task* me);

Task t1(1000, callBack1);

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(13, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.31,0.01,0,10);

  // let's add a task to the timer
  SoftTimer.add(&t1);

}

void loop()
{
  //TODO: Repeat the demo in a non blocking way.
  // Using the wheel def and what the actions do etc
  //Omni.demoActions(30,1500,500,false);
  // After careful consideration it's better to run everything with timer based stuff & interrupts
  SoftTimer.run();
}


void callBack1(Task* me) {
  digitalWrite(13, !digitalRead(13));
}