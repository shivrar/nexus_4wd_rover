#include <Arduino.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include "SoftTimer.h"
#include <Omni4WD.h>
#include "PinChangeInt.h"
#include "PPMReader.h"

#define MAX_LR_VEL 200
#define MAX_FORWARD_VEL 200
#define MAX_ANG_VEL PI/5

float MapCommand(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max){
  return static_cast<float>(x - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
}

// Rover wheel stuff here
/*
	Mecanum4WD
			  Front MOTORS_FB
	wheelUL	\\		// wheelUR


	wheelLL	//		\\ wheelLR
			  Back MOTORS_BF
 */
irqISR(irq1,isr1);
MotorWheel wheelUL(3,2,4,5,&irq1);
irqISR(irq2,isr2);
MotorWheel wheelLL(11,12,14,15,&irq2);
irqISR(irq3,isr3);
MotorWheel wheelLR(9,8,16,17,&irq3);
irqISR(irq4,isr4);
MotorWheel wheelUR(10,7,18,19,&irq4);
Omni4WD Omni(&wheelUL,&wheelLL,&wheelLR,&wheelUR);

uint8_t count = 0;

//Tasks are defined as such
void callBack1(Task* me);
Task t1(1000, callBack1);

void RegulationCallback(Task* me);
Task pid_reg(50, RegulationCallback);

void SpeedCheck(Task* me);
Task speed_check(500, SpeedCheck);

void ParseCommands(Task* me);
Task comm(50, ParseCommands);

void DeadReckon(Task* me);
Task dr(50, DeadReckon);

//void TestReader(Task *me);
//Task ppm(50, TestReader);
//void DemoCallback(Task* me);
//Task demo(1000, DemoCallback);
// the setup function runs once when you press reset or power the board

//void GetCommands();
//uint8_t i;
unsigned int prev_ch[8];

PPMReader *reader= NULL;

void setup() {
  pinMode(13, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.21,0.01,0.005,10);
  reader = new PPMReader(6, 8);


  //TODO: Update diagram with updated stuff
  //TODO: work on speed controllers/motion controller
  //TODO: Update DR code
  //lets use their classes then
  // velocity then angle & angular velocity
//  Omni.setCarMovefl(0, 0, PI/15);
  //Omni.setCarMove(30, (float)-PI/4, 0);
  //Omni.setCarMovefl(30,0,0);
  // let's add a task to the timer
  SoftTimer.add(&dr);
  SoftTimer.add(&t1);
  SoftTimer.add(&pid_reg);
  //SoftTimer.add(&speed_check);
  SoftTimer.add(&comm);

  // do some interrupt magic here
  //PCintPort::attachInterrupt(6, GetCommands, FALLING);
}

void loop()
{
  // After careful consideration it's better to run everything with timer based stuff & interrupts
  SoftTimer.run();
}


void callBack1(Task* me) {
//  float dt = ((float)micros() - (float)me->lastCallTimeMicros)/(float)1000000.0;
//  Serial.print("dt: ");
//  Serial.println(dt,6);
  digitalWrite(13, !digitalRead(13));
}

void RegulationCallback(Task* me){
  Omni.PIDRegulate();
}

void SpeedCheck(Task* me){
  Serial.print("speedRPM> ");
  Serial.println(wheelUL.getSpeedRPM(),DEC);
  Serial.print("MMPS--> "); //display the speed of the MotorWheel
  Serial.println(wheelUL.getSpeedMMPS(),DEC); //display the speed of the motor
}


void ParseCommands(Task* me){
  #ifdef DEBUG
  Serial.print("ch1:");
  Serial.print(reader->rawChannelValue(1));
  Serial.print("\t");
  Serial.print("ch2:");
  Serial.print(reader->rawChannelValue(2));
  Serial.print("\t");
  Serial.print("ch3:");
  Serial.print(reader->rawChannelValue(3));
  Serial.print("\t");
  Serial.print("ch4:");
  Serial.print(reader->rawChannelValue(4));
  Serial.print("\t");
  Serial.print("ch5:");
  Serial.print(reader->rawChannelValue(5));
  Serial.print("\t");
  Serial.print("ch6:");
  Serial.print(reader->rawChannelValue(6));
  Serial.print("\t");
  Serial.print("ch7:");
  Serial.print(reader->rawChannelValue(7));
  Serial.print("\t");
  Serial.print("ch8:");
  Serial.print(reader->rawChannelValue(8));
  Serial.print("\n");
  #endif

  float throttle, yaw;
  float fwd, lr;

  unsigned int curr_ch[8] = {reader->latestValidChannelValue(1,prev_ch[0]),
                             reader->latestValidChannelValue(2,prev_ch[1]),
                             reader->latestValidChannelValue(3,prev_ch[2]),
                             reader->latestValidChannelValue(4,prev_ch[3]),
                             reader->latestValidChannelValue(5,prev_ch[4]),
                             reader->latestValidChannelValue(6,prev_ch[5]),
                             reader->latestValidChannelValue(7,prev_ch[6]),
                             reader->latestValidChannelValue(8,prev_ch[7])};

  throttle = MapCommand(curr_ch[2],800, 2200, 0.0, 1.0);
  if(throttle <= 1.1 && curr_ch[6] > 1400) {
    lr = -throttle * (MapCommand(curr_ch[0], 800, 2200, -1.0, 1.0) * MAX_LR_VEL);
    fwd = -throttle * (MapCommand(curr_ch[1], 800, 2200,-1.0, 1.0) * MAX_FORWARD_VEL);
    yaw = -throttle * (MapCommand(curr_ch[3], 800, 2200, -1.0, 1.0) * MAX_ANG_VEL);
  } else {
    fwd = 0; lr = 0; yaw = 0.0; throttle = 0.0;
  #ifdef DEBUG
    Serial.println("Comms Disconnected!! zero'ing commands!!");
  #endif
  }
#ifdef DEBUG
  Serial.print("throttle:");
  Serial.print(throttle,4);
  Serial.print("\t");
  Serial.print("Left/Right:");
  Serial.print((int)lr);
  Serial.print("\t");
  Serial.print("Forward:");
  Serial.print((int)fwd);
  Serial.print("\t");
  Serial.print("Yaw:");
  Serial.print(yaw);
  Serial.print("\n");
#endif
  //TODO: Implement speed controllers for uf & ul instead of sending in raw
  Omni.setCarMovefl((int)fwd, (int)lr, yaw);
  //instead, iof defaulting to zero let's use the last valid data for the channel
  memcpy(prev_ch, curr_ch, 8 * sizeof (unsigned int));
}

void DeadReckon(Task* me){
  float dt = ((float)me->nowMicros - (float)me->lastCallTimeMicros)/(float)1000000.0;
  Omni.updatePose(dt);
//  Serial.print("MMPS 1--> "); //display the speed of the MotorWheel
//  Serial.print(wheelUL.getSpeedMMPS(),DEC); //display the speed of the motor
//  Serial.print("\t");
//  Serial.print("MMPS 2--> "); //display the speed of the MotorWheel
//  Serial.print(wheelUR.getSpeedMMPS(),DEC); //display the speed of the motor
//  Serial.print("\t");
//  Serial.print("MMPS 3--> "); //display the speed of the MotorWheel
//  Serial.print(wheelLL.getSpeedMMPS(),DEC); //display the speed of the motor
//  Serial.print("\t");
//  Serial.print("MMPS 4--> "); //display the speed of the MotorWheel
//  Serial.print(wheelLR.getSpeedMMPS(),DEC); //display the speed of the motor
//  Serial.print("\n");
//  int vtx = (wheelUL.getSpeedMMPS() - wheelUR.getSpeedMMPS() + wheelLL.getSpeedMMPS() - wheelLR.getSpeedMMPS())/4;
//  int vty = (wheelUL.getSpeedMMPS() + wheelUR.getSpeedMMPS() - wheelLL.getSpeedMMPS() - wheelLR.getSpeedMMPS())/4;
//  float omega = ((-wheelUL.getSpeedMMPS() - wheelUR.getSpeedMMPS() - wheelLL.getSpeedMMPS() - wheelLR.getSpeedMMPS())/(4.0 * WHEELSPAN));
//  Serial.print("vtx: ");
//  Serial.print(vtx, DEC);
//  Serial.print("\t");
//  Serial.print("vty: ");
//  Serial.print(vty, DEC);
//  Serial.print("\t");
//  Serial.print("omega: ");
//  Serial.println(omega, 4);
//  Serial.print("dt:");
//  Serial.print(dt,6);
//  Serial.print("\t");
//  Serial.print("x:");
//  Serial.print(Omni.getPosex(),4);
//  Serial.print("\t");
//  Serial.print("y:");
//  Serial.print(Omni.getPosey(),4);
//  Serial.print("\t");
//  Serial.print("theta:");
//  Serial.print(Omni.getPosetheta(),4);
//  Serial.print("\n");
}

//~~~~~~~~~~~~~~~~ ISR section
//void GetCommands(){
//  //TODO: look here to improve the controller commands
//  a=micros(); //store time value a when pin value falling
//  c=a-b;      //calculating time inbetween two peaks
//  b=a;        //
//  if(c>5000 || i>=8){
//    i = 0;
//    return;
//  }
//  ch[i] = c;
//  i++;
//}

//void TestReader(Task* me){
//  Serial.print("Ch1: ");
//  Serial.print(reader->latestValidChannelValue(1,0));
//  Serial.print("\t");
//  Serial.print("Ch2: ");
//  Serial.print(reader->latestValidChannelValue(2,0));
//  Serial.print("\t");
//  Serial.print("Ch3: ");
//  Serial.print(reader->latestValidChannelValue(3,0));
//  Serial.print("\t");
//  Serial.print("Ch4: ");
//  Serial.print(reader->latestValidChannelValue(4,0));
//  Serial.print("\t");
//  Serial.print("Ch5: ");
//  Serial.print(reader->latestValidChannelValue(5,0));
//  Serial.print("\t");
//  Serial.print("Ch6: ");
//  Serial.print(reader->latestValidChannelValue(6,0));
//  Serial.print("\t");
//  Serial.print("Ch7: ");
//  Serial.print(reader->latestValidChannelValue(7,0));
//  Serial.print("\t");
//  Serial.print("Ch8: ");
//  Serial.print(reader->latestValidChannelValue(8,0));
//  Serial.print("\n");
//}

//void DemoCallback(Task* me){
//  if(count == 0)
//    //Start forward
//    Omni.setCarMovefl(30,0,0);
//  else if(count == 2)
//    Omni.setCarMovefl(0,0,0);
//  else if(count == 3)
//    //Reverse
//    Omni.setCarMovefl(-30,0,0);
//  else if(count == 5)
//    Omni.setCarMovefl(0,0,0);
//  else if(count == 6)
//    //Go left
//    Omni.setCarMovefl(0,30,0);
//  else if(count == 8)
//    Omni.setCarMovefl(0,0,0);
//  else if(count == 9)
//    //Go right
//    Omni.setCarMovefl(0, -30, 0);
//  else if(count == 11)
//    Omni.setCarMovefl(0,0,0);
//  else if(count >=13)
//    count = 0;
//  count++;
//}