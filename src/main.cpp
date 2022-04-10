#include <Arduino.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include "SoftTimer.h"
#include <Omni4WD.h>
//#include "simple_pid.h"
#include "PPMReader.h"

#define MAX_LR_VEL 200
#define MAX_FORWARD_VEL 200
#define MAX_ANG_VEL PI/4

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

//PID *uf_c;
//SimplePID<int> uf_c(0.0001,0.31,0.00);
//SimplePID<int> ul_c(0.0001,0.31,0.00);
bool update_ang = false;
//SimplePID<float> w_c(0.00018,0.31,0.00);

int fwd_demand;
//int fwd_curr;
int fwd_out;

int lr_demand;
//int lr_curr;
int lr_out;

float w_demand;
//float w_curr;
float w_out;

uint8_t count = 0;

//Tasks are defined as such
void callBack1(Task* me);
Task t1(1000, callBack1);

void WheelRegulationCallback(Task* me);
Task wheel_pid_reg(50, WheelRegulationCallback);

void ParseCommands(Task* me);
Task comm(50, ParseCommands);

void DeadReckon(Task* me);
Task dr(50, DeadReckon);

void SpeedRegulationCallback(Task* me);
Task speed_pid_reg(100, SpeedRegulationCallback);
//void DemoCallback(Task* me);
//Task demo(1000, DemoCallback);
// the setup function runs once when you press reset or power the board

//void GetCommands();
//uint8_t i;
unsigned int prev_ch[8];

PPMReader *reader= nullptr;

void setup() {
  pinMode(13, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.21,0.01,0.005,10);
  reader = new PPMReader(6, 8);

//  uf_c = new PID(&fwd_curr, &fwd_out, &fwd_demand, 0.0,0.0,0.0);
//  uf_c->SetTunings(0.5, 0.0, 0.00);
//  uf_c->SetInputLimits(0, MAX_FORWARD_VEL);
//  uf_c->SetOutputLimits(0, MAX_FORWARD_VEL);
//  uf_c->SetSampleTime(200);
//  uf_c->SetMode(AUTO);
  
  //TODO: Update diagram with updated stuff
  //TODO: Add Serial IFC API
  //lets use their classes then
  // velocity then angle & angular velocity
//  Omni.setCarMovefl(0, 0, PI/15);
  //Omni.setCarMove(30, (float)-PI/4, 0);
  //Omni.setCarMovefl(30,0,0);

  // let's add a task to the timer
  SoftTimer.add(&dr);
  SoftTimer.add(&t1);
  SoftTimer.add(&wheel_pid_reg);
  //SoftTimer.add(&speed_check);
  SoftTimer.add(&comm);
  SoftTimer.add(&speed_pid_reg);
  //SoftTimer.add(&demo);

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

void WheelRegulationCallback(Task* me){
  Omni.PIDRegulate();
}

void SpeedRegulationCallback(Task* me){
  fwd_out = fwd_demand;
  lr_out = lr_demand;
  //update the angular velocity every other spin
  if(update_ang){
    w_out = w_demand;
    update_ang = false;
  } else
    update_ang = true;
  //There's no SS error and only a slight overshoot, so no speed controllers actually needed
  Omni.setCarMovefl(fwd_out, lr_out, w_out);
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
  fwd_demand = int(fwd);
  lr_demand = int(lr);
  w_demand = yaw;
  //instead, iof defaulting to zero let's use the last valid data for the channel
  memcpy(prev_ch, curr_ch, 8 * sizeof (unsigned int));
}

void DeadReckon(Task* me){
  float dt = ((float)me->nowMicros - (float)me->lastCallTimeMicros)/(float)1000000.0;
  Omni.updatePose(dt);
  // with the update also update the speeds for the controllers
//  fwd_curr = (int)Omni.getFwdVel();
//  lr_curr = (int)Omni.getLatVel();
//  w_curr = Omni.getAngVel();
  #ifdef DEBUG
  Serial.print("x:");
  Serial.print(Omni.getPosex(),4);
  Serial.print("\t");
  Serial.print("y:");
  Serial.print(Omni.getPosey(),4);
  Serial.print("\t");
  Serial.print("theta:");
  Serial.print(Omni.getPosetheta(),4);
  Serial.print("\n");
  #endif
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
//  if(count == 0){
//    fwd_out = 50;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 2){
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 3){
//    fwd_out = -50;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 5){
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 6){
//    fwd_out = 0;
//    lr_out = 50;
//    w_out = 0;
//  }
//  else if(count == 8){
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 9){
//    fwd_out = 0;
//    lr_out = -50;
//    w_out = 0;
//  }
//  else if(count == 11){
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 12){
//    //rotate left
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = MAX_ANG_VEL/2;
//  }
//  else if(count == 14){
//    //rotate left
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = 0;
//  }
//  else if(count == 15){
//    //rotate left
//    fwd_out = 0;
//    lr_out = 0;
//    w_out = -MAX_ANG_VEL/2;
//  }
//  else if(count >=17)
//    count = 0;
//  count++;
//}