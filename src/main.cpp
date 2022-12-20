#include <Arduino.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include "SoftTimer.h"
#include <Omni4WD.h>
#include "simple_pid.h"
#include "SimpleKalmanFilter.h"
#include "SoftTimers.h"
#include "PPMReader.h"
#include "common_items.h"

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
SimplePID<int> uf_c(0.0001,0.25,0.00);
SimplePID<int> ul_c(0.0001,0.25,0.00);
//bool update_ang = false;
SimplePID<float> w_c(0.001,0.25,0.00);

//SimpleKalmanFilter fwd_est(5.0,0.1,0.01);
//SimpleKalmanFilter lr_est(5.0,0.1,0.01);
//SimpleKalmanFilter w_est(5.0,0.1,0.01);

int fwd_demand;
int fwd_curr;
int fwd_out;

int lr_demand;
int lr_curr;
int lr_out;

float w_demand;
float w_curr;
float w_out;

SoftTimer countdown; //millisecond timer
VelCommand ex_cmd;

//uint8_t count = 0;

bool ex_arm = false;
bool first_step = false;
uint8_t last_val = LOW;

//Tasks are defined as such
void StatusCB(Task* me);
Task t1(50, StatusCB);

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

//Setting a default time here since we would never actually use this period
void SerialParser(Task* me);
Task cmd_check(25, SerialParser);

//void GetCommands();
//uint8_t i;
unsigned int prev_ch[8];

PPMReader *reader= nullptr;

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  uf_c.setMax(MAX_FORWARD_VEL);
  ul_c.setMax(MAX_LR_VEL);
  w_c.setMax(MAX_ANG_VEL);

  uf_c.reset();
  ul_c.reset();
  w_c.reset();

  Omni.PIDEnable(0.2,0.01,0.005,10);
  reader = new PPMReader(6, 8);

  //TODO: Update diagram with updated stuff
  //TODO: Add Serial IFC API
  //lets use their classes then

  // let's add a task to the timer
  scheduler.add(&dr);
  scheduler.add(&t1);
  scheduler.add(&wheel_pid_reg);
  //SoftTimer.add(&speed_check);
  scheduler.add(&comm);
  scheduler.add(&speed_pid_reg);
  //SoftTimer.add(&demo);

  // do some interrupt magic here
  //PCintPort::attachInterrupt(6, GetCommands, FALLING);
}

void loop()
{
  // After careful consideration it's better to run everything with timer based stuff & interrupts
  scheduler.run();
}


void StatusCB(Task* me) {
// can use this one as an indicator if armed or not
//  float dt = ((float)micros() - (float)me->lastCallTimeMicros)/(float)1000000.0;
//  Serial.print("dt: ");
//  Serial.println(dt,6);
//  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  digitalWrite(STATUS_LED, uint8_t(ex_arm));
}

void WheelRegulationCallback(Task* me){
  Omni.PIDRegulate();
}

void SerialParser(Task* me){
  //Set the time for this to tick over when the time based on the commnd
}

void SpeedRegulationCallback(Task* me){
  // TODO: this block will either combine/ reject / figureout what to do with the various speed demands from the rx or
  //  automated commands from the companion computer
  /**
   * Magic happens here
   *
   * basically before we send the commanda to the controllers we need to pick which one we would use.
   *
   * So before use the commands vels & timeout from either the external comms or the RX we wpould need to do several
   * things. ::
   *
   * Still being tbdddddddd
   *
   *
   * */


  fwd_out = uf_c.update(fwd_demand, fwd_curr);
  lr_out = ul_c.update(lr_demand, lr_curr);
  w_out = w_c.update(w_demand, w_curr);
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

  uint8_t curr_state = curr_ch[4] >= 1400 ? HIGH : LOW;

  // if change detected then we do something
  if(curr_state != last_val) {
    if(curr_state == HIGH)
      first_step = true;
    else if(first_step){
      ex_arm = !ex_arm;
      ex_cmd = VelCommand(1.0,0.0,0.0, 1000);
    }
    else {
      ex_arm = false;
      first_step = false;
    }
  }

  if(!ex_arm) {
    throttle = MapCommand(curr_ch[2], 800, 2200, 0.0, 1.0);
    if (throttle <= 1.1 && curr_ch[6] > 1400) {
      lr = -throttle * (MapCommand(curr_ch[0], 800, 2200, -1.0, 1.0) * MAX_LR_VEL);
      fwd = -throttle * (MapCommand(curr_ch[1], 800, 2200, -1.0, 1.0) * MAX_FORWARD_VEL);
      yaw = -throttle * (MapCommand(curr_ch[3], 800, 2200, -1.0, 1.0) * MAX_ANG_VEL);
      if (fabs(lr) < 0.1 * MAX_LR_VEL)
        lr = 0.0;
      if (fabs(fwd) < 0.1 * MAX_FORWARD_VEL)
        fwd = 0.0;
      if (fabs(yaw) < 0.05 * MAX_ANG_VEL)
        yaw = 0.0;
    } else {
      fwd = 0;
      lr = 0;
      yaw = 0.0;
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
  } else {
    //TODO: continue from here
    // This branch of the logic should do the following:
    // - Get the command from the serial and start the countdown timer, if the command is null do nothing.
//    if(ex_cmd.time == 0.0)
//    {
//      fwd = 0;
//      lr = 0;
//      yaw = 0.0;
//    } else {
//      fwd = 0.5;
//      lr = 0;
//      yaw = 0.0;
//    }
    if(countdown.hasTimedOut()){

    }
  }
  last_val = curr_state;

  fwd_demand = int(fwd);
  lr_demand = int(lr);
  w_demand = yaw;
  //instead, iof defaulting to zero let's use the last valid data for the channel
  memcpy(prev_ch, curr_ch, 8 * sizeof (unsigned int));
  // if ex_arm_is decided we can probably start the task for external commands
}

void DeadReckon(Task* me){
  float dt = ((float)me->nowMicros - (float)me->lastCallTimeMicros)/(float)1000000.0;
  Omni.updatePose(dt);
  // with the update also update the speeds for the controllers
  fwd_curr = (int)Omni.getFwdVel();
  lr_curr = (int)Omni.getLatVel();
  w_curr = Omni.getAngVel();
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