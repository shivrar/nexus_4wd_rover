#include"MotorWheel.h"

#ifndef Omni4WD_H
#define Omni4WD_H


/*
	Mecanum4WD
			  Front MOTORS_FB
	wheelUL	\\		// wheelUR


	wheelLL	//		\\ wheelLR
			  Back MOTORS_BF
 */


/*
	Omni4WD
			  Front MOTORS_FB
	wheelUL	//		\\ wheelUR


	wheelLL	\\		// wheelLR
			  Back MOTORS_BF
 */
#ifndef WHEELSPAN
#define WHEELSPAN 300
#endif
class Omni4WD {
public:
  /*Helper struct to help store 2D pose of the robot*/
  struct Pose2D
  {
    float x;
    float y;
    float theta;

    explicit Pose2D(float x_init=0.0, float y_init=0.0, float theta_init=0.0):x(x_init),
                                                                     y(y_init), theta(theta_init)
    {

    }

  };


	Omni4WD(MotorWheel* wheelUL,MotorWheel* wheelLL,
			MotorWheel* wheelLR,MotorWheel* wheelUR,unsigned int wheelspan=WHEELSPAN);
	unsigned char switchMotors();
	unsigned char switchMotorsReset();

  // time is passed in seconds
  void updatePose(float);
  inline float getPosex() const{
    return pose_.x;
  }
  inline float getPosey() const{
    return pose_.y;
  }
  inline float getPosetheta() const{
    return pose_.theta;
  }

  inline float getFwdVel() const{
    return vels_.x;
  }
  inline float getLatVel() const{
    return vels_.y;
  }
  inline float getAngVel() const{
    return vels_.theta;
  }
	
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);
	unsigned int setCarStop(unsigned int ms=0);

	void setCarMove(int speedMMPS,float rad,float omega=0);
  void setCarMovefl(int speedMMPS_uf,int speedMMPS_ul,float omega=0);
	void setCarAdvance(int speedMMPS=0);
	void setCarBackoff(int speedMMPS=0);
	void setCarLeft(int speedMMPS=0);
	void setCarRight(int speedMMPS=0);

	void setCarRotate(float omega);
	void setCarRotateLeft(int speedMMPS=0);
	void setCarRotateRight(int speedMMPS=0);

	void setCarUpperLeft(int speedMMPS=0);
	void setCarLowerLeft(int speedMMPS=0);
	void setCarUpperRight(int speedMMPS=0);
	void setCarLowerRight(int speedMMPS=0);

	void getCarSpeedRad() const;
	void getCarSpeedMMPS() const;
	void setCarSpeedMMPS(int speedMMPS=0,unsigned int ms=1000);
	void setCarSlow2Stop(unsigned int ms=1000);

	int wheelULGetSpeedMMPS() const;
	unsigned int wheelULSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelULSetSpeedMMPS(int speedMMPS);
	int wheelLLGetSpeedMMPS() const;
	unsigned int wheelLLSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelLLSetSpeedMMPS(int speedMMPS);
	int wheelURGetSpeedMMPS() const;
	unsigned int wheelURSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelURSetSpeedMMPS(int speedMMPS);
	int wheelLRGetSpeedMMPS() const;
	unsigned int wheelLRSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelLRSetSpeedMMPS(int speedMMPS);

	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int interval=1000);
	bool PIDDisable();		// 201209
	bool PIDGetStatus();	// 201209
	float PIDGetP_Param();	// 201210
	float PIDGetI_Param();	// 201210
	float PIDGetD_Param();	// 201210
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false,unsigned char* actBreak = 0);
	void demoActions(unsigned int speedMMPS=100,unsigned int duration=5000,unsigned int uptime=500,bool debug=false);
	void debugger(bool wheelULDebug=true,bool wheelLLDebug=true,
					bool wheelLRDebug=true,bool wheelURDebug=true) const;

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_LEFT,
			STAT_RIGHT,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
			ACTIONTYPES=STAT_UPPERRIGHT,	// 201209
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_FB,	// FrontBack
		MOTORS_BF,	// BackFront
	};
	unsigned char getSwitchMotorsStat() const;
	unsigned int getWheelspan() const;
	
private:
	MotorWheel* _wheelUL;	// UpperLeft
	MotorWheel* _wheelLL;	// LowerLeft
	MotorWheel* _wheelLR;	// LowerRight
	MotorWheel* _wheelUR;	// UpperRight

  Pose2D pose_;
  Pose2D vels_; // can use the same struct for velocity info as well

	unsigned int _wheelspan;	// 201208

	unsigned char _carStat;
	unsigned char setCarStat(unsigned char carStat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);
};

#endif





