#ifndef _SIMPLE_PID_H
#define _SIMPLE_PID_H
#include "Arduino.h"

/*Here, the definition of the PID class begins. This is indicated by the keyword: "class"
This is a general description of the data and functions that the class contains. 
To use a class, we must make a specific instance of the class by declaring it into the same way we declare a variable. 
For example, to create a version of the PID class, in our main file we might write:

PID LeftWheelPID;
PID RightWheelPID;

This will create two instances of the PID class; one for the left wheel and one for the right wheel. 
Each class will have a full copy of all the variables and functions defined for that particular class.
*/
template<typename T>
class SimplePID
{
  /* Public functions and variables are defined here. A public function / variable can be accessed from outside 
   * the class. 
   * For example, once we have made an instance of the PID class, we can call the update function by writing:
   * 
   * LeftWheelPID.update();
   * 
   * Note that this will only update the LeftWheelPID - RightWheelPID will not be updated unless we also call 
   * RightWheelPID.update()
   */
public:

  SimplePID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class
  void setGains(float P, float I, float D );      // This function updates the values of the gains
  void reset();                                   // This function resets any stored values used by the integral or derative terms
  T update(T demand, T measurement);  // This function calculates the PID control signal. It should be called in a loop
  void setMax(T  newMax);                     // This function sets the maximum output the controller can ask for

  /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
   * For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!) 
   * You might want to move some into public space when you are debugging ;)
   */
private:

  //Control gains
  float Kp; //Proportional
  float Ki; //Integral
  float Kd; //Derivative

  //We can use this to limit the output to a certain value
  T max_output;

  //Output components
  //These are used for debugging purposes
  float Kp_output;
  float Ki_output;
  float Kd_output;
  T output_signal;

  //Values to store between updates().
  //T last_demand;      //For storing the previous input
  //T last_measurement; //For storing the last measurement
  T last_error;       //For calculating the derivative term
  T integral_error;   //For storing the integral of the error
  unsigned long last_millis;       //To track elapsed_time
  //bool debug;             //This flag controls whether we print the contributions of each component when update is called

};

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
template<typename T>
SimplePID<T>::SimplePID
(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);

  // Initialise key variables.
  Kp_output     = 0;
  Ki_output     = 0;
  Kd_output     = 0;
  output_signal = 0;

  max_output        = 255;
  //last_demand       = 0;
  //last_measurement  = 0;
  last_error        = 0;
  integral_error    = 0;
  //debug             = false;
  last_millis       = millis();

}

/*
 * This function sets the gains of the PID controller
 */
template<typename T> void
SimplePID<T>::
setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

/*
 * This is the update function. 
 * This function should be called repeatedly. 
 * It takes a measurement of a particular variable (ex. Position, speed, heading) and a desired value for that quantity as input
 * It returns an output; this can be sent directly to the motors, 
 * combined with other control outputs
 * or sent as input to another controller
 */
template<typename T> T
SimplePID<T>::
update(T demand, T measurement) {
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  /*
   * ================================
   * Your code goes implementation of a PID controller should go here
   * ================================
   */

  //This represents the error term
  // Decide what your error signal is (demand vs measurement)
  T error;
  error = demand - measurement;

  //This represents the error derivative
  // Calculate the change in your error between update()
  float error_delta;
  error_delta = 1000.0*(error-last_error)/(float)time_delta;

  // This represents the error integral.
  // Integrate error over time.
  integral_error += error;

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output = Ki * integral_error;
  Kd_output = Kd * error_delta;

  // Add the three components to get the total output
  // Note: Check the sign of your d gain.  Check that the
  // Kd_output contribuition is the opposite of any 
  // overshoot you see using the Serial Plotter
  output_signal = Kp_output + Ki_output - Kd_output;

  /*
   * ===========================
   * Code below this point should not need to be changed
   * But of course, feel free to improve / experiment :)
   */


  //Update persistent variables.
  //last_demand = demand;
  //last_measurement = measurement;
  last_error = error;

  // Catching max in positive sign.
  //if (total > max_output) {
  //  total = max_output;
  //} 
  // Catching max in negative sign
  //if (total < -max_output) {
  //  total = -max_output;
  //}

  return max(min(output_signal, max_output), -max_output);
}

template<typename T> void
SimplePID<T>::
setMax(T newMax)
{
  if (newMax > 0) {
    max_output = newMax;
  }
  else {
    //Serial.println("Max output must be positive");
    max_output = -newMax;
  }
}

template<typename T> void
SimplePID<T>::
reset() {

  last_error = 0;
  integral_error = 0;
  last_millis = millis();

}

#endif