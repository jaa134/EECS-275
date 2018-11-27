#include "minimal_turtlebot/turtlebot_controller.h"
#include "turtlebot_controller.hpp"
#include "iostream"
#include "string"
#include "sstream"

/*
  ********************
  STAGE 1 DELIVERABLESlinearAccelY
  ********************
  If one of the bumpers is pressed, the robot should back away, turn, and continue on.
  Similar behavior should be implemented for the cliff sensors.
  The robot should stop moving if the wheel-drop sensors are triggered and announce an error.
  If the vector for linear acceleration passes 20 degrees, the motors should stop and announce error.
  The robot must stop if any obstacle becomes closer than a half a meter (0.5 m). It should not immediately move
  away from the obstacle, but announce its presence and wait fifteen (15) seconds for the obstacle move
  away before spinning in-place to find a direction that is not blocked and begin moving again
*/
bool handleCliffSensors(long, bool, bool, bool, float*, float*);
bool handleBumpers(long, bool, bool, bool, float*, float*);
bool handleSenseObstacle(long, float[], uint8_t*, float*, float* );
bool handleWheelDrop(bool, bool, uint8_t*, float*, float*);

void debug(std::string message) {
    std::cout << "DEBUG: " << message << std::endl;
}

template < typename T > std::string to_string(const T& n) {
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
{
    long currTime = turtlebot_inputs.nanoSecs;

    //default state
    *vel = FORWARD;
    *ang_vel = NONE;
	
	const int maxAccel = 20;
	float xAccel = turtlebot_inputs.linearAccelX;
	float yAccel = turtlebot_inputs.linearAccelY;
	float zAccel = turtlebot_inputs.linearAccelZ;
    if (xAccel  > maxAccel && yAccel > maxAccel && zAccel > maxAccel){
      *vel = NONE;
      *ang_vel = NONE;
      *soundValue = 4;
	  return;
    };
	bool lwd = turtlebot_inputs.leftWheelDropped;
	bool rwd = turtlebot_inputs.rightWheelDropped;
    if (handleWheelDrop(lwd, rwd, soundValue, vel, ang_vel)){
		return;
	}
	
    if (handleSenseObstacle(currTime, turtlebot_inputs.ranges, soundValue, vel, ang_vel)){
		return;
	}

    bool lcs = turtlebot_inputs.sensor0State;
    bool ccs = turtlebot_inputs.sensor1State;
    bool rcs = turtlebot_inputs.sensor2State;
    if (handleCliffSensors(currTime, lcs, ccs, rcs, vel, ang_vel)){
		return;
	}

    bool lb = turtlebot_inputs.leftBumperPressed;
    bool cb = turtlebot_inputs.centerBumperPressed;
    bool rb = turtlebot_inputs.rightBumperPressed;
    if (handleBumpers(currTime, lb, cb, rb, vel, ang_vel)) {
		return;
	}
}

bool handleWheelDrop(bool lwd, bool rwd, uint8_t *soundValue,  float *vel, float *ang_vel){
  if (lwd == 1 || rwd == 1){
    *vel = NONE;
    *ang_vel = NONE;
    *soundValue = 4;
	return true;
  }
  return false;
}

bool iter = false;
long obsActionDuration = 1500000000;//0; //15 seconds. FIX
long obsStartTime = 0;
bool handleSenseObstacle(long currTime,float ranges[], uint8_t *soundValue,  float *vel, float *ang_vel){
  int numPoints = 0;
  float total = 0.0;

  if (iter == false){
    for (int i = 0; i < 640; i++){
		if (!isnan(ranges[i])){
	       numPoints++;
		   total += ranges[i];
      }
	}

    if (total/numPoints < 0.5){ //check to see if an obstacle is close
      *vel = NONE;
      *ang_vel = NONE;
      *soundValue = 2;
      iter = true;
      obsStartTime = currTime;
      return true;
    }
  }
  else if (currTime < obsStartTime + obsActionDuration){
	*vel = NONE;
    *ang_vel = NONE;
	return true;
  }
  else if (iter) { // start rotating
	*vel = NONE;
    *ang_vel = RIGHT;

    for (int i = 0; i < 640; i++){
		if (!isnan(ranges[i])){
	       numPoints++;
		   total += ranges[i];
      }
	}

    if (total/numPoints >= 0.5){ //check to see if an obstacle is close
        *ang_vel = NONE;
        *vel = FORWARD;
		iter = false;
        return true;
    }
  }
  else {
    return false;
  }
}

long cActionDuration = 4000000000;
long lcStartTime = 0;
long ccStartTime = 0;
long rcStartTime = 0;

bool handleCliffSensors(long currTime, bool lcs, bool ccs, bool rcs, float *vel, float *ang_vel) {
  /*********
  * CENTER *
  *********/
  if (ccs)
      ccStartTime = currTime;
  if (currTime < ccStartTime + (0.25 * cActionDuration)) {
      *vel = BACKWARD;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < ccStartTime + (0.50 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < ccStartTime + (0.75 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = RIGHT;
      return true;
  }
  else if (currTime < ccStartTime + (1.00 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }

  /*******
  * LEFT *
  *******/
  if (lcs)
      lcStartTime = currTime;
  if (currTime < lcStartTime + (0.25 * cActionDuration)) {
      *vel = BACKWARD;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < lcStartTime + (0.50 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < lcStartTime + (0.75 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = RIGHT;
      return true;
  }
  else if (currTime < lcStartTime + (1.00 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }

  /********
  * RIGHT *
  *********/
  if (rcs)
      rcStartTime = currTime;
  if (currTime < rcStartTime + (0.25 * cActionDuration)) {
      *vel = BACKWARD;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < rcStartTime + (0.50 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }
  else if (currTime < rcStartTime + (0.75 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = LEFT;
      return true;
  }
  else if (currTime < rcStartTime + (1.00 * cActionDuration)) {
      *vel = NONE;
      *ang_vel = NONE;
      return true;
  }

  return false;
}


long bActionDuration = 4000000000;
long lbStartTime = 0;
long cbStartTime = 0;
long rbStartTime = 0;

bool handleBumpers(long currTime, bool lb, bool cb, bool rb, float *vel, float *ang_vel) {
    /*********
    * CENTER *
    *********/
    if (cb)
        cbStartTime = currTime;
    if (currTime < cbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < cbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < cbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = RIGHT;
        return true;
    }
    else if (currTime < cbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }

    /*******
    * LEFT *
    *******/
    if (lb)
        lbStartTime = currTime;
    if (currTime < lbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < lbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < lbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = RIGHT;
        return true;
    }
    else if (currTime < lbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }

    /********
    * RIGHT *
    *********/
    if (rb)
        rbStartTime = currTime;
    if (currTime < rbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < rbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }
    else if (currTime < rbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = LEFT;
        return true;
    }
    else if (currTime < rbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return true;
    }

	return false;
}



