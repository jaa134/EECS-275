#include "minimal_turtlebot/turtlebot_controller.h"
#include "turtlebot_controller.hpp"
#include "iostream"
#include "string"
#include "sstream"

/* 
  ********************
  STAGE 1 DELIVERABLES
  ********************

  If one of the bumpers is pressed, the robot should back away, turn, and continue on.
  Similar behavior should be implemented for the cliff sensors.

  The robot should stop moving if the wheel-drop sensors are triggered and announce an error.

  If the vector for linear acceleration passes 20 degrees, the motors should stop and announce error.

  The robot must stop if any obstacle becomes closer than a half a meter (0.5 m). It should not immediately move
  away from the obstacle, but announce its presence and wait fifteen (15) seconds for the obstacle move
  away before spinning in-place to find a direction that is not blocked and begin moving again
*/
void handleCliffSensors(long, bool, bool, bool, float*, float*);
void handleBumpers(long, bool, bool, bool, float*, float*);

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
    //default state
    *vel = FORWARD;
    *ang_vel = 0;

    long currTime = turtlebot_inputs.nanoSecs;

    bool lcs = turtlebot_inputs.sensor0State;
    bool ccs = turtlebot_inputs.sensor1State;
    bool rcs = turtlebot_inputs.sensor2State;
    handleCliffSensors(currTime, lcs, ccs, rcs, vel, ang_vel);

    bool lb = turtlebot_inputs.leftBumperPressed;
    bool cb = turtlebot_inputs.centerBumperPressed;
    bool rb = turtlebot_inputs.rightBumperPressed;
    handleBumpers(currTime, lb, cb, rb, vel, ang_vel);
}    


void handleCliffSensors(long currTime, bool lcs, bool ccs, bool rcs, float *vel, float *ang_vel) {
}


long bActionDuration = 4000000000;
long lbStartTime = 0;
long cbStartTime = 0;
long rbStartTime = 0;

void handleBumpers(long currTime, bool lb, bool cb, bool rb, float *vel, float *ang_vel) {
    /*********
    * CENTER *
    *********/
    if (cb)
        cbStartTime = currTime;
    if (currTime < cbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < cbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < cbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = RIGHT;
        return;
    }
    else if (currTime < cbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }

    /*******
    * LEFT *
    *******/
    if (lb)
        lbStartTime = currTime;
    if (currTime < lbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < lbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < lbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = RIGHT;
        return;
    }
    else if (currTime < lbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }

    /********
    * RIGHT *
    *********/
    if (rb)
        rbStartTime = currTime;
    if (currTime < rbStartTime + (0.25 * bActionDuration)) {
        *vel = BACKWARD;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < rbStartTime + (0.50 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }
    else if (currTime < rbStartTime + (0.75 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = LEFT;
        return;
    }
    else if (currTime < rbStartTime + (1.00 * bActionDuration)) {
        *vel = NONE;
        *ang_vel = NONE;
        return;
    }
}