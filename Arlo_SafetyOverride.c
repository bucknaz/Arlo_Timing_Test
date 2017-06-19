
#include "simpletools.h"
#include <stdbool.h>

#include "per_robot_settings_for_propeller_c_code.h"

#include "Arlo_SafetyOverride.h"
#include "Arlo_Ping.h"

extern int abd_speedLimit;
extern int abdR_speedLimit;

// For "Safety Override" Cog
volatile int safeToProceed = 0;
volatile int safeToRecede = 0;
volatile int cliff = 0;
volatile int floorO = 0;
volatile int Escaping = 0;
volatile int escapeLeftSpeed = 0;
volatile int escapeRightSpeed = 0;
volatile int minDistanceSensor = 0;
volatile int ignoreProximity = 0;
volatile int ignoreCliffSensors = 0;
volatile int ignoreFloorSensors = 0;
volatile int ignoreIRSensors = 0;
volatile int pluggedIn = 0;
volatile int wasEscaping = 0;

// This can use proximity sensors to detect obstacles (including people) and cliffs
// This can use the gyro to detect tipping
// This can use the gyro to detect significant heading errors due to slipping wheels when an obstacle is encountered or high centered

// Local Sensor Polling Cog
static volatile int safety_Overridecog;
static int safetyOverride_Stack[128]; // If things get weird make this number bigger!

/* Here you set up arrays showing what the halt distance,
   and distance to start slowing down is for each sensor.
   Each sensor needs to have its own response because of their various angles.
   We cannot use the same distance response for each,
   and we may want different max speeds and escape sequences too.
   Remember the order is based on the order that they are plugged
   in to your board.
   WARNING: Be sure the array has the correct number of entries! */

int haltDistance[NUMBER_OF_PING_SENSORS];
int startSlowDownDistance[NUMBER_OF_PING_SENSORS];
// Set shorter distances for IR sensors because they are less reliable.
int IRstartSlowDownDistance[NUMBER_OF_IR_SENSORS];
/* My last two IR sensors are cliff sensors, thus the "0, 0" because slowdown does not apply to them. */


int safetyOverride_start()
{
  safetyOverride_stop();
  /* Build the ping sensor distance arrays */
  #ifdef FRONT_FAR_LEFT_SENSOR
  haltDistance[FRONT_FAR_LEFT_SENSOR] = FRONT_FAR_LEFT_HALT_DIST;
  startSlowDownDistance[FRONT_FAR_LEFT_SENSOR] = FRONT_FAR_LEFT_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_NEAR_LEFT_SENSOR
  haltDistance[FRONT_NEAR_LEFT_SENSOR] = FRONT_NEAR_LEFT_HALT_DIST;
  startSlowDownDistance[FRONT_NEAR_LEFT_SENSOR] = FRONT_NEAR_LEFT_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_CENTER_SENSOR
  haltDistance[FRONT_CENTER_SENSOR] = FRONT_CENTER_HALT_DIST;
  startSlowDownDistance[FRONT_CENTER_SENSOR] = FRONT_CENTER_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_NEAR_RIGHT_SENSOR
  haltDistance[FRONT_NEAR_RIGHT_SENSOR] = FRONT_NEAR_RIGHT_HALT_DIST;
  startSlowDownDistance[FRONT_NEAR_RIGHT_SENSOR] = FRONT_NEAR_RIGHT_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_FAR_RIGHT_SENSOR
  haltDistance[FRONT_FAR_RIGHT_SENSOR] = FRONT_FAR_RIGHT_HALT_DIST;
  startSlowDownDistance[FRONT_FAR_RIGHT_SENSOR] = FRONT_FAR_RIGHT_SLOWDWN_DIST;
  #endif

  #ifdef REAR_FAR_LEFT_SENSOR
  haltDistance[REAR_FAR_LEFT_SENSOR] = REAR_FAR_LEFT_HALT_DIST;
  startSlowDownDistance[REAR_FAR_LEFT_SENSOR] = REAR_FAR_LEFT_SLOWDWN_DIST;
  #endif

  #ifdef REAR_NEAR_LEFT_SENSOR
  haltDistance[REAR_NEAR_LEFT_SENSOR] = REAR_NEAR_LEFT_HALT_DIST;
  startSlowDownDistance[REAR_NEAR_LEFT_SENSOR] = REAR_NEAR_LEFT_SLOWDWN_DIST;
  #endif

  #ifdef REAR_CENTER_SENSOR
  haltDistance[REAR_CENTER_SENSOR] = REAR_CENTER_HALT_DIST;
  startSlowDownDistance[REAR_CENTER_SENSOR] = REAR_CENTER_SLOWDWN_DIST;
  #endif

  #ifdef REAR_NEAR_RIGHT_SENSOR
  haltDistance[REAR_NEAR_RIGHT_SENSOR] = REAR_NEAR_RIGHT_HALT_DIST;
  startSlowDownDistance[REAR_NEAR_RIGHT_SENSOR] = REAR_NEAR_RIGHT_SLOWDWN_DIST;
  #endif
  
  #ifdef REAR_FAR_RIGHT_SENSOR
  haltDistance[REAR_FAR_RIGHT_SENSOR] = REAR_FAR_RIGHT_HALT_DIST;
  startSlowDownDistance[REAR_FAR_RIGHT_SENSOR] = REAR_FAR_RIGHT_SLOWDWN_DIST;
  #endif
  
  /* Build the IR sensor distance arrays */
  #ifdef FRONT_FAR_LEFT_IR_SENSOR
  //haltDistance[FRONT_FAR_LEFT_IR_SENSOR] = FRONT_FAR_LEFT_IR_HALT_DIST;
  startSlowDownDistance[FRONT_FAR_LEFT_IR_SENSOR] = FRONT_FAR_LEFT_IR_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_NEAR_LEFT_IR_SENSOR
  //haltDistance[FRONT_NEAR_LEFT_IR_SENSOR] = FRONT_NEAR_LEFT_IR_HALT_DIST;
  startSlowDownDistance[FRONT_NEAR_LEFT_IR_SENSOR] = FRONT_NEAR_LEFT_IR_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_CENTER_IR_SENSOR
  //haltDistance[FRONT_CENTER_IR_SENSOR] = FRONT_CENTER_IR_HALT_DIST;
  startSlowDownDistance[FRONT_CENTER_IR_SENSOR] = FRONT_CENTER_IR_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_NEAR_RIGHT_IR_SENSOR
  //haltDistance[FRONT_NEAR_RIGHT_IR_SENSOR] = FRONT_NEAR_RIGHT_IR_HALT_DIST;
  startSlowDownDistance[FRONT_NEAR_RIGHT_IR_SENSOR] = FRONT_NEAR_RIGHT_IR_SLOWDWN_DIST;
  #endif

  #ifdef FRONT_FAR_RIGHT_IR_SENSOR
  //haltDistance[FRONT_FAR_RIGHT_IR_SENSOR] = FRONT_FAR_RIGHT_IR_HALT_DIST;
  startSlowDownDistance[FRONT_FAR_RIGHT_IR_SENSOR] = FRONT_FAR_RIGHT_IR_SLOWDWN_DIST;
  #endif

  #ifdef REAR_FAR_LEFT_IR_SENSOR
  //haltDistance[REAR_FAR_LEFT_IR_SENSOR] = REAR_FAR_LEFT_IR_HALT_DIST;
  startSlowDownDistance[REAR_FAR_LEFT_IR_SENSOR] = REAR_FAR_LEFT_IR_SLOWDWN_DIST;
  #endif

  #ifdef REAR_NEAR_LEFT_IR_SENSOR
  //haltDistance[REAR_NEAR_LEFT_IR_SENSOR] = REAR_NEAR_LEFT_IR_HALT_DIST;
  startSlowDownDistance[REAR_NEAR_LEFT_IR_SENSOR] = REAR_NEAR_LEFT_IR_SLOWDWN_DIST;
  #endif

  #ifdef REAR_CENTER_IR_SENSOR
  //haltDistance[REAR_CENTER_IR_SENSOR] = REAR_CENTER_IR_HALT_DIST;
  startSlowDownDistance[REAR_CENTER_IR_SENSOR] = REAR_CENTER_IR_SLOWDWN_DIST;
  #endif

  #ifdef REAR_NEAR_RIGHT_IR_SENSOR
  //haltDistance[REAR_NEAR_RIGHT_IR_SENSOR] = REAR_NEAR_RIGHT_IR_HALT_DIST;
  startSlowDownDistance[REAR_NEAR_RIGHT_IR_SENSOR] = REAR_NEAR_IR_RIGHT_SLOWDWN_DIST;
  #endif
  
  #ifdef REAR_FAR_RIGHT_IR_SENSOR
  //haltDistance[REAR_FAR_RIGHT_IR_SENSOR] = REAR_FAR_RIGHT_IR_HALT_DIST;
  startSlowDownDistance[REAR_FAR_RIGHT_IR_SENSOR] = REAR_FAR_RIGHT_IR_SLOWDWN_DIST;
  #endif
  
  safety_Overridecog = 1 + cogstart(&safetyOverride, NULL, safetyOverride_Stack, sizeof safetyOverride_Stack);
}

void safetyOverride_stop()
{
  if(safety_Overridecog)
  {
    cogstop(safety_Overridecog -1);
    safety_Overridecog = 0;
  }    
}



int safty_check(double CVel,double *LeftSpeed,double *RightSpeed)
{
  int results = 0;
  if (Escaping == 1) 
  {
    wasEscaping = 1;
    *LeftSpeed = escapeLeftSpeed;
    *RightSpeed = escapeRightSpeed;
    results = 1;
  } 
  else if (wasEscaping == 1) 
  {
    // Halt robot before continuing normally if we were escaping before now.
    wasEscaping = 0;
    *LeftSpeed = 0;
    *RightSpeed = 0;
    results = 1;
  } 
  else if((CVel > 0 && safeToProceed == 0) || (CVel < 0 && safeToRecede == 0) ) 
  {
    *LeftSpeed = 0;
    *RightSpeed = 0;
    results = 1;    
  }              
  /* since cliff and floor also clear safeToProceed 
  this check is not needed
  else if (CVel >= 0 && (cliff == 1 || floorO == 1)) 
  {
    // Cliffs and cats are no joke!
    *LeftSpeed = 0;
    *RightSpeed = 0;
    results = 1;
  } 
  */ 
  return(results);
}



/* TESTS:
   1. Make sure output sensor readings to ROS are near real time.
   2. Make sure "escape" operations are fast and accurate.
   */
void safetyOverride(void *par) {

   void setEscapeSpeeds(int left, int right) {
       escapeLeftSpeed = left;
       escapeRightSpeed = right;
   }

    int increaseThrottleRamp = 0;
    int decreaseThrottleRamp = 0;
    // Declare all variables up front so they do not have to be created in the loop, only set.
    // This may or may not improve performance.
    int blockedSensor[NUMBER_OF_PING_SENSORS] = {0};
    int i, blockedF = 0, blockedR = 0, foundCliff = 0, floorObstacle = 0, pleaseEscape = 0, minDistance = 255, minRDistance = 255, newSpeedLimit = MAXIMUM_SPEED;
    while (1) {
        if (ignoreProximity == 0) {
            // Reset blockedSensor array to all zeros.
            memset(blockedSensor, 0, sizeof(blockedSensor));
            blockedF = 0;
            blockedR = 0;
            pleaseEscape = 0;
            minDistance = 255;
            minRDistance = 255;

            #ifdef hasCliffSensors
            foundCliff = 0;
            if (ignoreCliffSensors == 0) {
              // Check Cliff Sensors first
              for (i = FIRST_CLIFF_SENSOR; i < FIRST_CLIFF_SENSOR + NUMBER_OF_CLIFF_SENSORS; i++) {
                if (irArray[i] > FLOOR_DISTANCE) {
                  // Set the global 'cliff' variable so we can see this in ROS.
                  cliff = 1;
                  safeToProceed = 0; // Prevent main thread from setting any drive_speed
                  // Use this to give the "all clear" later if it never gets set
                  blockedF = 1;
                  // Use this to clear the 'cliff' variable later if this never gets set.
                  foundCliff = 1;
                  blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
                  pleaseEscape = 1;
                  }
              }
            }
            // Clear the global 'cliff' variable if no cliff was seen.
            if (foundCliff == 0) {
                cliff = 0;
            }
            #endif

            #ifdef hasFloorObstacleSensors
            floorObstacle = 0;
            if (ignoreFloorSensors == 0) {
              for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
                if (floorArray[i] == 0) {
                  // Set the global 'floorO' variable so we can see this in ROS.
                  floorO = 1;
                  safeToProceed = 0; // Prevent main thread from setting any drive_speed
                  // Use this to give the "all clear" later if it never gets set
                  blockedF = 1;
                  // Use this to clear the 'floorO' variable later if this never gets set.
                  floorObstacle = 1;
                  blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
                  pleaseEscape = 1;
                  }
              }
            }
            // Clear the global 'floorO' variable if no floor obstacle was seen.
            if (floorObstacle == 0) {
                floorO = 0;
            }
            #endif

            #ifdef hasFrontPingSensors
            // Walk Front PING Sensors to find blocked paths and halt immediately
            for (i = FIRST_FRONT_PING_SENSOR_NUMBER; i < HOW_MANY_FRONT_PING_SENSORS + FIRST_FRONT_PING_SENSOR_NUMBER; i++) {
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToProceed = 0;
                        blockedF = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        // Escape just after, to try make a buffer to avoid back and forthing.
                        if (pingArray[i] < haltDistance[i]) {
                            pleaseEscape = 1;
                        }
                    }
                    // For speed restriction:
                    if (pingArray[i] < minDistance) {
                        minDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasFrontUpperDeckSensors
            // Walk Upper Deck Sensors
            for (i = FIRST_FRONT_UPPER_SENSOR_NUMBER; i < HOW_MANY_FRONT_UPPER_SENSORS + FIRST_FRONT_UPPER_SENSOR_NUMBER; i++) {
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    // Halt just before.
                    if (pingArray[i] <= haltDistance[i] + 1) {
                        // Prevent main thread from setting any drive_speed
                        safeToProceed = 0;
                         // Use this to give the "all clear" later if it never gets set
                        blockedF = 1;
                         // Keep track of which sensors are blocked for intelligent escape sequences.
                        blockedSensor[i] = 1;
                        if (pingArray[i] < haltDistance[i]) {
                            // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                        }
                    }
                    // For speed restriction:
                    if (pingArray[i] < minDistance) {
                        minDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasRearPingSensors
            // Walk REAR Sensor Array to find blocked paths and halt immediately
            for (i = FIRST_REAR_PING_SENSOR_NUMBER; i < FIRST_REAR_PING_SENSOR_NUMBER + HOW_MANY_REAR_PING_SENSORS; i++) {
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToRecede = 0; // Prevent main thread from setting any drive_speed
                        blockedR = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        if (pingArray[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                    }
                    // For speed restriction:
                    if (pingArray[i] < minRDistance) {
                        minRDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasRearUpperDeckSensors
            for (i = FIRST_REAR_UPPER_SENSOR_NUMBER; i < FIRST_REAR_UPPER_SENSOR_NUMBER + HOW_MANY_REAR_UPPER_SENSORS; i++) { // Only use the rear sensors
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToRecede = 0; // Prevent main thread from setting any drive_speed
                        blockedR = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        if (pingArray[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                    }
                    // For speed restriction:
                    if (pingArray[i] < minRDistance) {
                        minRDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            if (ignoreIRSensors == 0) {
              #ifdef hasFrontIRSensors
              // Walk front IR Sensors
              for (i = FIRST_FRONT_IR_SENSOR_NUMBER; i < HOW_MANY_FRONT_IR_SENSORS + FIRST_FRONT_IR_SENSOR_NUMBER; i++) {
                  if (irArray[i] < IRstartSlowDownDistance[i])  {
                      if (irArray[i] <= haltDistance[i] + 1) {
                          // Prevent main thread from setting any drive_speed
                          safeToProceed = 0;
                          // Use this to give the "all clear" later if it never gets set
                          blockedF = 1;
                          // Keep track of which sensors are blocked for intelligent escape sequences.
                          blockedSensor[i] = 1;
                          if (irArray[i] < haltDistance[i]) {
                              pleaseEscape = 1;
                          }
                      }
                      // For speed restriction:
                      if (irArray[i] < minDistance) {
                          minDistance = irArray[i];
                          minDistanceSensor = i;
                      }
                  }
              }
              #endif

              #ifdef hasRearIRSensors
              for (i = FIRST_REAR_IR_SENSOR_NUMBER; i < FIRST_REAR_IR_SENSOR_NUMBER + HOW_MANY_REAR_IR_SENSORS; i++) {
                  #ifdef RENAME_REAR_IR_SENSOR
                  int sensorFakeIndex = RENAME_REAR_IR_SENSOR;
                  #else
                  int sensorFakeIndex = i;
                  #endif
                  if (irArray[i] < IRstartSlowDownDistance[i]) {
                     if (irArray[i] <= haltDistance[sensorFakeIndex] + 1) {
                          safeToRecede = 0; // Prevent main thread from setting any drive_speed
                          blockedR = 1; // Use this to give the "all clear" later if it never gets set
                          blockedSensor[sensorFakeIndex] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                          if (irArray[i] < haltDistance[sensorFakeIndex])
                              pleaseEscape = 1;
                      }
                      // For speed restriction:
                      if (irArray[i] < minRDistance) {
                          minRDistance = irArray[i];
                          minDistanceSensor = i;
                      }
                  }
              }
            #endif
            }

            // Reduce Speed Limit when we are close to an obstruction
            /* EXPLANATION minDistance won't be set unless a given sensor is closer than its particular startSlowDownDistance value, so we won't be slowing down if sensor 0 is 40, only if it is under 10 */
            if (minDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minDistance - haltDistance[minDistanceSensor]) * (MAXIMUM_SPEED / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > MAXIMUM_SPEED) {
                    newSpeedLimit = MAXIMUM_SPEED;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abd_speedLimit) {
                    if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) {
                        abd_speedLimit = abd_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abd_speedLimit) {
                    if (decreaseThrottleRamp == DECREASE_THROTTLE_RATE) {
                        abd_speedLimit = abd_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abd_speedLimit < MAXIMUM_SPEED) {
                    if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) // Slow ramping up
                        abd_speedLimit = abd_speedLimit + 1;
                }
            }

            // Same for REVERSE Speed Limit
            if (minRDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minRDistance - haltDistance[minDistanceSensor]) * (MAXIMUM_SPEED / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > MAXIMUM_SPEED) {
                    newSpeedLimit = MAXIMUM_SPEED;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abdR_speedLimit) {
                    if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) {
                        abdR_speedLimit = abdR_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abdR_speedLimit) {
                    if (decreaseThrottleRamp == DECREASE_THROTTLE_RATE) {
                        abdR_speedLimit = abdR_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abdR_speedLimit < MAXIMUM_SPEED) {
                    if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) // Slow ramping up
                        abdR_speedLimit = abdR_speedLimit + 1;
                }
            }

            // Clear forward and backward individually now.
            if (blockedF == 0) {
                safeToProceed = 1;
            }
            if (blockedR == 0) {
                safeToRecede = 1;
            }

            // If NO sensors are blocked, give the all clear!
            if (blockedF == 0 && blockedR == 0) {
                Escaping = 0; // Have fun!
            } else {
                if (pleaseEscape == 1 && pluggedIn == 0) {
                    // If it is plugged in, don't escape!
                    Escaping = 1; // This will stop main thread from driving the motors.
                    /* At this point we are blocked, so it is OK to take over control
                       of the robot (safeToProceed == 0, so the main thread won't do anything),
                       and it is safe to do work ignoring the need to slow down or stop
                       because we know our position pretty well.
                       HOWEVER, you will have to RECHECK distances yourself if you are going to move
                       in this program location.
                       */
                       if (safeToRecede == 1) {
                        // The order here determines priority.
                        #ifdef FRONT_CENTER_SENSOR
                        if (blockedSensor[FRONT_CENTER_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #ifdef FRONT_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[FRONT_3D_MOUNTED_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_UPPER_DECK_CENTER_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_CENTER_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_LEFT_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, -(MINIMUM_SPEED * 2)); // Curve out to the right
                        #endif
                        #ifdef FRONT_UPPER_DECK_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_LEFT_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, -(MINIMUM_SPEED * 2)); // Curve out to the right
                        #endif
                        #ifdef FRONT_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_RIGHT_SENSOR] == 1) {
                            setEscapeSpeeds(-(MINIMUM_SPEED * 2), -MINIMUM_SPEED); // Curve out to the left
                        #endif
                        #ifdef FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR] == 1) {
                            setEscapeSpeeds(-(MINIMUM_SPEED * 2), -MINIMUM_SPEED); // Curve out to the left
                        #endif
                        #ifdef FRONT_FAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_LEFT_SENSOR] == 1) {
                            setEscapeSpeeds(0, -MINIMUM_SPEED); // Turn out to the right slowly
                        #endif
                        #ifdef FRONT_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_RIGHT_SENSOR] == 1) {
                            setEscapeSpeeds(-MINIMUM_SPEED, 0); // Turn out to the left slowly
                        #endif
                        }
                        #endif
                    } else if (safeToProceed == 1) { // Escaping for rear sensors, these will move more generically forward.
                        #ifdef REAR_CENTER_SENSOR
                        if (blockedSensor[REAR_CENTER_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
                        #ifdef REAR_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[REAR_3D_MOUNTED_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_UPPER_DECK_SENSOR
                        } else if (blockedSensor[REAR_UPPER_DECK_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_RIGHT_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED * 2);
                        #endif
                        #ifdef REAR_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_LEFT_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED * 2, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_FAR_RIGHT_SENSOR] == 1) {
                            setEscapeSpeeds(MINIMUM_SPEED, 0);
                        #endif
                        #ifdef REAR_FAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_FAR_LEFT_SENSOR] == 1) {
                            setEscapeSpeeds(0, MINIMUM_SPEED);
                        #endif
                        }
                        #endif
                    } else { // We are trapped!!
                        // Turns out we cannot escape, so turn off "Escaping",
                        // and now drive control should refuse to move forward or back,
                        // due to safeToRecede & safeToProceed both == 1,
                        // but it should be willing to rotate in place,
                        // which is a normal function of both arlobot_explore
                        // and the navigation stack's "clearing" function.
                        Escaping = 0;
                    }
                } else { // This is the "halt" but don't "escape" action for that middle ground.
                    if (Escaping == 1) {// If it WAS Escaping, stop it now.
                        setEscapeSpeeds(0, 0); // return to stopped before giving control back to main thread
                    }
                    Escaping = 0; // Blocked, but not close enough to Escape yet
                }
            }

            increaseThrottleRamp = increaseThrottleRamp + 1;
            if(increaseThrottleRamp > INCREASE_THROTTLE_RATE)
                increaseThrottleRamp = 0;
            decreaseThrottleRamp = decreaseThrottleRamp + 1;
            if(decreaseThrottleRamp > DECREASE_THROTTLE_RATE)
                decreaseThrottleRamp = 0;

        } else {
            /* All limits and blocks must be cleared if we are going to ignore
            proximity. Otherwise we get stuck! */
            Escaping = 0;
            safeToProceed = 1;
            safeToRecede = 1;
            abd_speedLimit = MAXIMUM_SPEED;
            abdR_speedLimit = MAXIMUM_SPEED;
        }
        pause(1); // Just throttles this cog a little.
    }
}



