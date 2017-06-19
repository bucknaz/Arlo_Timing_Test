/*
 *
 *
 *
 *
 */


#ifndef ARLO_SAFETYOVERIDE_H
#define ARLO_SAFETYOVERIDE_H

#if defined(__cplusplus)
extern "C" { 
#endif

extern volatile int safeToProceed;
extern volatile int safeToRecede;
extern volatile int cliff;
extern volatile int floorO;
extern volatile int Escaping;
extern volatile int escapeLeftSpeed;
extern volatile int escapeRightSpeed;
extern volatile int minDistanceSensor;
extern volatile int ignoreProximity;
extern volatile int ignoreCliffSensors;
extern volatile int ignoreFloorSensors;
extern volatile int ignoreIRSensors;
extern volatile int pluggedIn;



/* Public Functions in this file */
int safetyOverride_start();
void safetyOverride_stop();
int safty_check(double CVel,double *LeftSpeed,double *RightSpeed);
void safetyOverride(void *par); // Use a cog to squelch incoming commands and perform safety procedures like halting, backing off, avoiding cliffs, calling for help, etc.




#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_SAFETYOVERIDE_H */
