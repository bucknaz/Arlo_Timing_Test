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

extern volatile int safeToProceed ,
safeToRecede,
cliff,
floorO,
Escaping,
escapeLeftSpeed,
escapeRightSpeed,
minDistanceSensor,
ignoreProximity,
ignoreCliffSensors,
ignoreFloorSensors,
ignoreIRSensors,
pluggedIn;



/* Public Functions in this file */
int safetyOverride_start();
void safetyOverride_stop();
int safty_check(double CVel,double *LeftSpeed,double *RightSpeed);
void safetyOverride(void *par); // Use a cog to squelch incoming commands and perform safety procedures like halting, backing off, avoiding cliffs, calling for help, etc.




#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_SAFETYOVERIDE_H */
