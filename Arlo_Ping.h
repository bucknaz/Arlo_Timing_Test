/*
 *
 *
 *
 *
 */


#ifndef ARLO_PING_H
#define ARLO_PING_H

#if defined(__cplusplus)
extern "C" { 
#endif


// Global Storage for PING & IR Sensor Data:
extern int pingArray[NUMBER_OF_PING_SENSORS];
extern int irArray[NUMBER_OF_IR_SENSORS];


/* Public Functions in this file */
int ping_start();
void ping_stop();
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances




#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_PING_H */
