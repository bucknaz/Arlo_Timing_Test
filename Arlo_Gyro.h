/*
 *
 *
 *
 *
 */


#ifndef ARLO_GYRO_H
#define ARLO_GYRO_H

#if defined(__cplusplus)
extern "C" { 
#endif

/* Public Functions in this file */
int gyro_start();
void gyro_stop();
void set_rotating(int ls,int rs );
void pollGyro(void *par);


#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_GYRO_H */
