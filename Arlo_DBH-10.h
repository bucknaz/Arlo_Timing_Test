/*
 *
 *
 *
 *
 */


#ifndef ARLO_DBH10_H
#define ARLO_DBH10_H

#if defined(__cplusplus)
extern "C" { 
#endif


/* Some defines for comunicating with the dhb-10 board */
//#define ARLO_DEBUG
//#define XBEE_TERMINAL 
//#define HALF_DUPLEX

#define DHB10_LEN 64
#define ARD_DEFAULT_SERVO_L 16
#define ARD_DEFAULT_SERVO_R 17
#define ARD_DEFAULT_OPENED 0
// Minimum number of ms between sending a new command
#define MIN_MSG_DELTA 25



/* Public Functions in this file */
int dhb10_send(char *CmdStr);
char *drive_open(void);
void drive_close(void);

int drive_set_stop();
int drive_set_gospd(int left,int right);
int drive_get_dist(int *left,int *right);
int drive_get_head(int *heading);
int drive_get_spd(int *left,int *right);
int drive_rst();
int drive_get_hwver(char *ver);
int drive_get_ver(char *ver);
int get_reply(char *buf);
int get_last(char *buf);

#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_DBH10_H */
