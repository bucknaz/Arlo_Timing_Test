
#include "simpletools.h"
#include "fdserial.h"
#include "Arlo_DBH-10.h"
#include "per_robot_settings_for_propeller_c_code.h"
#include "sequencer.h"

//Include our DHB-10 board driver
#include "libdhb10.h"


//fdserial *ard_dhb10_arlo;

//int ard_servo_pin_L    = ARD_DEFAULT_SERVO_L;
//int ard_servo_pin_R    = ARD_DEFAULT_SERVO_R;
//int ard_opened         = ARD_DEFAULT_OPENED;

//char dhb10_reply[DHB10_LEN];
//char dhb10_cmd[DHB10_LEN];





/*
 *   functions to open, close and comunicate with the dhb10_reply
 *
 */

void drive_open(void)
{  
  dbh10_cog_start();  //Start the cog for comunicating with the DBH-10
}


void drive_close(void)
{
//  drive_set_stop();
//  drive_rst();
  #ifdef EMULATE_ARLO
  pause(2); //should time this
  #else
  dbh10_cog_stop();
  #endif
}

/*
 * dhb10_send(char *CmdStr)
 *
 * Returns lenght of responce on succsess else -1
*/
/*
int dhb10_send(char *CmdStr)
{  
  #ifdef EMULATE_ARLO
  pause(3);//this is not the right time but never should get called directly
  return(1);
  #else
  memset(dhb10_reply, 0, DHB10_LEN);
  memset(dhb10_cmd, 0, DHB10_LEN);
  char *reply = dhb10_reply;
  memset(reply,0,DHB10_LEN);//Empty the String
  strcpy(dhb10_cmd, CmdStr);
    
  if(!ard_opened) drive_open();

  int ca = 0, cta = 0;
  
  fdserial_rxFlush(ard_dhb10_arlo);

  int i = 0;
  
  while(1) 
  {
    //if( (CmdStr[i] == 0)) break;
    fdserial_txChar(ard_dhb10_arlo, CmdStr[i]);
    if((CmdStr[i] == '\r') || (CmdStr[i] == 0)) break;
    i++;
    //if( (CmdStr[i] == 0)) break;
  }
  
  // No cmd just a return
  if((i == 0) && (CmdStr[i] == '\r'))
  {
    return 0;
  }    
  
  i = 0;
  int dt = sequencer_get()+100;
  while(1)
  {
    cta = fdserial_rxCount(ard_dhb10_arlo);
    if(cta)
    {
      ca = readChar(ard_dhb10_arlo);
      dhb10_reply[i] = ca;
      if(ca == '\r' || ca == 0)
      {
        reply = dhb10_reply;
        break;
      }  
      i++;
    }

    if( dt < sequencer_get())
    {
      strcpy(reply, "Error, no reply from DHB-10!\r");
      break;
    }  

  }
  
  
  //remove the \r
  if(strlen(reply) > 0)
     reply[strlen(reply)-1] = 0;
  //Check for errors
  if( reply[0] == 'E' || reply[strlen(reply)] == 'E')
    return(-1);
    
  return(strlen(reply));    
  #endif
}
*/

/*
 * drive_set_stop()
 *
 * Returns 0 on succsess else 1
*/
int drive_set_stop()
{
  #ifdef EMULATE_ARLO
  pause(7);
  #else
    dhb10_stop();
  #endif
  return(0);  
}

/*
 * drive_set_gospd(int left,int right)
 *
 * Returns 0 on succsess else 1
*/
int drive_set_gospd(int left,int right)
{
  #ifdef EMULATE_ARLO
  pause(7);
  #else
  dhb10_gospd(left,right);
  #endif
  return(0);  
}

/*
 * drive_get_dist(int *left,int *right)
 * Returns 0 on succsess else 1
*/
int drive_get_dist(int *left,int *right)
{
  #ifdef EMULATE_ARLO
  pause(8);
  *left = 45;
  *right = 44;
  #else
  get_speed(left,right);
  //errors = get_speed(left,right);
  #endif
  return(0);
}

/*
 * drive_get_head(int *heading)
 *
 * Returns 0 on succsess else 1
*/
int drive_get_head(int *heading)
{
  #ifdef EMULATE_ARLO
  pause(7);
  *heading = 45;
  #else
  get_heading(heading);
  #endif
  return(0);
}


/*
 * drive_get_spd(int *left,int *right)
 *
 * Returns 0 on succsess else 1
*/
int drive_get_spd(int *left,int *right)
{
  #ifdef EMULATE_ARLO
  pause(8);
  *left = 120;
  *right = 119;
  #else
  get_distance(left,right);
  #endif
  return(0);
}


/*
 * drive_rst()
 *
 * Returns 0 on succsess else 1
*/
int drive_rst()
{
  #ifdef EMULATE_ARLO
  pause(4);
  #else
  dhb10_rst();
  #endif
  return(0);  
}

/*   Utility functions to make life easyer */
//RST
//VERB
/*
 * drive_get_hwver(int *ver)
 *
 * Returns 0 on succsess else 1
*/
/*
int drive_get_hwver(char *ver)
{
  #ifdef EMULATE_ARLO
  pause(7);
  *ver = 0;
  #else
  char *reply = dhb10_reply;
  int results = dhb10_send("HWVER\r");
  if ( results == -1) 
  {
    *ver = 0;
    return(1);
  } 
  strcpy(ver,dhb10_reply); 
  #endif
  return(0);
}
*/

/*
 * drive_get_ver(int *ver)
 *
 * Returns 0 on succsess else 1
*/
/*
int drive_get_ver(char *ver)
{
  #ifdef EMULATE_ARLO
  pause(7);
  *ver = 0;
  #else
  char *reply = dhb10_reply;
  int results = dhb10_send("VER\r");
  if ( results == -1) 
  {
    *ver = 0;
    return(1);
  } 
  strcpy(ver,dhb10_reply);
  #endif
  return(0);
}
*/
/*
int get_reply(char *buf)
{
  strcpy(buf,dhb10_reply);
  return(strlen(buf));
}
*/
/*
int get_last(char *buf)
{
  strcpy(buf,dhb10_cmd);
  return(strlen(buf));
}
*/