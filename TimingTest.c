/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "fdserial.h"

#include "per_robot_settings_for_propeller_c_code.h"
#include "Arlo_Ping.h"
#include "Arlo_SafetyOverride.h"
#include "Arlo_DBH-10.h"
#include "Arlo_Ir.h"
#include "Arlo_Gyro.h"
#include "sequencer.h"

// This is the ROS io port
fdserial *term;

static double gyroHeading = 0.0;
double Heading = 1.0, X = 0.0, Y = 0.0, deltaDistance, V, Omega;

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script

double distancePerCount = 0.0, trackWidth = 0.0;
double CommandedVelocity = 0.0;
double CommandedAngularVelocity = 0.0;
double angularVelocityOffset = 0.0, expectedLeftSpeed = 0.0, expectedRightSpeed = 0.0;
int curLeftspeed = 0, curRightSpeed =0;
int robotInitialized=0;

// declared outside main
int abd_speedLimit = MAXIMUM_SPEED;
int abdR_speedLimit = MAXIMUM_SPEED; // Reverse speed limit to allow robot to reverse fast if it is blocked in front and visa versa


static char sensorbuf[132];
#define RXBUFFERLEN 40
static char rx_buf[RXBUFFERLEN];// A Buffer long enough to hold the longest line ROS may send.
static char in_buf[RXBUFFERLEN];// A Buffer long enough to hold the longest line ROS may send.
static int rx_count = 0;
static int got_one = 0;

float BatteryVolts=12.0, RawBatVolts=4.0;


//Stack for ros emulater
#ifdef EMULATE_ROS
uint32_t rstack[40+25];
void ROS();
#endif


/*
 *  check_input(fdserial *term)
 *
 *  Checks if any characters have been recived and move then to the buffer
 *  when a complete line has been recived copy the buffer to in_buf and 
 *  sets a flag to indicate a complete command has been recived.
 *
 */
int check_input(fdserial *term)
{
  if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer        
    while (rx_count < RXBUFFERLEN && (fdserial_rxReady(term) != 0)) {
      rx_buf[rx_count] = fdserial_rxTime(term, 10); // fdserial_rxTime will time out. Otherwise a spurious character on the line will cause us to get stuck forever      
      if (rx_buf[rx_count] == '\r' || rx_buf[rx_count] == '\n')
      {
        rx_buf[rx_count] = 0;
        memcpy(in_buf,rx_buf,rx_count+1);
        rx_count = 0;
        got_one = 1;
        break;
      }       
      rx_count++;
    }          
  }
  return(got_one);
}

/*
 *  pars_input()
 *
 *  Parse the data recived in the in_buf and populate 
 *  values as required
 */
void pars_input()
{
  if (in_buf[0] == 's') 
  {
    char *token;
    token = strtok(in_buf, delimiter);
    token = strtok(NULL, delimiter);
    char *unconverted;
    CommandedVelocity = strtod(token, &unconverted);
    token = strtok(NULL, delimiter);
    CommandedAngularVelocity = strtod(token, &unconverted);
        angularVelocityOffset = CommandedAngularVelocity * (trackWidth * 0.5);
    /* Prevent saturation at max wheel speed when a compound command is sent.
       Without this, if your max speed is 50, and ROS asks us to set
       one wheel at 50 and the other at 100, we will end up with both
       at 50 changing a turn into a straight line!

       Remember that max speed is variable based on parameters within
       this code, such as proximity to walls, etc. */

    // Use forward speed limit for rotate in place.
    if (CommandedVelocity > 0 && (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset) < CommandedVelocity) 
    {
      CommandedVelocity = (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset);
      // Use abdR_speedLimit for reverse movement.
    } 
    else if (CommandedVelocity < 0 && -((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset)) > CommandedVelocity) 
    {
      // In theory ROS never requests a negative angular velocity, only teleop
      CommandedVelocity = -((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset));
    } 

    // These only need to be calculated once per twist msg
    expectedLeftSpeed =  CommandedVelocity - angularVelocityOffset;
    expectedRightSpeed = CommandedVelocity + angularVelocityOffset;
    expectedLeftSpeed =  expectedLeftSpeed / distancePerCount;
    expectedRightSpeed = expectedRightSpeed / distancePerCount;                
  }     
   
  else if (in_buf[0] == 'd') 
  {
    char *token;
    token = strtok(in_buf, delimiter);
    token = strtok(NULL, delimiter);
    char *unconverted;
    trackWidth = strtod(token, &unconverted);
    token = strtok(NULL, delimiter);
    distancePerCount = strtod(token, &unconverted);
    token = strtok(NULL, delimiter);
    ignoreProximity = (int)(strtod(token, &unconverted));
    token = strtok(NULL, delimiter);
    ignoreCliffSensors = (int)(strtod(token, &unconverted));
    token = strtok(NULL, delimiter);
    ignoreIRSensors = (int)(strtod(token, &unconverted));
    token = strtok(NULL, delimiter);
    ignoreFloorSensors = (int)(strtod(token, &unconverted));
    token = strtok(NULL, delimiter);
    pluggedIn = (int)(strtod(token, &unconverted));
 
    #ifdef debugModeOn
    // For Debugging
    dprint(term, "GOT D! %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn); 
    #endif

    //Additional included in init message but not reconfigure
    if(!robotInitialized)
    {
      token = strtok(NULL, delimiter);
      // Set initial location from ROS, in case we want to recover our last location!
      X = strtod(token, &unconverted);
      token = strtok(NULL, delimiter);
      Y = strtod(token, &unconverted);
      token = strtok(NULL, delimiter);
      Heading = strtod(token, &unconverted);
      gyroHeading = Heading;
      if (trackWidth > 0.0 && distancePerCount > 0.0){
        robotInitialized = 1;
        #ifdef debugModeOn
        dprint(term, "Initalized \n");
        #endif
      }
    }               
  }   
}  


/*
 *  clearTwistRequest()
 *  
 *  Reset the twist velocitys to 0
 *  
 */
void clearTwistRequest() {
  CommandedVelocity = 0.0;
  CommandedAngularVelocity = 0.0;
  angularVelocityOffset = 0.0;
}


/*
 *  The Main Event
 *  
 */
int main()
{
  int tm = 0;
  int loop_time = 0;
  //int i;
  int state=0;
  //int throttleStatus=0;
  
  
  simpleterm_close();
  term = fdserial_open(31, 30, 0, 115200);
  pause(1000);//Give the terminal a sec to get started
  
  sequencer_start();

  rx_count = 0; // clear the input buffer  

  #ifdef EMULATE_ROS
  pingArray[0] = 23;
  pingArray[1] = 23;
  pingArray[2] = 233;
  pingArray[3] = 24;
  pingArray[4] = 203;
  pingArray[5] = 23;
  irArray[0] = 22;
  deltaDistance=0;
  Omega = 0;
  V=0;  
  dprint(term, "Starting\n");  
  cogstart(&ROS, NULL, rstack, sizeof(rstack));
  #endif

  memset(sensorbuf, 0, 132);
  char *curbuf = sensorbuf;  
       
/////////////////////////////////////////////
//Copied from ROSInterfaceForArloBotWithDHB10
  
  // Robot description: We will get this from ROS so that it is easier to tweak between runs without reloading the Propeller EEPROM.
  // http://learn.parallax.com/activitybot/calculating-angles-rotation
  // See ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml to set or change this value
  distancePerCount = 0.0; 
  trackWidth = 0.0;

  // For Odometry
  int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
  double Heading = 0.0, X = 0.0, Y = 0.0, deltaDistance, deltaX, deltaY, V, Omega;
  int speedLeft, speedRight; 
  int throttleStatus = 0;
  int heading, deltaTicksLeft, deltaTicksRight;
  BatteryVolts = 12; //Just set it to something sane for right now

  /* Wait for ROS to give us the robot parameters,
     broadcasting 'i' until it does to tell ROS that we
     are ready */
  robotInitialized = 0; // Do not compute odometry until we have the trackWidth and distancePerCount
   
  // For PIRsensor
  #ifdef hasPIR
  int PIRhitCounter = 0;
  int personThreshhold = 15; // Must get more than this number of hits before we call it a person.
  int personDetected = 0;
  #endif
  
  // For DHB-10 Interaction See drive_speed.c for example code
  // NOTE: Because this function has a loop, ALL interaction with the DHB-10 is done in this main loop.
  // Any other cog/function that needs to affect the robot's motors will set variables that are read in this function.

  // Halt motors in case they are moving and reset all stats.
  if (drive_set_stop() )
  {
    ;// handle error
  }      
  if ( drive_rst() )
  {
    ;// handle error
  }      
  // For Debugging without ROS:
  // See ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml for most up to date values
  /*  
  trackWidth = 0.403000; // from measurement and then testing
  distancePerCount = 0.00338;
  // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544
  robotInitialized = 1;
  */  
  // Comment out above lines for use with ROS

  // Declaring variables outside of loop
  // This may or may not improve performance
  // Some of these we want to hold and use later too
  // A Buffer long enough to hold the longest line ROS may send.
  const int bufferLength = 35; // A Buffer long enough to hold the longest line ROS may send.
  char buf[bufferLength];
  int count = 0, i = 0;
    
  // To hold received commands
  CommandedVelocity = 0.0;
  CommandedAngularVelocity = 0.0;
  angularVelocityOffset = 0.0; 
  expectedLeftSpeed = 0.0; 
  expectedRightSpeed = 0.0;
  curLeftspeed = 0; 
  curRightSpeed =0;    
    
  // Listen for drive commands
  int timeoutCounter = 0;
        
//End of init from ROSInterfaceForArloBotWithDHB10  
//////////////////////////////////////////////////

  while(1)
  {    
  
     timeoutCounter++;//keep track of timoutcount
     
     check_input(term);
     if(got_one){
      //dprint(term,"%s%d",in_buf,tm);
      pars_input(); //5ms
      got_one = 0;
      timeoutCounter = 0;
      #ifdef debugModeOn
       
      #endif
    }// Timout code needs to be though out better    
    else if (timeoutCounter > ROStimeout) 
    {
        #ifdef debugModeOn
        dprint(term, "DEBUG: Stopping Robot due to serial timeout.\n");
        #endif
        expectedLeftSpeed = 0;
        expectedRightSpeed = 0;
        clearTwistRequest();
        timeoutCounter = 0; //ROStimeout; // Prevent runaway integer length          
    }     
    
      
    /* This updates the motor controller on EVERY
       round. This way even if there is no updated twist command
       from ROS, we will still account for updates in the speed limit
       from the SaftyOverride cog by recalculating the drive commands
       based on the new speed limit at every loop.

       This also allows us to have a STOP action if there is no input
       from ROS for too long.
    */

    if ( safty_check(CommandedVelocity,&expectedLeftSpeed,&expectedRightSpeed) )
    {
      #ifdef debugModeOn
      //dprint(term, "Safty_Check l=%f r=%f\n",expectedLeftSpeed,expectedRightSpeed );  
      #endif
      clearTwistRequest();//ignore twist msg if we are escaping or blocked
    }    
              
    /* to simplify communications with teh dhb-10 we send the command to go in only one place */
    /* first check if there has been a change dont overload with needless commands */
    if(curLeftspeed != expectedLeftSpeed || curRightSpeed != expectedRightSpeed && robotInitialized )
    {
      curLeftspeed = (int)expectedLeftSpeed;
      curRightSpeed = (int)expectedRightSpeed;
      //pause(dhb10OverloadPause);
      if ( drive_set_gospd(curLeftspeed,curRightSpeed) )
      {
        ;// handle error
      }    
      #ifdef debugModeOn
      dprint(term, "go speed l=%d r=%d\n",curLeftspeed,curRightSpeed );
      #endif
    }        


    // Broadcast Odometry  
    /* Some of the code below came from Dr. Rainer Hessmer's robot.pde
       The rest was heavily inspired/copied from here:
       http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
    */
    #ifdef debugModeOn
    //dprint(term, "state =  %d\n",state);
    #endif

    switch(state)
    {
      
      case 0: //Ros has not initalized the bot yet
      
        if(!robotInitialized)
        {
          state = -1; //We increment after switch putting us back to 0
          throttleStatus++;
          if(throttleStatus > 30)
          {
            // Request Robot distancePerCount and trackWidth 
            //NOTE: Python code cannot deal with a line with no divider characters on it.
            #ifdef hasPIR
            dprint(term, "i\t%d\n", personDetected);
            #else
            dprint(term, "i\t0\n");
            #endif 
            
          
            //Copied from origional code, should be reworked.
            #ifdef hasPIR
            int PIRstate = 0;
            for (i = 0; i < 5; i++) // 5 x 200ms pause = 1000 between updates
            {
              PIRstate = input(PIR_PIN); // Check sensor (1) motion, (0) no motion
              // Count positive hits and make a call:
              if (PIRstate == 0) 
              {
                PIRhitCounter = 0;
              } 
              else 
              {
              PIRhitCounter++; // Increment on each positive hit
              }
              if (PIRhitCounter > personThreshhold) 
              {
                personDetected = 1;
              } 
              else 
              {
                personDetected = 0;
              }
              pause(200); // Pause 1/5 second before repeat
            }
            #endif 
            throttleStatus=0;         
          }
        }                 
        else
        {//Start ping code etc.
          #ifdef debugModeOn
          dprint(term, "Starting Cogs\n");
          #endif
          // Start the local sensor polling cog
          ping_start();
    
          // Start Gyro polling in another cog
          #ifdef hasGyro
            gyro_start();
          #endif

          // Start safetyOverride cog: (AFTER the Motors are initialized!)
          safetyOverride_start();     
          
          timeoutCounter = 0;//clear timeout counter  
           
        }          
        break;
        
      case 1:// read speed   8ms        
        if ( drive_get_spd(&speedLeft, &speedRight) )
        {
          ;// handle error
        }    
        break;

      case 2:// read distance  8ms
        ticksLeftOld = ticksLeft;
        ticksRightOld = ticksRight;
        if ( drive_get_dist(&ticksLeft,&ticksRight) )
        {
          ;// handle error
        }          
        break;

      case 3:// read heading  7ms
        if ( drive_get_head(&heading) )
        {
          ;// handle error
        }            
        // The heading is apparently reversed in relation to what ROS expects, hence the "-heading"
        Heading = -heading * PI / 180.0; // Convert to Radians

        deltaTicksLeft = ticksLeft - ticksLeftOld;
        deltaTicksRight = ticksRight - ticksRightOld;
        deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * distancePerCount;
        deltaX = deltaDistance * (double) cos(Heading);
        deltaY = deltaDistance * (double) sin(Heading);

        X += deltaX;
        Y += deltaY;

        // http://webdelcire.com/wordpress/archives/527
        V = ((speedRight * distancePerCount) + (speedLeft * distancePerCount)) / 2;
        Omega = ((speedRight * distancePerCount) - (speedLeft * distancePerCount)) / trackWidth;
    
        break;

      case 4:// format odometery  13ms
        // Odometry for ROS
        /*
           I sending ALL of the proximity data (IR and PING sensors) to ROS
           over the "odometry" line, since it is real time data which is just as important
           as the odometry, and it seems like it would be faster to send and deal with one packet
           per cycle rather than two.

           In the propeller node I will convert this to fake laser data.
           I have two goals here:
           1. I want to be able to visualize in RVIZ what the sensors are reporting. This will help with debugging
           situations where the robot gets stalled in doorways and such due to odd sensor readings from angled
           surfaces near the sides.
           2. I also want to use at least some of this for obstacle avoidance in AMCL.
           Note that I do not think that IR and PING data will be useful for gmapping, although it is possible.
           It is just too granular and non specific. It would be nice to be able to use the PING (UltraSonic) data
           to deal with mirrors and targets below the Kinect/Xtion, but I'm not sure how practical that is.
        */
        memset(sensorbuf, 0, 132);
        sprint(sensorbuf,"o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",X,Y,Heading,gyroHeading,V,Omega);
        curbuf = sensorbuf + strlen(sensorbuf);
        break;

      case 5:// format ping       8ms
        sprint(curbuf,"{");
        curbuf = sensorbuf + strlen(sensorbuf);
        for (i = 0; i < NUMBER_OF_PING_SENSORS; i++) { // Loop through all of the sensors
          if (i > 0)
          {
           sprint(curbuf, ",");
           curbuf = sensorbuf + strlen(sensorbuf);
          }                
          sprint(curbuf, "\"p%d\":%d ", i, pingArray[i]);
          curbuf = sensorbuf + strlen(sensorbuf);
        }      
        for (i = 0; i < NUMBER_OF_IR_SENSORS; i++) // Loop through all of the sensors
        {
          if (irArray[i] > 0) // Don't pass the empty IR entries, as we know there are some.
            sprint(curbuf, ",\"i%d\":%d", i, irArray[i]);
            curbuf = sensorbuf + strlen(sensorbuf);
        }
        #ifdef hasFloorObstacleSensors
        for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) // Loop through all of the sensors
        {
          sprint(curbuf, ",\"f%d\":%d", i, floorArray[i]);
          curbuf = sensorbuf + strlen(sensorbuf);
        }
        #endif
        sprint(curbuf, "}\n");
        break;

      case 6:// xmit odometry     10ms
        #ifdef enableOutput
        dprint(term,sensorbuf); // 10 ms
        #endif 
        throttleStatus = throttleStatus + 1;
        if(throttleStatus < 10)
           state = 0;//We increment after switch
        break;

      case 7:// xmit status       16ms
        // Send a regular "status" update to ROS including information that does not need to be refreshed as often as the odometry.
        dprint(term, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%d\n", 
                  safeToProceed, safeToRecede, Escaping, abd_speedLimit, 
                  abdR_speedLimit, minDistanceSensor, 
                  BatteryVolts, RawBatVolts, 
                  cliff, floorO);
        throttleStatus = 0;
        state = 0; //We increment after switch
//        tm +=  sequencer_get(); // ~1ms or less    
//        dprint(term,"%d \n ",tm);
//        tm=0;
        break;
    } 
    
           
    state++;
    while(sequencer_get() < 16 ) {;} //16 = 60 hz

//dprint(term,"%d %d\n",loop_time,state);

//tm = sequencer_get(); // ~1ms or less   
loop_time =  sequencer_get();
tm += loop_time;
sequencer_reset(); 

 }
 return(0); 
}  


/*
 *twist message
 *message = 's,%.3f,%.3f\r' % (v, omega)
 *'s,2.000,0.000r'
 *
 *
 *startup message
 *message = 'd,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f\r' % 
             (self.track_width, self.distance_per_count, ignore_proximity, 
              ignore_cliff_sensors, ignore_ir_sensors, ignore_floor_sensors, 
               ac_power, self.lastX, self.lastY, self.lastHeading)
 *'d,0.0,0.0,1,1,1,1,1,0.0,0.0,0.0r' 
 *
 *ReConfiguer
 *message = 'd,%f,%f,%d,%d,%d,%d,%d\r' % 
             (self.track_width, self.distance_per_count, ignore_proximity, 
              ignore_cliff_sensors, ignore_ir_sensors, ignore_floor_sensors,
              ac_power)
 *
 *
 */
#ifdef EMULATE_ROS
void ROS()
{
 //pretend we are ROS and stuff the buffer 
 pause(3000); 
 strcpy(in_buf,"d,0.403,0.00338,0,0,0,0,0,0.0,0.0,0.0\r");// A Buffer long enough to hold the longest line ROS may send.
 got_one = 1; 
 pause(1000);
 while(1)
 {
    pause(100);
    strcpy(in_buf,"s,0.000,0.000\r");// A Buffer long enough to hold the longest line ROS may send.
    got_one = 1;      
 }    
}  
#endif






