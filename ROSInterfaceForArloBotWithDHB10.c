// TODO for DHB-10: Go over EACH line and remove unneeded code relating to old way arlodrive worked

// TODO for DHB-10: How does this change the Python ROS node, since we don't have to monitor the Propellery
// TODO for DHB-10: before turning on motor power?
// TODO for DHB-10: Or do we?
// TODO for DHB-10: If not, how do we build the propeller node to work with both? Or do we?

// TODO for DHB-10: Run through all of the ROS by Example test scripts to see how good our odemtry is,
// TODO for DHB-10: and see if we need to adjust any settings, and prove that the new setup is better!

// TODO for DHB-10: In the Parallax learn examples they put a piezo speaker on the Propeller board
// TODO for DHB-10: and beep it when the program boots, for status. Should I add something like this?
// TODO for DHB-10: Beep or LED blink or something?

// TODO for DHB-10: Finally, clean up all other "TODO" lines in this code!

/* ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!
NOTE: This code is for the DHB-10 Motor Controller that comes with the new Parallax Arlo kit.
You MUST edit the settings in
~/.arlobot/per_robot_settings_for_propeller_c_code.h
based on the physical layout of your robot!
For each QUESTION:
UNCOMMENT '#define' lines for any included items,
COMMENT '#define' lines for anything that is not included.
For each SETTING:
Set the variable as required, noting that usually these are ignored if the preceding QUESTION is commented out.

Example, My robot has a "Thing1", but not a "Thing2"
*/
//#define hasThingOne
//#define hasTHingTwo

/* Just like that, comment out the "has" line for things you do not have,
and if you do have the thing, adjust the numbers on the other definition as needed.
By using the #define lines, code for items you do not have is never seen by the compiler and is never even loaded on the Propeller bard, saving memory. */



#include "per_robot_settings_for_propeller_c_code.h"
/* If SimpleIDE build fails because the above file is missing,
open up the "Project Manager", then the "Compiler" tab,
and fix the path to your ~/.arlobot/ folder
under Other Compiler Options
and/or copy the above file from the dotfiles folder
to your ~/.arlobot folder.
You could also just move the files in the dotfiles folder into
the folder with this file, but future "git pull" updates
may erase your changes.*/

/*
Full details on how to use the DHB-10 Motor Controller on the Parallax Arlo Robot platform with a
Propeller Activity Board can be found here:
http://learn.parallax.com/tutorials/robot/arlo/arlo-activity-board-brain
I highly suggets you work through the instructions there and run the example programs and tests before using this code.
*/

/*
This is the code to run on a Parallax Propeller based Activity Board
in order to interface ROS with an ArloBot.

Author: Chris L8 https://github.com/chrisl8
URL: https://github.com/chrisl8/ArloBot

The ROS Node for this code is called propellerbot_node.py
and can be found in the arlobot_bringup package from the above URL.

Special thanks to Dr. Rainer Hessmer. Much of this code is based on his work at
https://code.google.com/p/drh-robotics-ros/

Please also see these sites which helped me tremendously with the formulas:
http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
http://webdelcire.com/wordpress/archives/527

And of course the entire point of this is to interface with ROS, so read about everything ROS here:
http://wiki.ros.org/

All code here heavily borrowed from everywhere code can be found! :)
*/

/* SimpleIDE Options
 * Everything here is the default for a new SimpleIDE project except for "Enable Pruning."
 * Project Options:
 * Board Type: ACTIVITYBOARD
 * Compiler Type: C - This is C code
 * Memory Model: CMM Main RAM Compact - My code does not fit into LMM, and there is no LMM version of arlodrive.h
 * Optimization: -Os Size - When I change this to "Speed" I get strange behavior.
 * Compiler:
 * CHECK - 32bit Double
 * CHECK - Enable Pruning - This does not make much difference, but a little.
 * Other Compiler Options: -std=c99 - this is part of the default SimpleIDE New project
 * Linker:
 * CHECK - Math lib - required for floating point math!
 * nothing else checked or added under the Linker tab.
 */

#include "simpletools.h"
#include "fdserial.h"
//#include <stdbool.h>
#include "mstimer.h"




#include "Arlo_Ping.h"
#include "Arlo_SafetyOverride.h"
#include "Arlo_DBH-10.h"
#include "Arlo_Ir.h"
#include "Arlo_Gyro.h"





/*
Full details on how to use the DHB-10 Motor Controller on the Parallax Arlo Robot platform with a
Propeller Activity Board can be found here:
http://learn.parallax.com/tutorials/robot/arlo/arlo-activity-board-brain
I highly suggets you work through the instructions there and run the example programs and tests before using this code.
*/


//#include "arlodrive.h"




// See ~/.arlobot/per_robot_settings_for_propeller_c_code.h to adjust MAXIMUM_SPEED
int abd_speedLimit = MAXIMUM_SPEED;
int abdR_speedLimit = MAXIMUM_SPEED; // Reverse speed limit to allow robot to reverse fast if it is blocked in front and visa versa

fdserial *term;
//serial *term;

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script


// Global Storage for PING & IR Sensor Data:

double BatteryVolts = 4.69;
double RawBatVolts = 0;






#ifdef hasFloorObstacleSensors
int floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif


// We need this even if there is no Gyro. Just pass 0.0 if it doesn't exist.
// Otherwise I would have to modify the propeller_node.py too.
static double gyroHeading = 0.0;

// For DHB-10 Interaction See drive_speed.c for example code
//char dhb10_reply[DHB10_LEN];

// We need some error checking for dhb10_com(char *)
//dhb10_com will return the reply/status from the dhb-10 board.
//we need to check for an error and handle it if we get one
//else proceed as normale.
// If verbose mode we will get a nak followed by the error string
// if not verbose we recive the number followed by " E"
/*
Verbose = "ERROR " + "- Overflow"
non verbose = "1"+ " E"

Nack                    byte "ERROR ", 0
Overflow                byte "1", 0, "- Overflow", 0
*/


/**************************************************************/
/*
static volatile int t, dt, cog;               // Global var for cogs to share
static unsigned int stack[40 + 25];           // Stack vars for other cog

void ms_timer(void *par);                 

int mstime_start()
{
  mstime_stop();
  cog = 1 + cogstart(ms_timer, NULL, stack, sizeof(stack));
}

void mstime_stop()
{
  if(cog)
  {
    cogstop(cog -1);
    cog = 0;
  }    
}

int mstime_get()
{
  return t;
}

void mstime_reset()
{
  t = 0;
}

void mstime_set(int newTime)
{
  t = newTime;
}

// Function runs in another cog
void ms_timer(void *par)                      
{
  dt = CLKFREQ/1000; //get the clk cycles per sec / 1000
  int ticks = CNT;//get the current counter value
  while(1)                                   
  {
    waitcnt(ticks+=dt);                              
    t++; //increments every ms 
  
  }                            
}
*/

//5 mhz xtal
//64 KB EEPROM for program and data storage
// pause is in ms 
/*
1000 1 per sec 
500  2 times a sec
250  4 times a sec
125  8 times a sec

dhb-10 needs update atlease once per sec

dhb10OverloadPause 2 * 4 = 8ms
mainLoopPause 10          10 ms

looptime around  20 ms
ROStimeout 10 times though loop befor doing time out, about 200 ms
throttleStatus every tenth time though the loop
run though the loop at 10 times a second, 
10 ms = 100 hz
100 ms = 10hz
1000 ms = 1hz

125 go
135 dist
145 speed
155 head
200 internals

1000/times per sec

dhb-10 max rate = ~20 ms

dbh-10comm will ensure no comands are sent faster then every 25ms





waitcnt(((clkfreq/1000) * num_ms ) + cnt ); //delay cog num_ms millisecond
read the twist every time though the loop.
Only send 00 imidiatly else
 
send the latest twist if it changed to the dhb-10 every 100ms 10 times a sec
send unchanged messages to the dhb-10 every 250ms 4 times a sec
get the dist 10ms later
get the speed 10ms later
get the head 10ms later and emit the odometry

send the internals every 200 ms

    switch (t)
    {
     case 125:// 1/8
     case 250:// 1/4 2/8
     case 375:// 3/8
     case 500:// 1/2 2/4 4/8 
     case 625:// 5/8
     case 750:// 3/4 6/8
     case 875:// 7/8
     case 1000:// 1 2/2 4/4 8/8
     
        mstime_reset();
        break    
    }        
*/

/*************************************************************/


/**************************************************************/
/*

mstime_start(); //Start the mstimer in another cog
int time = mstime_get(); //get the elapsed time

int main()                                   
{
  mstime_start();
  int dt = CLKFREQ;
  int t = CNT;
  while(1)
  {
    int time = mstime_get();
    print("time = %d\n", time);                    
    waitcnt(t += dt);
  }    
}

pace timer
  indicate when ok to send new command
  indicate time to retreive odometry
  indicate time to send slow state


*/
int Looptime;
int starttime;
int tm;
//60 - 75ms Total loop time
//0 - 1 ms imput
//0 - 1 ms safty check
//28 ms odometry collections
//23 ms odometry printing
//16 ms status printing
volatile char sensorbuf[133];


int main() {
    char *curbuf;

    simpleterm_close(); // Close simplex serial terminal
    term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection
mstime_start();

    // Robot description: We will get this from ROS so that it is easier to tweak between runs without reloading the Propeller EEPROM.
    // http://learn.parallax.com/activitybot/calculating-angles-rotation
    // See ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml to set or change this value
    double distancePerCount = 0.0, trackWidth = 0.0;

    // For Odometry
    int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
    double Heading = 0.0, X = 0.0, Y = 0.0, deltaDistance, deltaX, deltaY, V, Omega;
    int speedLeft, speedRight, throttleStatus = 0, heading, deltaTicksLeft, deltaTicksRight;
    BatteryVolts = 12; //Just set it to something sane for right now

    /* Wait for ROS to give us the robot parameters,
       broadcasting 'i' until it does to tell ROS that we
       are ready */
    int robotInitialized = 0; // Do not compute odometry until we have the trackWidth and distancePerCount

    // For PIRsensor
    int PIRhitCounter = 0;
    int personThreshhold = 15; // Must get more than this number of hits before we call it a person.
    int personDetected = 0;


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
    
       trackWidth = 0.403000; // from measurement and then testing
       distancePerCount = 0.00338;
       // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544
       robotInitialized = 1;
    
    // Comment out above lines for use with ROS

    // Declaring variables outside of loop
    // This may or may not improve performance
    // Some of these we want to hold and use later too
    // A Buffer long enough to hold the longest line ROS may send.
    const int bufferLength = 35; // A Buffer long enough to hold the longest line ROS may send.
    char buf[bufferLength];
    int count = 0, i = 0;
        
    // Preinitialized loop
    while (robotInitialized == 0) {            
        dprint(term, "i\t%d\n", personDetected); // Request Robot distancePerCount and trackWidth NOTE: Python code cannot deal with a line with no divider characters on it.
        pause(10); // Give ROS time to respond, but not too much or we bump into other stuff that may be coming in from ROS.
        if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
            count = 0;
            while (count < bufferLength) {
                buf[count] = fdserial_rxTime(term, 100); // fdserial_rxTime will time out. Otherwise a spurious character on the line will cause us to get stuck forever
                if (buf[count] == '\r' || buf[count] == '\n')
                    break;
                count++;
            }

            if (buf[0] == 'd') {
                char *token;
                token = strtok(buf, delimiter);
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
                token = strtok(NULL, delimiter);
                // Set initial location from ROS, in case we want to recover our last location!
                X = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                Y = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                Heading = strtod(token, &unconverted);
                gyroHeading = Heading;
                if (trackWidth > 0.0 && distancePerCount > 0.0)
                    robotInitialized = 1;
            }
        } else {
            #ifdef hasPIR
                int PIRstate = 0;
                for (i = 0; i < 5; i++) { // 5 x 200ms pause = 1000 between updates
                    PIRstate = input(PIR_PIN); // Check sensor (1) motion, (0) no motion
                    // Count positive hits and make a call:
                    if (PIRstate == 0) {
                        PIRhitCounter = 0;
                    } else {
                        PIRhitCounter++; // Increment on each positive hit
                    }
                    if (PIRhitCounter > personThreshhold) {
                        personDetected = 1;
                    } else {
                        personDetected = 0;
                    }
                    pause(200); // Pause 1/5 second before repeat
                }
            #else
                pause(1000); // Longer pauses when robot is uninitialized
            #endif
            }
    }

    // Start the local sensor polling cog
    ping_start();
    
    // Start Gyro polling in another cog
    #ifdef hasGyro
        gyro_start();
    #endif

    // Start safetyOverride cog: (AFTER the Motors are initialized!)
    safetyOverride_start();
    

    // To hold received commands
    double CommandedVelocity = 0.0;
    double CommandedAngularVelocity = 0.0;
    double angularVelocityOffset = 0.0, expectedLeftSpeed = 0.0, expectedRightSpeed = 0.0;
    int curLeftspeed = 0, curRightSpeed =0;

    void clearTwistRequest() {
        CommandedVelocity = 0.0;
        CommandedAngularVelocity = 0.0;
        angularVelocityOffset = 0.0;
    }

    // Listen for drive commands
    int timeoutCounter = 0;
    while (1) {
 // Input 0 ms  no messages waiting
    
        timeoutCounter++;

        if (fdserial_rxReady(term) != 0) // Non blocking check for data in the input buffer
        {
            count = 0;
            while (count < bufferLength) 
            {
                buf[count] = readChar(term);
                if (buf[count] == '\r' || buf[count] == '\n')
                    break;
                count++;
            }
      
            if (buf[0] == 's') 
            {
                char *token;
                token = strtok(buf, delimiter);
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
                else 
                {
                  // Not doing this on in place rotations (Velocity = 0)
                  // Or if requested speed does not exceed maximum.
                  CommandedVelocity = CommandedVelocity;
                }
                // These only need to be calculated once per twist msg
                expectedLeftSpeed =  CommandedVelocity - angularVelocityOffset;
                expectedRightSpeed = CommandedVelocity + angularVelocityOffset;
                expectedLeftSpeed =  expectedLeftSpeed / distancePerCount;
                expectedRightSpeed = expectedRightSpeed / distancePerCount;                
            } 
            else if (buf[0] == 'd') 
            {
                char *token;
                token = strtok(buf, delimiter);
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
                dprint(term, "GOT D! %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn); // For Debugging
                #endif
                
            }
            timeoutCounter = 0;
        }
        else if (timeoutCounter > ROStimeout) 
        {
            #ifdef debugModeOn
            dprint(term, "DEBUG: Stopping Robot due to serial timeout.\n");
            #endif
            expectedLeftSpeed = 0;
            expectedRightSpeed = 0;
            clearTwistRequest();
            timeoutCounter = ROStimeout; // Prevent runaway integer length          
        }


        /* This updates the motor controller on EVERY
           round. This way even if there is no updated twist command
           from ROS, we will still account for updates in the speed limit
           from the SaftyOverride cog by recalculating the drive commands
           based on the new speed limit at every loop.

           This also allows us to have a STOP action if there is no input
            from ROS for too long.
        */
//safty check 0 ms        
        if ( safty_check(CommandedVelocity,&expectedLeftSpeed,&expectedRightSpeed) )
        {
          clearTwistRequest();//ignore twist msg if we are escaping or blocked
        }    
              
        /* to simplify communications with teh dhb-10 we send the command to go in only one place */
        /* first check if there has been a change dont overload with needless commands */
        if(curLeftspeed != expectedLeftSpeed || curRightSpeed != expectedRightSpeed)
        {
          curLeftspeed = (int)expectedLeftSpeed;
          curRightSpeed = (int)expectedRightSpeed;
          pause(dhb10OverloadPause);
          if ( drive_set_gospd(curLeftspeed,curRightSpeed) )
          {
            ;// handle error
          }            
        }        
        
       
        // Broadcast Odometry  
        /* Some of the code below came from Dr. Rainer Hessmer's robot.pde
           The rest was heavily inspired/copied from here:
        http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
        */
// odometry collection 28 ms  3ms in pause about 8ms per call
        ticksLeftOld = ticksLeft;
        ticksRightOld = ticksRight;

        // Get the distance traveled   
        pause(dhb10OverloadPause);     
        if ( drive_get_dist(&ticksLeft,&ticksRight) )
        {
          ;// handle error
        }          
                  
        // Get the current speed
        pause(dhb10OverloadPause);
        if ( drive_get_spd(&speedLeft, &speedRight) )
        {
          ;// handle error
        }          
        
        //Get the gyro if we have one        
        #ifdef hasGyro
           set_rotating(speedLeft,speedRight);
        #endif
        

        //Get the heading
        pause(dhb10OverloadPause);
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
//odmetry printing 23 ms
        #ifdef enableOutput
        
dprint(term,"%d ",tm);
mstime_reset(); 
          memset(sensorbuf, 0, 132);

          //dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", X, Y, Heading, gyroHeading, V, Omega);
          sprint(sensorbuf,"o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", X, Y, Heading, gyroHeading, V, Omega);
          curbuf = sensorbuf + strlen(sensorbuf);

/* 
          // Send the PING/IR sensor data as a JSON packet:
          dprint(term, "{");
          for (i = 0; i < NUMBER_OF_PING_SENSORS; i++) { // Loop through all of the sensors
              if (i > 0)
                dprint(term, ",");
              dprint(term, "\"p%d\":%d ", i, pingArray[i]);
          }
          for (i = 0; i < NUMBER_OF_IR_SENSORS; i++) { // Loop through all of the sensors
            if (irArray[i] > 0) // Don't pass the empty IR entries, as we know there are some.
                dprint(term, ",\"i%d\":%d", i, irArray[i]);
          }
          #ifdef hasFloorObstacleSensors
          for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) { // Loop through all of the sensors
            dprint(term, ",\"f%d\":%d", i, floorArray[i]);
          }
          #endif
          dprint(term, "}\n");
*/
// from here
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
          for (i = 0; i < NUMBER_OF_IR_SENSORS; i++) { // Loop through all of the sensors
            if (irArray[i] > 0) // Don't pass the empty IR entries, as we know there are some.
                sprint(curbuf, ",\"i%d\":%d", i, irArray[i]);
		           curbuf = sensorbuf + strlen(sensorbuf);
          }
          #ifdef hasFloorObstacleSensors
          for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) { // Loop through all of the sensors
            sprint(curbuf, ",\"f%d\":%d", i, floorArray[i]);
	          curbuf = sensorbuf + strlen(sensorbuf);
          }
          #endif
          sprint(curbuf, "}\n");

//To here takes 6 -7 ms 
        #endif
dprint(term,"%s",sensorbuf);
//dprint(term,"\n");
tm = mstime_get();

        // Send a regular "status" update to ROS including information that does not need to be refreshed as often as the odometry.
        throttleStatus = throttleStatus + 1;
//status printing 16ms
        if (throttleStatus > 9) {
            dprint(term, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%d\n", safeToProceed, safeToRecede, Escaping, abd_speedLimit, abdR_speedLimit, minDistanceSensor, BatteryVolts, RawBatVolts, cliff, floorO);
            throttleStatus = 0;
        }
        
        #ifdef debugModeOn
          dprint(term, "DEBUG: %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn);
        #endif
        pause(mainLoopPause); // Maximum read frequency.
    }
}

/*
mstime_start();
mstime_reset(); 
tm = mstime_get();
dprint(term,"%d ",tm);

*/






