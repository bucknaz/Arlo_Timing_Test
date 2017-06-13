/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "fdserial.h"
#define EMULATE_ROS
#define EMULATE_ARLO
//should we emit odometery lines to ROS terminal
#define enableOutput

//#include "mstimer.h"

// Ring buffer header file contents
// 12 bytes long
typedef struct ringbuffer_t {
  void *buffer;       // buffer should be sizeof(uint32)*buffer_capacity bytes
  uint8_t size_t;         // Size of the Item being stored in the queue
  uint8_t buffer_capacity;    
  uint8_t in,out;
  uint8_t queue_count;   // number of entries currently enqueued
  int lockId;
} RingBuffer;


/* Initialize a new RingBuffer. 
 *
 *  Returns
 *   0 - Success
 */
int RingBuffer_init(RingBuffer *rbuf, void *buffer, uint8_t buffer_capacity,uint8_t size_t);
void RingBuffer_destroy(RingBuffer *rbuf);

/* Synchronized put, will overwrite oldest entry if full */
void RingBuffer_enqueue(RingBuffer *rbuf, void *entry);

/* Synchronized get, dest must point to entry_size bytes 
 *
 * Returns
     0 - Success, new entry present at dest
     1 - Failure, queue is empty. dest undefined
 */
int RingBuffer_dequeue(RingBuffer *rbuf, void *dest);

//End ring buffer header file contents



#define MIN_MSG_DELTA 500
#define MIN_Q_DELTA 1500
#define LOOP_DELTA 100
#define MIN_MSG_DELTA 20

int nextPtime;
int testtime;
int Looptime;
int starttime;
int nextcmdtime;

fdserial *term;

//static volatile int  dt;   
//static volatile int  nextcnt;   

//    clkfreq: 80000000
//    clkmode: XTAL1+PLL16X
  //clkset(_CLKMODE,50000000);
/*
int xmain()                                    // Main function
{
  int ms = 0, tm = 0;
  int  nextcnt;   
  simpleterm_close();
  term = fdserial_open(31, 30, 0, 115200);
  pause(1000);
  dprint(term, "Starting\n");
  // Add startup code here.
  sequencer_start();

  //dt = CLKFREQ/1000; //every 1 ms
  dt = CLKFREQ/100; //every 10 ms
  //dt = CLKFREQ/10; //every 100 ms
  nextcnt = CNT + dt;
  
  nextcmdtime = (( CLKFREQ ) * MIN_MSG_DELTA) + CNT;//get the current counter value   
 
  while(1)
  {     
                   
    nextcnt = CNT + (dt-Looptime);// dt - clock cycles used
    waitcnt(nextcnt); //wait 1ms   
    ms++;                    

    starttime = CNT;       

    // Add main loop code here.
    if(ms % 100 == 0 )
    {
         dprint(term," IS printed %u %u %u\n",tm,dt, dt-Looptime);
         
         //ms=0;
    }
    else if(ms % 50 == 0 )
    {
         dprint(term,"*");
    }             
    else if(ms % 10 == 0 )// every 100ms
    {
         dprint(term,"." );
    }     

    tm = sequencer_get();
    sequencer_reset();         

       
 /*   
    if(nextcmdtime < CNT ){
      waitcnt(nextcmdtime);
    }
    if(nextcmdtime > CNT ){
      dprint(term,"!");
      nextcmdtime = (( CLKFREQ ) * MIN_MSG_DELTA) + CNT;//get the current counter value   
   }     
*/    
/*    
    //pause(20);
    Looptime = CNT - starttime ; // How many clock cycle we used   

  }  
}
*/

#define NUMBER_OF_PING_SENSORS 6
#define NUMBER_OF_IR_SENSORS 1
//#define NUMBER_OF_FLOOR_SENSORS 4

static double gyroHeading = 0.0;
double Heading = 1.0, X = 0.0, Y = 0.0, deltaDistance, V, Omega;

//These are in safety override
volatile int ignoreProximity = 0;
volatile int ignoreCliffSensors = 0;
volatile int ignoreIRSensors = 0;
volatile int ignoreFloorSensors = 0;
volatile int pluggedIn = 0;


const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script
// Maximum speed in ticks per second. Even if ROS asks us to go faster, we will not.
#define MAXIMUM_SPEED 200 // 200 is default in arlodrive too, but we may change it.


double distancePerCount = 0.0, trackWidth = 0.0;
double CommandedVelocity = 0.0;
double CommandedAngularVelocity = 0.0;
double angularVelocityOffset = 0.0, expectedLeftSpeed = 0.0, expectedRightSpeed = 0.0;
int curLeftspeed = 0, curRightSpeed =0;
int robotInitialized=0;

// declared outside main
int abd_speedLimit = MAXIMUM_SPEED;
int abdR_speedLimit = MAXIMUM_SPEED; // Reverse speed limit to allow robot to reverse fast if it is blocked in front and visa versa



int pingArray[NUMBER_OF_PING_SENSORS] = {0};
int irArray[NUMBER_OF_IR_SENSORS] = {0};
#ifdef hasFloorObstacleSensors
int floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif

static volatile char sensorbuf[132];
#define RXBUFFERLEN 40
static char rx_buf[RXBUFFERLEN];// A Buffer long enough to hold the longest line ROS may send.
static char in_buf[RXBUFFERLEN];// A Buffer long enough to hold the longest line ROS may send.
static int rx_count = 0;
static int got_one = 0;
int floorO=0;
int cliff=0;
float BatteryVolts=12.0, RawBatVolts=4.0;
int minDistanceSensor = 2;
int Escaping = 0;
int safeToProceed=1, safeToRecede=1;


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
    //else 
    //{
    //  // Not doing this on in place rotations (Velocity = 0)
    //  // Or if requested speed does not exceed maximum.
    //  CommandedVelocity = CommandedVelocity;
    //}
    
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
      if (trackWidth > 0.0 && distancePerCount > 0.0)
        robotInitialized = 1;
    }               
  }   
}  



void clearTwistRequest() {
  CommandedVelocity = 0.0;
  CommandedAngularVelocity = 0.0;
  angularVelocityOffset = 0.0;
}


/* create a state machine to select which
   task get executed each time though the loop
   
  every time though the loop
  check for twist cmd
  safty override
  send twistcmd 5ms guessing
  
  switch (state)
  {
    read dist  8ms
    read spd   8ms
    read head  7ms
    format odometery  13ms
    format ping       8ms
    format ir         2ms
    format floor      2ms
    xmit odometry     10ms
    xmit status       call it 10
  }
  state++; 
*/

int main()
{
  int tm = 0,loop_time = 0, i;
  int state=0,throttleStatus=0;
  simpleterm_close();
  term = fdserial_open(31, 30, 0, 115200);
  pause(1000);
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
  while(1)
  {    
     check_input(term);
     if(got_one){
      //dprint(term,"%s%d",in_buf,tm);
      pars_input(); //5ms
      got_one = 0;
    }
//Add code to handle timeout

    #ifdef EMULATOR
      pause(1);
    #else
    if ( safty_check(CommandedVelocity,&expectedLeftSpeed,&expectedRightSpeed) )
    {
      clearTwistRequest();//ignore twist msg if we are escaping or blocked
    }    
    #endif
              
    /* to simplify communications with teh dhb-10 we send the command to go in only one place */
    /* first check if there has been a change dont overload with needless commands */
    if(curLeftspeed != expectedLeftSpeed || curRightSpeed != expectedRightSpeed && robotInitialized )
    {
      curLeftspeed = (int)expectedLeftSpeed;
      curRightSpeed = (int)expectedRightSpeed;
      //pause(dhb10OverloadPause);
      #ifdef EMULATE_ARLO
        pause(5);
      #else
      if ( drive_set_gospd(curLeftspeed,curRightSpeed) )
      {
        ;// handle error
      }    
      #endif        
    }        


    // Broadcast Odometry  
    /* Some of the code below came from Dr. Rainer Hessmer's robot.pde
       The rest was heavily inspired/copied from here:
       http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
    */

    switch(state)
    {
      case 0: //Ros has not initalized the bot yet
        if(!robotInitialized)
        {
          state = -1; //We increment after switch putting us back to 0
        }          
        else
        {//Start ping code etc.
          // Start the local sensor polling cog
          ping_start();
    
          // Start Gyro polling in another cog
          #ifdef hasGyro
            gyro_start();
          #endif

          // Start safetyOverride cog: (AFTER the Motors are initialized!)
          safetyOverride_start();          
        }          
        break;
        
      case 1:// read speed   8ms
        #ifdef EMULATE_ARLO
        pause(8);
        #else
        if ( drive_get_spd(&speedLeft, &speedRight) )
        {
          ;// handle error
        }    
        #endif      
        break;

      case 2:// read distance  8ms
        ticksLeftOld = ticksLeft;
        ticksRightOld = ticksRight;
        #ifdef EMULATE_ARLO
        pause(8);
        #else
        if ( drive_get_dist(&ticksLeft,&ticksRight) )
        {
          ;// handle error
        }          
        #endif      
        break;

      case 3:// read heading  7ms
        #ifdef EMULATE_ARLO
        pause(7);
        #else
        if ( drive_get_head(&heading) )
        {
          ;// handle error
        }          
        #endif  
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
        dprint(term, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%d\n", safeToProceed, safeToRecede, Escaping, abd_speedLimit, abdR_speedLimit, minDistanceSensor, BatteryVolts, RawBatVolts, cliff, floorO);
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
 pause(1000); 
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


////////////////////////////////////////////////////////////////////////////////
//#include "ringbuffer.h"

int RingBuffer_init(RingBuffer *rbuf, void *buffer, uint8_t buffer_capacity, uint8_t size_t)
{
  rbuf->buffer = buffer;
  rbuf->size_t = size_t;
  rbuf->buffer_capacity = buffer_capacity;
  rbuf->in = rbuf->out = 0;
  rbuf->lockId = locknew();
  return 0;
}


void RingBuffer_destroy(RingBuffer *rbuf) {
  lockret(rbuf->lockId);
}

void RingBuffer_enqueue(RingBuffer *rbuf, void *entry)
{
  while (lockset(rbuf->lockId) != 0) { /*spin lock*/ }
  //rbuf->buffer[rbuf->in++ % rbuf->buffer_capacity] = *(uint32_t*)entry;

  memcpy(rbuf->buffer +(rbuf->in++ % rbuf->buffer_capacity)*rbuf->size_t,entry,rbuf->size_t);

  rbuf->queue_count++;
  lockclr(rbuf->lockId);
}


int RingBuffer_dequeue(RingBuffer *rbuf, void *dest)
{
  if (rbuf->queue_count < 1) {
    return 1;
  }

  while (lockset(rbuf->lockId) != 0) { /*spin lock*/ }
  // *(uint32_t*)dest = rbuf->buffer[rbuf->out++ % rbuf->buffer_capacity];
  memcpy(dest,rbuf->buffer +(rbuf->out++ % rbuf->buffer_capacity)*rbuf->size_t,rbuf->size_t);
  rbuf->queue_count--;
  lockclr(rbuf->lockId);
  return 0;
}

//Ring buffer test
//4 bytes long
typedef struct {
  uint8_t foo ;
  uint32_t bar ;
} TestStruct;
  
//TestStruct doda;
//uint8_t eight;      //1
//uint16_t sixteen;   //2
//uint32_t thirtytwo; //4
//uint32_t *pointer;  //4

#define TEST_BUF_SIZE 8
TestStruct structbuffer[TEST_BUF_SIZE];//A array 0x20 32 bytes long
RingBuffer ringbuff; //12 

uint32_t stack[40+25];

void producer();


int ringbuf_main() {
  TestStruct s;
  RingBuffer_init(&ringbuff, structbuffer, TEST_BUF_SIZE,sizeof(s));
  // start cog1 as the producer
  cogstart(&producer, NULL, stack, sizeof(stack));

  while (1) {
    //while (RingBuffer_dequeue(&ringbuff, &s) != 0) { /* spinlock */ }
   
    while (!RingBuffer_dequeue(&ringbuff, &s) )
    {
        printf("foo=%d bar=%d cnt=%d\n", s.foo, s.bar,ringbuff.queue_count);
    }    
    pause(500);
  }    
}  


void producer() {
  TestStruct newStruct;
  uint16_t fooCounter = 0;
  uint8_t barCounter = 0;

  volatile unsigned int wait_time = CLKFREQ;
  unsigned int nextcnt = wait_time + CNT;
  
  while (1) {    
    newStruct.foo = fooCounter++;
    newStruct.bar = barCounter--;
    RingBuffer_enqueue(&ringbuff, &newStruct);
    nextcnt = waitcnt2(nextcnt, wait_time);
  }      
}






/* Though to use another cog

static unsigned int ostack[128];           // Stack vars for other cog
static volatile int ocog;  
int odometry_start()
{
  odometry_stop();
  ocog = 1 + odometry(odometry, NULL, ostack, sizeof(ostack));
}

void odometry_stop()
{
  if(ocog)
  {
    cogstop(ocog -1);
    ocog = 0;
  }    
}

// Function runs in another cog
void odometry(void *par)                      
{
  term = fdserial_open(31, 30, 0, 115200);
  pause(1000);
  
  while(1)                                   
  {

  //send odometery
  //check for twist
  ;
  
  

  }
                              
}

*/
