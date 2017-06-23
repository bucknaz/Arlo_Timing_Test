

//Set this to test without ROS connected
#define EMULATE_ROS

//If the DHB-10 is not present set this to test code
#define EMULATE_ARLO

//Set this to emit odometry
//#define enableOutput // Do NOT comment this out when running ROS!

#define debugModeOn

// PROXIMITY SENSOR TUNING
// Use these to tune the robots responses!
// What is the maximum distance at which sensor readings should be noticed?
#define MAX_DISTANCE 50

// What is the maximum distance at which IR (Infrared) sensor readings should be noticed?
// IR sensors tend to be less reliable, so make this shorter than the MAX_DISTANCE, which applies to PING sensors.
#define IR_MAX_DISTANCE 50

// What is the minimum speed that the speed limiter should set the robot to?
#define MINIMUM_SPEED 20
// Maximum speed in ticks per second. Even if ROS asks us to go faster, we will not.
#define MAXIMUM_SPEED 200 // 200 is default in arlodrive too, but we may change it.

// THROTTLE_STOP determines how quickly the speed limit is changed.
// Make this number bigger to cause the speed to change more slowly.
#define INCREASE_THROTTLE_RATE 50
#define DECREASE_THROTTLE_RATE 5


// Settings for PING and IR (Infrared) sensors connected DIRECTLY to the Activity Board:
// SETTING: Which pin on the Activity Board is the first PING sensor connected to?
#define FIRST_PING_SENSOR_PIN 10

// QUESTION: Are you using an MCP3208 chip to read poll analog sensors from the Activity Board?
#define hasMCP3008
// Settings for MCP3008, if it has one:
// SETTING: How many IR sensors (if any) are connected to your MCP3008?
#define NUMBER_OF_IR_ON_MCP3008 1
#define MCP3008_CS_PIN 5
#define MCP3008_DINOUT_PIN 6
#define MCP3008_CLK_PIN 7
// MCP3208 reference voltage setting. I use 5.0v for the 5.0v IR sensors from Parallax
#define MCP3008_REFERENCE_VOLTAGE 5.031
// howmany samples for ir averging
#define NB_SAMPLE 25




// PROXIMITY SENSORS:
// IMPORTANT! This section is very important!
// SETTINGS: Proximity (PING & IR) Sensor count and location:
#define NUMBER_OF_PING_SENSORS 6

#define NUMBER_OF_IR_SENSORS 1


// QUESTION: Do you have PING sensors on the front of your robot?
#define hasFrontPingSensors

#define FIRST_FRONT_PING_SENSOR_NUMBER 3 // Count from 0

// Your sensors need to poll consecutively for each section!
#define HOW_MANY_FRONT_PING_SENSORS 3

// QUESTION: Do you have PING sensors on the back of your robot?
#define hasRearPingSensors

#define FIRST_REAR_PING_SENSOR_NUMBER 0 // Count from 0

#define HOW_MANY_REAR_PING_SENSORS 3

// QUESTION: Do you have IR sensors on the front of your robot?
#define hasFrontIRSensors

#define FIRST_FRONT_IR_SENSOR_NUMBER 0 // channel number Count from 0

#define HOW_MANY_FRONT_IR_SENSORS 1

// QUESTION: Do you have IR sensors on the back of your robot?
//#define hasRearIRSensors
//#define FIRST_REAR_IR_SENSOR_NUMBER 7 // Count from 0
//#define HOW_MANY_REAR_IR_SENSORS 0

// QUESTION: Do you have PING sensors on the front "upper deck" of your robot?
/* These would be facing straight forward in the middle of decks above
the deck the circle of sensors is on.
I have one on the top deck, and then one on the stand holding the Xtion sensor.
All of these will be assumed to face forward. */
//#define hasFrontUpperDeckSensors
//#define FIRST_FRONT_UPPER_SENSOR_NUMBER 10 // Count from 0
//#define HOW_MANY_FRONT_UPPER_SENSORS 0

// QUESTION: Do you have PING sensors on the REAR "upper deck" of your robot?
//#define hasRearUpperDeckSensors
//#define FIRST_REAR_UPPER_SENSOR_NUMBER 13 // Count from 0
//#define HOW_MANY_REAR_UPPER_SENSORS 1

// QUESTION: Sensor Identities:
// Put the sensor number by each label
// Comment out any sensor you do not have
// "Left" refers to the ROBOT's left, assuming he faces the way he drives "forward"

/* defining variables in a header is bad form, moved to the Arlo_SafetyOveride file */
//#define FRONT_FAR_LEFT_SENSOR 3
//#define FRONT_FAR_LEFT_HALT_DIST 10
//#define FRONT_FAR_LEFT_SLOWDWN_DIST 40

#define FRONT_NEAR_LEFT_SENSOR 3
#define FRONT_NEAR_LEFT_HALT_DIST 10
#define FRONT_NEAR_LEFT_SLOWDWN_DIST 20

// NOTE: IF you comment out the FRONT_CENTER_SENSOR, then ALL Front sensors are ignored for escaping!
#define FRONT_CENTER_SENSOR 4
#define FRONT_CENTER_HALT_DIST 10
#define FRONT_CENTER_SLOWDWN_DIST 20

#define FRONT_NEAR_RIGHT_SENSOR 5
#define FRONT_NEAR_RIGHT_HALT_DIST 5
#define FRONT_NEAR_RIGHT_SLOWDWN_DIST 20

//#define FRONT_FAR_RIGHT_SENSOR 4
//#define FRONT_FAR_RIGHT_HALT_DIST 5
//#define FRONT_FAR_RIGHT_SLOWDWN_DIST 8

//#define REAR_FAR_LEFT_SENSOR 9
//#define FAR_NEAR_LEFT_HALT_DIST 5
//#define FAR_NEAR_LEFT_SLOWDWN_DIST 8

#define REAR_NEAR_LEFT_SENSOR 0
#define REAR_NEAR_LEFT_HALT_DIST 10
#define REAR_NEAR_LEFT_SLOWDWN_DIST 20

// NOTE: IF you comment out the REAR_CENTER_SENSOR, then ALL Rear sensors are ignored for escaping!
#define REAR_CENTER_SENSOR 1
#define REAR_CENTER_HALT_DIST 10
#define REAR_CENTER_SLOWDWN_DIST 20

#define REAR_NEAR_RIGHT_SENSOR 2
#define REAR_NEAR_RIGHT_HALT_DIST 10
#define REAR_NEAR_RIGHT_SLOWDWN_DIST 20

//#define REAR_FAR_RIGHT_SENSOR 2
//#define REAR_FAR_RIGHT_HALT_DIST 12
//#define REAR_FAR_RIGHT_SLOWDWN_DIST MAX_DISTANCE


//#define FRONT_UPPER_DECK_NEAR_LEFT_SENSOR 10
//#define FRONT_UPPER_DECK_CENTER_SENSOR 11
//#define FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR 12
//#define FRONT_3D_MOUNTED_SENSOR 11
//#define REAR_UPPER_DECK_SENSOR 13

//#define REAR_3D_MOUNTED_SENSOR 13

// QUESTION: Do you want to "rename" the rear IR sensor(s)?
/* If this is set, all "rear" IR sensors will be labeled with this number
when checked. I use this because while I have 5 rear PING sensors,
I only have 1 rear IR sensor. So the escape options for the one rear
IR sensor should match the rear center PING sensor.
If you don't want to do this, just comment this setting out:
*/
//#define RENAME_REAR_IR_SENSOR REAR_CENTER_SENSOR

//#define FRONT_FAR_LEFT_IR_SENSOR 1
//#define FRONT_FAR_LEFT_IR_HALT_DIST 10
//#define FRONT_FAR_LEFT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define FRONT_NEAR_LEFT_IR_SENSOR 2
//#define FRONT_NEAR_LEFT_IR_HALT_DIST 10
//#define FRONT_NEAR_LEFT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

#define FRONT_CENTER_IR_SENSOR 0
#define FRONT_CENTER_IR_HALT_DIST 5
#define FRONT_CENTER_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define FRONT_NEAR_RIGHT_IR_SENSOR 3
//#define FRONT_NEAR_RIGHT_IR_HALT_DIST 5
//#define FRONT_NEAR_RIGHT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define FRONT_FAR_RIGHT_IR_SENSOR 4
//#define FRONT_FAR_RIGHT_IR_HALT_DIST 5
//#define FRONT_FAR_RIGHT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define REAR_FAR_LEFT_IR_SENSOR 5
//#define FAR_NEAR_LEFT_IR_HALT_DIST 5
//#define FAR_NEAR_LEFT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define REAR_NEAR_LEFT_IR_SENSOR 6
//#define REAR_NEAR_LEFT_IR_HALT_DIST 5
//#define REAR_NEAR_LEFT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

// NOTE: IF you comment out the REAR_CENTER_SENSOR, then ALL Rear sensors are ignored for escaping!
//#define REAR_CENTER_IR_SENSOR 7
//#define REAR_CENTER_IR_HALT_DIST 10
//#define REAR_CENTER_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define REAR_NEAR_RIGHT_IR_SENSOR 8
//#define REAR_NEAR_RIGHT_IR_HALT_DIST 12
//#define REAR_NEAR_RIGHT_IR_SLOWDWN_DIST IR_MAX_DISTANCE

//#define REAR_FAR_RIGHT_IR_SENSOR 9
//#define REAR_FAR_RIGHT_IR_HALT_DIST 12
//#define REAR_FAR_RIGHT_IR_SLOWDWN_DIST IR_MAX_DISTANCE




// QUESTION: Does your robot have a PIR (Passive Infrared) sensor connected to the Activity Board?
/* NOTE: I got this PIR sensor for free, and it is fun,
   but it doesn't do much for you when the robot is moving
   and sending out IR signals with the Kinect/Xtion
   and with the IR distance sensors. :)
   It is cool though to have it watch for people and alert
   ROS when someone is around. Then ROS can initialize the ArloBot,
   start up the motors with a USB Relay Board
   and start chasing them . . . or something.
   */
//#define hasPIR
// SETTING: IF you have a PIR sensor, which pin is it connected to?
//#define PIR_PIN 11

// GYROSCOPE:
// QUESTION: Does your robot have a Gyro module installed on the Activity Board?
//#define hasGyro
// Settings for Gyroscope, if it has one:
// SETTING: Activity Board pin that the SCL pin from the Gyro is connected to:
//#define GYRO_SCL_PIN 1
// SETTING: Activity Board pin that the SDA pin from the Gyro is connected to:
//#define GYRO_SDA_PIN 0

// VOLTAGE MONITORING:
// QUESTION: Do you have a voltage divider connected to the Activity Board's built in ADC to monitor voltage at the left and right motors?
#define hasPowerMonitorCircuit
// Settings for Power Monitor Circuit, if it has one:
// If these get flipped just flip the wires, or the numbers.
#define CH_BATT 7
#define BATT_SAMPLES 5
//#define LEFT_MOTOR_ADC_PIN 0
//#define RIGHT_MOTOR_ADC_PIN 1

// CLIFF SENSORS:
// QUESTION: Do you have IR "cliff" sensors mounted to the front of the robot?
//#define hasCliffSensors
// SETTING: Which IR sensor number (counting from 0) is the first cliff sensor?
//#define FIRST_CLIFF_SENSOR 5
// SETTING: How many cliff sensors are there?
//#define NUMBER_OF_CLIFF_SENSORS 2
// SETTING: What is the maximum distance before the robot should consider it to be a cliff?
// at 30 it was stalling when crossing half inch bumps.
//#define FLOOR_DISTANCE 40

// FLOOR OBSTACLE SENSORS:
// QUESTION: Do you have digital infrared sensors on the front of the robot?
//#define hasFloorObstacleSensors
//#define FIRST_FLOOR_SENSOR_PIN 7
//#define NUMBER_OF_FLOOR_SENSORS 4

/* END OF QUESTION SECTION!
That is all, you are ready to attempt to build this code and load it
onto your Propeller Activity board!
*/




/* These settings affect the speed at which the "loop" on the robot runs.
If it runs too fast we overwhelm serial connections and crash things or get garbage.
Running too slow will reduce the responsiveness of the robot.
Remember though that going faster and "hanging" isn't more responsive.
*/
#define dhb10OverloadPause 1 // Pause before each read/write to DHB10 to avoid overloading it.
// 1 seems to work fine. 2 seems perfectly safe without any ill affects. I suggest filing a github issue before increasing this.
#define mainLoopPause 10 // Pause after each main loop. 10 seems to work fine.
// The total loop "pause" time is (dhb10OverloadPause * 4 ) + mainLoopPause
// So you can probalby decrease mainLoopPause if you increase dhb10OverloadPause
/* Timeout setting. After this period (loops) the robot will stop if it
has not received a twist command from ROS.
The loop speed is determined by the above two settings, so it isn't a strict time.
*/
#define ROStimeout 120 // 10 = about 1 second if dhb10OverloadPause === 2 && mainLoopPause === 10

/* Enable this to turn on extra debugging information,
for use with the
/home/chrisl8/catkin_ws/src/ArloBot/scripts/direct2PropSerialTest.sh
script.
Do not try to use it with ROS, as the extra output will confuse it. */


/* You can disable this for use with the
~/catkin_ws/src/ArloBot/scripts/direct2PropSerialTest.sh
script in order to disable the normal sensor data lines.
Sometimes this is helpful to clean up the output for debugging.
This MUST be enabled for ROS to work though!
*/

