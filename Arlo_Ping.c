
#include "simpletools.h"
#include "ping.h"
#include "per_robot_settings_for_propeller_c_code.h"
#include "Arlo_Ping.h"
#include "Arlo_Ir.h"
#include "Arlo_mcp3008.h"


extern double BatteryVolts;
extern double RawBatVolts;

// Global Storage for PING & IR Sensor Data:
int pingArray[NUMBER_OF_PING_SENSORS] = {0};

int irArray[NUMBER_OF_IR_SENSORS] = {0};

#ifdef hasFloorObstacleSensors
int floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif

// Local Sensor Polling Cog
static volatile int ping_cog;
static int ping_stack[128]; // If things get weird make this number bigger!


int ping_start()
{
  ping_stop();
  ping_cog = 1 + cogstart(&pollPingSensors, NULL, ping_stack, sizeof ping_stack);
  return(ping_cog);
}

void ping_stop()
{
  if(ping_cog)
  {
    cogstop(ping_cog -1);
    ping_cog = 0;
  }    
}


void pollPingSensors(void *par) {
    int i=0;
    //int ir = 0;
    int adc_val; //temp valu5 for calculating the battery volatage
    while (1)                                    // Repeat indefinitely
    {
        for (i = 0; i < NUMBER_OF_PING_SENSORS; i++) {
            pingArray[i] = ping_cm(FIRST_PING_SENSOR_PIN + i);
            #ifdef hasMCP3008
            // If there is also an IR sensor at this number check it too
            if (i < NUMBER_OF_IR_ON_MCP3008) {
               irArray[i] = mcp3008_IR_cm(i);
            }
            #endif
        }
        // Check battery Power
        #ifdef hasPowerMonitorCircuit
        adc_val = readADCAverage(CH_BATT, MCP3008_DINOUT_PIN, MCP3008_CLK_PIN, MCP3008_CS_PIN, BATT_SAMPLES);
        BatteryVolts = (MCP3008_REFERENCE_VOLTAGE * adc_val / 1024) * 2.799116998;
        RawBatVolts = (MCP3008_REFERENCE_VOLTAGE * adc_val / 1024) ;
        #endif

        pause(mainLoopPause);
    }
}

