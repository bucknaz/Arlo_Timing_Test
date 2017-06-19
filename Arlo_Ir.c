
#include "simpletools.h"
#include <stdbool.h>

#include "per_robot_settings_for_propeller_c_code.h"
#include "Arlo_mcp3008.h"

#ifdef hasMCP3008
// Sort an array


void SharpIR_sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

//long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

// Read distance and compute it
int mcp3008_IR_cm(int channel) {

    int ir_val[NB_SAMPLE];
    int distanceCM;
    int i=0;
    float current;

    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = readADC(channel, MCP3008_DINOUT_PIN, MCP3008_CLK_PIN, MCP3008_CS_PIN);
    }
    
    // Sort it 
    SharpIR_sort(ir_val,NB_SAMPLE);
    // Previous formula used by  Dr. Marcal Casas-Cartagena
    float volts = 5.031 * ir_val[NB_SAMPLE / 2] / 1024;
    distanceCM = 65*pow(volts, -1.10);
    return(distanceCM);
}


    // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
/*
int mcp3008_IR_cm(int channel) {
    int mcp3008reading = readADC(channel, MCP3008_DINOUT_PIN, MCP3008_CLK_PIN, MCP3008_CS_PIN);
   float mcp3008volts = (float) mcp3008reading * MCP3008_REFERENCE_VOLTAGE / 1024.0;
    int mcp3008cm = 27.86 * pow(mcp3008volts, -1.15); 
    return (mcp3008cm);
}
*/
#endif


