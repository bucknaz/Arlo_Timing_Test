/*
 *
 *
 *
 *
 */


#ifndef ARLO_MCP3008_H
#define ARLO_MCP3008_H

#if defined(__cplusplus)
extern "C" { 
#endif


void pinHigh(int pin);
void pinLow(int pin);
void pinInput(int pin);
void pinOutput(int pin);
int pinRead(int pin);
int pinWrite(int pin, int state);
void delay(int us);
void pinPulseHL(int pin, int d, int d1);
void pinPulseLH(int pin, int d, int d1);
int readADC(int channel, int dinout, int clk, int cs);
int readADCAverage(int channel, int dinout, int clk, int cs, int samples);


#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_MCP3008_H */
