

#ifndef ARLO_SEQUENCER_H
#define ARLO_SEQUENCER_H

#if defined(__cplusplus)
extern "C" { 
#endif

#include "simpletools.h"                      // Include simpletools

static volatile int t, dt, cog;               // Global var for cogs to share
//static unsigned int stack[40 + 25];           // Stack vars for other cog

void sequencer(void *par);                 
int sequencer_start();
void sequencer_stop();
int sequencer_get();
void sequencer_reset();
void sequencer_set(int newTime);
// Function runs in another cog
void sequencer(void *par);                      

#if defined(__cplusplus)                     
}
#endif /* __cplusplus */

#endif /* ARLO_SEQUENCER_H */
